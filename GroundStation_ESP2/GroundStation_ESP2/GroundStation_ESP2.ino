// =============================================================================
// GroundStation_ESP32.ino — Código da Ground Station (Estação de Solo)
// Capital Rocket Team — Software Trainee 2026
//
// Responsável pelo controle de válvulas na rampa, leitura de sensores (pressão
// e massa) e execução da segurança autônoma por limite de pressão e watchdog.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <HX711.h>
#include "../../Shared_Config/config.h"
#include "../../Shared_Config/packet_protocol.h"

// =============================================================================
// HABILITAÇÃO DO MODO SIMULADOR INTERATIVO
// =============================================================================
// Defina como 1 para testar a Ground Station localmente no terminal sem sensores
// ou outro microcontrolador conectado.
// Defina como 0 para rodar com o hardware real conectado.
#define SIMULATION_MODE 1

// Pino extra provisório para o Relé de Ignição (Livre no ESP32 da rampa)
#define PIN_RELE_IGNITION 26

// =============================================================================
// ENUMS E ESTADOS (CONVENÇÕES: UPPER_SNAKE_CASE)
// =============================================================================
typedef enum : uint8_t {
  STATE_STANDBY         = 0x00,
  STATE_ABASTECIMENTO   = 0x01,
  STATE_LIMITE_ATINGIDO = 0x02,
  STATE_ABORTO          = 0x05
} EstadoSolo;

// =============================================================================
// VARIÁVEIS GLOBAIS (CONVENÇÕES: g_prefixo + camelCase)
// =============================================================================
EstadoSolo g_estadoAtual         = STATE_STANDBY;
CommState  g_commState           = COMM_RADIO;
HX711      g_escala;

unsigned long g_lastESP3PacketTime  = 0;
unsigned long g_lastTelemetryTime   = 0;
unsigned long g_ignitionStartTime   = 0;
bool          g_isIgniting          = false;

// Configuração do LoRa SPI
RH_RF95 g_rf95(PIN_LORA_CS, PIN_LORA_DIO0);
RHReliableDatagram g_manager(g_rf95, NODE_ESP2_GS);

// =============================================================================
// VARIÁVEIS E FUNÇÕES EXCLUSIVAS DA SIMULAÇÃO (Se ativa)
// =============================================================================
#if SIMULATION_MODE
float g_simMass        = 0.0f;
float g_simPressure    = 0.0f;

void updateSimulation();
#endif

// Declaração de funções auxiliares
float lerPressao();
float lerPeso();
void processIncomingPackets();
void handleCommand(uint8_t type, uint8_t commandId);
void sendTelemetry();
void executeSafeState();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 3000);

  // Inicialização do cabo Serial2 para redundância
  Serial2.begin(CABLE_BAUD_RATE, SERIAL_8N1, PIN_CABLE_RX, PIN_CABLE_TX);

  // Configuração dos pinos das Válvulas e Relés
  pinMode(PIN_RELE_PURGA_NA, OUTPUT);
  pinMode(PIN_RELE_VENT, OUTPUT);
  pinMode(PIN_RELE_PURGA_NF, OUTPUT);
  pinMode(PIN_RELE_VALVULA_FILL, OUTPUT);
  pinMode(PIN_RELE_IGNITION, OUTPUT);

  // Define o estado seguro inicial (Válvulas fechadas, Purga aberta para despressurizar)
  executeSafeState();

#if SIMULATION_MODE
  Serial.println(F("[INFO] Ground Station iniciada em MODO SIMULADOR."));
#else
  // Inicialização da balança (HX711)
  g_escala.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
  g_escala.tare();

  // Inicialização do rádio LoRa SPI
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

  if (!g_manager.init()) {
    Serial.println(F("[CRITICAL] Falha ao inicializar LoRa na Ground Station!"));
  } else {
    g_rf95.setFrequency(LORA_FREQUENCY / 1e6);
    g_rf95.setSpreadingFactor(LORA_SPREADING_FACTOR);
    g_rf95.setSignalBandwidth(LORA_BANDWIDTH);
    g_rf95.setCodingRate4(LORA_CODING_RATE);
    g_rf95.setTxPower(LORA_TX_POWER, false);
    g_manager.setRetries(RH_MAX_RETRIES);
    g_manager.setTimeout(RH_TIMEOUT_MS);
    Serial.println(F("[INFO] LoRa da Ground Station pronto."));
  }
#endif

  unsigned long now = millis();
  g_lastESP3PacketTime = now;
  g_lastTelemetryTime  = now;
}

// =============================================================================
// LOOP PRINCIPAL
// =============================================================================
void loop() {
#if SIMULATION_MODE
  updateSimulation();
#endif

  // --- 1. SEGURANÇA AUTÔNOMA DE PRESSÃO (Primeira prioridade!) ---
  if (lerPressao() >= PRESSURE_MAX_BAR) {
    if (g_estadoAtual != STATE_ABORTO) {
      Serial.println(F("[WARNING] Pressão crítica excedida no manifold! ABORTANDO..."));
      g_estadoAtual = STATE_ABORTO;
    }
  }

  // --- 2. WATCHDOG DE COMUNICAÇÃO (Perda completa de link) ---
  // Se o ESP2 estiver em abastecimento ou armado e ficar sem pings/comandos por 3s
  if (g_estadoAtual == STATE_ABASTECIMENTO || g_estadoAtual == STATE_LIMITE_ATINGIDO) {
    if (millis() - g_lastESP3PacketTime > WATCHDOG_ESP2_MS) {
      g_estadoAtual = STATE_ABORTO;
      g_commState = COMM_ABORT;
      Serial.println(F("[CRITICAL] Watchdog expirado sem comunicação com ESP3! ABORTANDO POR SEGURANÇA."));
    }
  }

  // --- 3. ATUAÇÃO DO PULSO DE IGNIÇÃO ---
  if (g_isIgniting) {
    if (millis() - g_ignitionStartTime >= IGNITION_PULSE_MS) {
      g_isIgniting = false;
      digitalWrite(PIN_RELE_IGNITION, LOW); // Corta o pulso de ignição após 1s
      Serial.println(F("[INFO] Pulso de ignição finalizado."));
    }
  }

  // --- 4. ATUAÇÃO DOS ESTADOS ---
  switch (g_estadoAtual) {
    case STATE_STANDBY:
      // Mantém válvulas fechadas, purga aberta por segurança
      digitalWrite(PIN_RELE_VALVULA_FILL, LOW);
      digitalWrite(PIN_RELE_VENT, LOW);
      digitalWrite(PIN_RELE_PURGA_NA, LOW); // LOW = desenergizado = Aberto para purgar
      digitalWrite(PIN_RELE_PURGA_NF, LOW);
      break;

    case STATE_ABASTECIMENTO:
      // Válvula fill aberta, purga NA energizada (fechada) para segurar o oxidante
      digitalWrite(PIN_RELE_VALVULA_FILL, HIGH);
      digitalWrite(PIN_RELE_VENT, LOW);
      digitalWrite(PIN_RELE_PURGA_NA, HIGH); // HIGH = energizado = Fechado para acumular
      digitalWrite(PIN_RELE_PURGA_NF, LOW);

      // Transição automática por massa alvo batida
      if (lerPeso() - TANK_EMPTY_KG >= MASS_TARGET_KG) {
        g_estadoAtual = STATE_LIMITE_ATINGIDO;
        Serial.println(F("[INFO] Massa alvo atingida. Fechando válvula de abastecimento."));
      }
      break;

    case STATE_LIMITE_ATINGIDO:
      // Válvula fill fecha. Purga NA continua energizada (fechada) aguardando ignição/armamento
      digitalWrite(PIN_RELE_VALVULA_FILL, LOW);
      digitalWrite(PIN_RELE_VENT, LOW);
      digitalWrite(PIN_RELE_PURGA_NA, HIGH);
      digitalWrite(PIN_RELE_PURGA_NF, LOW);
      break;

    case STATE_ABORTO:
      executeSafeState();
      break;
  }

  processIncomingPackets(); // Trata mensagens recebidas (LoRa ou Cabo)
  sendTelemetry();          // Envia dados para o ESP3 periodicamente
}

// =============================================================================
// FUNÇÃO DE SEGURANÇA / ABORTO
// =============================================================================
void executeSafeState() {
  digitalWrite(PIN_RELE_VALVULA_FILL, LOW); // Fecha a linha de entrada
  digitalWrite(PIN_RELE_VENT, HIGH);         // Abre o Vent para aliviar pressão
  digitalWrite(PIN_RELE_PURGA_NA, LOW);     // Abre Purga NA (desenergizada = aberta)
  digitalWrite(PIN_RELE_PURGA_NF, HIGH);     // Abre Purga NF (energizada = aberta)
}

// =============================================================================
// RECEPÇÃO DE COMANDOS (LORA OU CABO)
// =============================================================================
void processIncomingPackets() {
#if !SIMULATION_MODE
  // --- 1. Recepção via Rádio LoRa ---
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (g_manager.recvfromAck(buf, &len, &from)) {
    if (len >= 2 && from == NODE_ESP3_CENTRAL) {
      if (calcCRC8(buf, len - 1) == buf[len - 1]) {
        g_commState = COMM_RADIO;
        g_lastESP3PacketTime = millis();
        handleCommand(buf[0], buf[1]);
      }
    }
  }

  // --- 2. Recepção via Cabo (Serial2) ---
  while (Serial2.available() >= (int)sizeof(CommandPacket)) {
    if (Serial2.peek() == PKT_COMMAND) {
      uint8_t serialBuf[sizeof(CommandPacket)];
      Serial2.readBytes(serialBuf, sizeof(serialBuf));

      if (calcCRC8(serialBuf, sizeof(serialBuf) - 1) == serialBuf[sizeof(serialBuf) - 1]) {
        g_commState = COMM_CABLE;
        g_lastESP3PacketTime = millis();
        CommandPacket* cmdPkt = (CommandPacket*)serialBuf;
        handleCommand(cmdPkt->packet_type, cmdPkt->command_id);
      }
    } 
    else if (Serial2.peek() == PKT_PING) {
      uint8_t serialBuf[sizeof(PingPacket)];
      Serial2.readBytes(serialBuf, sizeof(serialBuf));

      if (calcCRC8(serialBuf, sizeof(serialBuf) - 1) == serialBuf[sizeof(serialBuf) - 1]) {
        g_commState = COMM_CABLE;
        g_lastESP3PacketTime = millis();
      }
    } 
    else {
      Serial2.read(); // Limpa bytes corrompidos
    }
  }
#else
  // Simulador local: processa comandos emulados caso o ESP3 envie algo
  // Na simulação local, a máquina de estados reage diretamente à física/teclado do ESP3.
#endif
}

// =============================================================================
// TRATAMENTO DE COMANDOS RECEBIDOS
// =============================================================================
void handleCommand(uint8_t type, uint8_t commandId) {
  if (type == PKT_PING) {
    // Apenas mantém o temporizador de comunicação resetado
    return;
  }

  if (type == PKT_COMMAND) {
    switch (commandId) {
      case CMD_OPEN_VENT:
        digitalWrite(PIN_RELE_VENT, HIGH);
        break;
      case CMD_CLOSE_VENT:
        digitalWrite(PIN_RELE_VENT, LOW);
        break;
      case CMD_OPEN_PURGA:
        digitalWrite(PIN_RELE_PURGA_NA, LOW); // Abre NA
        digitalWrite(PIN_RELE_PURGA_NF, HIGH); // Abre NF
        break;
      case CMD_CLOSE_PURGA:
        digitalWrite(PIN_RELE_PURGA_NA, HIGH); // Fecha NA
        digitalWrite(PIN_RELE_PURGA_NF, LOW); // Fecha NF
        break;
      case CMD_OPEN_FILL:
        if (g_estadoAtual == STATE_STANDBY) {
          g_estadoAtual = STATE_ABASTECIMENTO;
          Serial.println(F("[CMD] Iniciando abastecimento (FILLING)."));
        }
        break;
      case CMD_CLOSE_FILL:
        digitalWrite(PIN_RELE_VALVULA_FILL, LOW);
        if (g_estadoAtual == STATE_ABASTECIMENTO) {
          g_estadoAtual = STATE_LIMITE_ATINGIDO;
          Serial.println(F("[CMD] Válvula de abastecimento fechada."));
        }
        break;
      case CMD_ARM:
        // Armamento remoto aceito
        Serial.println(F("[CMD] Sistema Armado para Ignição!"));
        break;
      case CMD_DISARM:
        if (g_estadoAtual != STATE_ABORTO) {
          g_estadoAtual = STATE_STANDBY;
          Serial.println(F("[CMD] Sistema desarmado de volta ao Standby."));
        }
        break;
      case CMD_IGNITION:
        if (!g_isIgniting) {
          g_isIgniting = true;
          g_ignitionStartTime = millis();
          digitalWrite(PIN_RELE_IGNITION, HIGH); // Aciona o relé de ignição por 1s
          Serial.println(F("[CMD_IGNITION] Ativando relé de ignição por 1s!"));
        }
        break;
      case CMD_ABORT:
        g_estadoAtual = STATE_ABORTO;
        Serial.println(F("[CMD_ABORT] Aborto comandado remotamente!"));
        break;
    }
  }
}

// =============================================================================
// ENVIO DE TELEMETRIA
// =============================================================================
void sendTelemetry() {
  unsigned long now = millis();
  if (now - g_lastTelemetryTime >= GS_TELEMETRY_INTERVAL) {
    g_lastTelemetryTime = now;

    GSTelemetryPacket pkt;
    pkt.packet_type = PKT_TELEMETRY_GS;
    pkt.tank_mass_kg = lerPeso();
    pkt.oxidant_mass_kg = pkt.tank_mass_kg - TANK_EMPTY_KG;
    pkt.manifold_bar = lerPressao();

    // Monta o bitmask das válvulas baseado no estado atual dos pinos
    uint8_t valves = 0;
    if (digitalRead(PIN_RELE_PURGA_NA) == HIGH)     valves |= (1 << 0);
    if (digitalRead(PIN_RELE_VENT) == HIGH)          valves |= (1 << 1);
    if (digitalRead(PIN_RELE_PURGA_NF) == HIGH)       valves |= (1 << 2);
    if (digitalRead(PIN_RELE_VALVULA_FILL) == HIGH)     valves |= (1 << 3);
    // Válvula Vent Manifold opcional
    pkt.valve_state = valves;

    pkt.current_a   = 0.0f;
    pkt.gs_state    = (uint8_t)g_estadoAtual;
    pkt.comm_state  = (uint8_t)g_commState;
    pkt.crc8        = calcCRC8((uint8_t*)&pkt, sizeof(pkt) - 1);

#if !SIMULATION_MODE
    if (g_commState == COMM_RADIO) {
      g_manager.sendtoWait((uint8_t*)&pkt, sizeof(pkt), NODE_ESP3_CENTRAL);
    } 
    else if (g_commState == COMM_CABLE) {
      Serial2.write((uint8_t*)&pkt, sizeof(pkt));
    }
#endif
  }
}

// =============================================================================
// LEITURA DOS SENSORES (REAL OU SIMULADO)
// =============================================================================
float lerPressao() {
#if SIMULATION_MODE
  return g_simPressure;
#else
  int reading = analogRead(PIN_ACS712); // Pino correspondente ao sensor de pressão
  float voltage = (reading / 4095.0f) * 3.3f;
  // Converte a leitura de tensão (0 a 3.3V) para escala (0 a 100 bar)
  return (voltage / 3.3f) * 100.0f;
#endif
}

float lerPeso() {
#if SIMULATION_MODE
  return g_simMass + TANK_EMPTY_KG;
#else
  if (g_escala.is_ready()) {
    return g_escala.get_units(5);
  }
  return TANK_EMPTY_KG;
#endif
}

// =============================================================================
// FÍSICA E LÓGICA DO SIMULADOR (Se ativa)
// =============================================================================
#if SIMULATION_MODE
void updateSimulation() {
  static unsigned long lastSimUpdate = 0;
  unsigned long now = millis();
  
  if (now - lastSimUpdate < 100) return;
  lastSimUpdate = now;

  // Trata comandos do teclado para simular eventos que viriam do ESP3
  if (Serial.available() > 0) {
    char key = Serial.read();

    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
      Serial.read();
    }

    switch (key) {
      case 'f':
        handleCommand(PKT_COMMAND, CMD_OPEN_FILL);
        break;
      case 'c':
        handleCommand(PKT_COMMAND, CMD_CLOSE_FILL);
        break;
      case 'a':
        handleCommand(PKT_COMMAND, CMD_ARM);
        break;
      case 'i':
        handleCommand(PKT_COMMAND, CMD_IGNITION);
        break;
      case 'x':
        handleCommand(PKT_COMMAND, CMD_ABORT);
        break;
      case 'r':
        g_estadoAtual = STATE_STANDBY;
        g_commState = COMM_RADIO;
        g_simMass = 0.0f;
        g_simPressure = 0.0f;
        g_lastESP3PacketTime = now;
        Serial.println(F("[SIM_EVENT] Simulador do ESP2 resetado para Standby."));
        break;
    }
  }

  // Simulação física dos sensores baseado nos relés virtuais
  if (g_estadoAtual == STATE_ABASTECIMENTO) {
    if (digitalRead(PIN_RELE_VALVULA_FILL) == HIGH) {
      if (g_simMass < MASS_TARGET_KG) {
        g_simMass += 0.08f;
        g_simPressure += 1.5f;
      }
    }
  } 
  else if (g_estadoAtual == STATE_ABORTO) {
    if (g_simMass > 0.0f) {
      g_simMass -= 0.3f;
      if (g_simMass < 0.0f) g_simMass = 0.0f;
    }
    if (g_simPressure > 0.0f) {
      g_simPressure -= 4.5f;
      if (g_simPressure < 0.0f) g_simPressure = 0.0f;
    }
  }

  // Mantém o watchdog resetado em modo de simulação local para permitir testes manuais
  g_lastESP3PacketTime = now;
}
#endif