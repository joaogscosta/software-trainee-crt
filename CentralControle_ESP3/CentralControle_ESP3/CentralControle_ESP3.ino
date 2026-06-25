// =============================================================================
// CentralControle_ESP3.ino — Código da Central de Controle (Maleta)
// Capital Rocket Team — Software Trainee 2026
//
// Responsável pela coordenação dos estados de missão, interface de solo do
// operador, gerenciamento de redundância LoRa/Cabo e saída de telemetria CSV.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include "../../Shared_Config/config.h"
#include "../../Shared_Config/packet_protocol.h"

// =============================================================================
// HABILITAÇÃO DO MODO SIMULADOR INTERATIVO
// =============================================================================
// Defina como 1 para testar toda a lógica da central de controle via terminal 
// serial (sem necessidade de rádio, cabo ou outro ESP conectado).
// Defina como 0 para rodar no hardware real.
#define SIMULATION_MODE 1

// =============================================================================
// ENUMS E MAQUINAS DE ESTADO
// =============================================================================
typedef enum : uint8_t {
  STATE_STANDBY   = 0x00,
  STATE_FILLING   = 0x01,
  STATE_ARMED     = 0x02,
  STATE_IGNITION  = 0x03,
  STATE_FLIGHT    = 0x04,
  STATE_ABORT     = 0x05
} MissionState;

typedef enum : uint8_t {
  LINK_OK   = 0x00,
  LINK_LOST = 0x01
} LinkStateESP1;

const char* stateNames[] = {
  "STANDBY",
  "FILLING",
  "ARMED",
  "IGNITION",
  "FLIGHT",
  "ABORT"
};

// =============================================================================
// VARIÁVEIS GLOBAIS
// =============================================================================
MissionState g_missionState   = STATE_STANDBY;
CommState    g_gsCommState    = COMM_RADIO;
LinkStateESP1 g_rocketLinkState = LINK_LOST;
bool         g_fillValveClosedSent = false;

// Pacotes mais recentes recebidos dos outros nós
RocketTelemetryPacket g_lastRocketPkt;
GSTelemetryPacket     g_lastGSPkt;

// Controle de Watchdogs e Temporizadores
unsigned long g_lastESP1PacketTime = 0;
unsigned long g_lastESP2PacketTime = 0;
unsigned long g_lastCableRetryTime = 0;
unsigned long g_lastCSVPrintTime   = 0;
unsigned long g_ignitionStartTime  = 0;

// Drivers de Comunicação LoRa SPI
RH_RF95 g_rf95(PIN_LORA_CS, PIN_LORA_DIO0);
RHReliableDatagram g_manager(g_rf95, NODE_ESP3_CENTRAL);

// =============================================================================
// VARIÁVEIS E FUNÇÕES EXCLUSIVAS DA SIMULAÇÃO (Se ativa)
// =============================================================================
#if SIMULATION_MODE
bool   g_simArmSwitch          = false;
bool   g_simIgnitionBtn        = false;
bool   g_simAbortBtn           = false;
bool   g_simESP1Connected      = true;
bool   g_simESP2LoRaConnected  = true;
bool   g_simESP2CableConnected = true;
float  g_simTankMass           = TANK_EMPTY_KG;
float  g_simManifoldPres       = 0.0f;
float  g_simAltitude           = 0.0f;
float  g_simAccelZ             = -9.81f;
bool   g_simGpsFix             = true;

void updateSimulation();
void printSimulationHelp();
#endif

// Declaração de funções auxiliares
bool readArmSwitch();
bool readIgnitionButton();
bool readAbortButton();
void processComms();
void manageCommsRedundancy();
void runMissionStateMachine();
void printCSVLine();
bool sendGSCommand(CommandID cmd);
void triggerAbort();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  // Inicialização do canal de debug e do CSV
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 3000); // Aguarda conexão da porta serial por no máximo 3s

  // Imprime o cabeçalho CSV requerido uma única vez
  Serial.println(CSV_HEADER);

  // Inicialização do cabo de redundância UART
  Serial2.begin(CABLE_BAUD_RATE, SERIAL_8N1, PIN_CABLE_RX, PIN_CABLE_TX);

  // Se estiver em simulação, avisa no console e imprime ajuda
#if SIMULATION_MODE
  printSimulationHelp();
#else
  // Configuração dos Pinos dos botões físicos
  pinMode(PIN_CHAVE_ARM, INPUT_PULLUP);
  pinMode(PIN_BTN_IGNITION, INPUT_PULLUP);
  pinMode(PIN_BTN_ABORT, INPUT_PULLUP);

  // Inicialização do barramento LoRa SPI
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

  // Inicialização do gerenciador RadioHead
  if (!g_manager.init()) {
    Serial.println("[CRITICAL] Falha ao inicializar o gerenciador RadioHead!");
  } else {
    g_rf95.setFrequency(LORA_FREQUENCY / 1e6);
    g_rf95.setSpreadingFactor(LORA_SPREADING_FACTOR);
    g_rf95.setSignalBandwidth(LORA_BANDWIDTH);
    g_rf95.setCodingRate4(LORA_CODING_RATE);
    g_rf95.setTxPower(LORA_TX_POWER, false);
    g_manager.setRetries(RH_MAX_RETRIES);
    g_manager.setTimeout(RH_TIMEOUT_MS);
    Serial.println("[INFO] RadioHead inicializado com sucesso.");
  }
#endif

  // Inicialização de timestamps para evitar falso disparo de watchdog no boot
  unsigned long now = millis();
  g_lastESP1PacketTime = now;
  g_lastESP2PacketTime = now;
  g_lastCableRetryTime = now;
  g_lastCSVPrintTime   = now;

  // Limpa as structs de telemetria iniciais
  memset(&g_lastRocketPkt, 0, sizeof(RocketTelemetryPacket));
  memset(&g_lastGSPkt, 0, sizeof(GSTelemetryPacket));
  g_lastGSPkt.tank_mass_kg = TANK_EMPTY_KG;
}

// =============================================================================
// LOOP PRINCIPAL
// =============================================================================
void loop() {
#if SIMULATION_MODE
  updateSimulation(); // Atualiza a física simulada e entrada do teclado
#endif

  processComms();           // Recebe dados do rádio e do cabo
  manageCommsRedundancy();  // Gerencia falhas de comunicação e fallbacks
  runMissionStateMachine(); // Executa a máquina de estados de missão principal

  // Impressão periódica da telemetria CSV (a cada 300ms)
  if (millis() - g_lastCSVPrintTime >= 300) {
    g_lastCSVPrintTime = millis();
    printCSVLine();
  }
}

// =============================================================================
// MÉTODOS DE LEITURA DOS BOTÕES DA MALETA
// =============================================================================
bool readArmSwitch() {
#if SIMULATION_MODE
  return g_simArmSwitch;
#else
  // Retorna true se a chave física estiver ligada (pino aterrado via pull-up)
  return (digitalRead(PIN_CHAVE_ARM) == LOW);
#endif
}

bool readIgnitionButton() {
#if SIMULATION_MODE
  return g_simIgnitionBtn;
#else
  return (digitalRead(PIN_BTN_IGNITION) == LOW);
#endif
}

bool readAbortButton() {
#if SIMULATION_MODE
  return g_simAbortBtn;
#else
  return (digitalRead(PIN_BTN_ABORT) == LOW);
#endif
}

// =============================================================================
// PROCESSAMENTO DE PACOTES DE ENTRADA (RÁDIO E CABO)
// =============================================================================
void processComms() {
#if !SIMULATION_MODE
  // --- 1. Recepção via LoRa ---
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (g_manager.recvfromAck(buf, &len, &from)) {
    if (len >= 2) {
      // Validação do Checksum CRC8
      if (calcCRC8(buf, len - 1) == buf[len - 1]) {
        uint8_t pktType = buf[0];

        if (pktType == PKT_TELEMETRY_ROCKET && from == NODE_ESP1_ROCKET) {
          memcpy(&g_lastRocketPkt, buf, sizeof(RocketTelemetryPacket));
          g_lastESP1PacketTime = millis();
          g_rocketLinkState = LINK_OK;
        } 
        else if (pktType == PKT_TELEMETRY_GS && from == NODE_ESP2_GS) {
          memcpy(&g_lastGSPkt, buf, sizeof(GSTelemetryPacket));
          g_lastESP2PacketTime = millis();
          
          // Se recebemos telemetria pelo rádio, restaura para o modo primário
          if (g_gsCommState == COMM_CABLE) {
            g_gsCommState = COMM_RADIO;
            Serial.println("[COMM_RESTORE] Retorno automático do link rádio com ESP2.");
          }
        }
      }
    }
  }

  // --- 2. Recepção via Cabo (Serial2) ---
  while (Serial2.available() >= (int)sizeof(GSTelemetryPacket)) {
    // Procura sincronia verificando o byte identificador do tipo de pacote
    if (Serial2.peek() == PKT_TELEMETRY_GS) {
      uint8_t serialBuf[sizeof(GSTelemetryPacket)];
      Serial2.readBytes(serialBuf, sizeof(serialBuf));

      // Validação do Checksum CRC-8
      if (calcCRC8(serialBuf, sizeof(serialBuf) - 1) == serialBuf[sizeof(serialBuf) - 1]) {
        memcpy(&g_lastGSPkt, serialBuf, sizeof(GSTelemetryPacket));
        g_lastESP2PacketTime = millis();
        // Não transiciona automaticamente para COMM_RADIO ao receber dados no cabo,
        // apenas garante a recepção dos dados no estado de fallback ativo.
      }
    } else {
      // Descarte de byte dessincronizado
      Serial2.read();
    }
  }
#endif
}

// =============================================================================
// GERENCIADOR DE REDUNDÂNCIA E WATCHDOGS
// =============================================================================
void manageCommsRedundancy() {
  unsigned long now = millis();

  // --- Watchdog 1: Monitoramento do Foguete (ESP1) ---
  if (now - g_lastESP1PacketTime > WATCHDOG_ESP1_MS) {
    if (g_rocketLinkState != LINK_LOST) {
      g_rocketLinkState = LINK_LOST;
      Serial.println("[WARNING] Conexão LoRa com o FOGUETE (ESP1) perdida!");
    }
  } else {
    g_rocketLinkState = LINK_OK;
  }

  // --- Watchdog 2: Monitoramento da Ground Station (ESP2) ---
  if (g_gsCommState == COMM_RADIO) {
    // Se passar do timeout configurado (3s), assume que perdeu o LoRa e ativa cabo
    if (now - g_lastESP2PacketTime > WATCHDOG_ESP2_MS) {
      g_gsCommState = COMM_CABLE;
      Serial.println("[COMM_FALLBACK] Telemetria LoRa do ESP2 perdida. Migrando para Cabo (Serial2)...");
      g_lastESP2PacketTime = now; // Reinicia o temporizador para dar fôlego ao cabo
    }
  } 
  else if (g_gsCommState == COMM_CABLE) {
    // Se o cabo também expirar o timeout (3s adicionais), ativa o aborto automático
    if (now - g_lastESP2PacketTime > CABLE_TIMEOUT_MS) {
      g_gsCommState = COMM_ABORT;
      Serial.println("[COMM_CRITICAL] Falha total no Cabo e Rádio com ESP2! ABORTO AUTOMÁTICO DISPARADO!");
      triggerAbort();
    }

    // Ping de teste para tentar reestabelecer o rádio a cada 30 segundos
    if (now - g_lastCableRetryTime > CABLE_RETRY_LORA_MS) {
      g_lastCableRetryTime = now;
      Serial.println("[COMM_RETRY] Tentando reestabelecer rádio LoRa com ESP2...");

#if !SIMULATION_MODE
      PingPacket ping;
      ping.packet_type = PKT_PING;
      ping.timestamp_ms = now;
      ping.crc8 = calcCRC8((uint8_t*)&ping, sizeof(ping) - 1);

      // Envia diretamente via rádio para ver se a GS escuta
      bool ok = g_manager.sendtoWait((uint8_t*)&ping, sizeof(ping), NODE_ESP2_GS);
      if (ok) {
        g_gsCommState = COMM_RADIO;
        g_lastESP2PacketTime = now;
        Serial.println("[COMM_RESTORE] Link de rádio reestabelecido com ESP2!");
      } else {
        Serial.println("[COMM_RETRY] Sem resposta do ESP2 no rádio. Mantendo modo Cabo.");
      }
#else
      // Simulação de reestabelecimento automático se o LoRa virtual for reconectado
      if (g_simESP2LoRaConnected) {
        g_gsCommState = COMM_RADIO;
        g_lastESP2PacketTime = now;
        Serial.println("[COMM_RESTORE] Link de rádio reestabelecido com ESP2!");
      } else {
        Serial.println("[COMM_RETRY] Sem resposta do ESP2 no rádio. Mantendo modo Cabo.");
      }
#endif
    }
  }
}

// =============================================================================
// MÁQUINA DE ESTADOS DA MISSÃO PRINCIPAL
// =============================================================================
void runMissionStateMachine() {
  bool armSwitch   = readArmSwitch();
  bool ignitionBtn = readIgnitionButton();
  bool abortBtn    = readAbortButton();

  // 1. Interrupção por Aborto Manual (Maior prioridade)
  if (abortBtn && g_missionState != STATE_ABORT) {
    Serial.println("[ABORT_MANUAL] Aborto comandado manualmente pelo operador!");
    triggerAbort();
    return;
  }

  // 2. Interrupção por Segurança de Pressão ou Massa (Ground Station)
  if (g_missionState == STATE_FILLING || g_missionState == STATE_ARMED) {
    if (g_lastGSPkt.manifold_bar >= PRESSURE_MAX_BAR) {
      Serial.println("[ABORT_AUTO] Pressão máxima atingida no manifold! ABORTO AUTOMÁTICO.");
      triggerAbort();
      return;
    }
  }

  // 3. Estrutura Case Principal da State Machine
  switch (g_missionState) {
    case STATE_STANDBY:
      // Operador ativa a abertura do enchimento
      // Na prática, pode ser disparado via botões ou entrada de interface
      break;

    case STATE_FILLING:
      // Monitora enchimento do tanque
      if (g_lastGSPkt.tank_mass_kg - TANK_EMPTY_KG >= MASS_TARGET_KG) {
        if (!g_fillValveClosedSent) {
          Serial.println("[INFO] Massa alvo de N2O alcançada. Fechando válvula de preenchimento.");
          sendGSCommand(CMD_CLOSE_FILL);
          g_fillValveClosedSent = true;
        }
      }

      // Para armar exige-se: tanque cheio, chave ARM ativada e GPS fix
      if (armSwitch && (g_lastGSPkt.tank_mass_kg - TANK_EMPTY_KG >= MASS_TARGET_KG)) {
        if (g_lastRocketPkt.gps_lat != 0.0f && g_lastRocketPkt.gps_lon != 0.0f) {
          Serial.println("[ARMED] Sistema ARMADO! Pronto para ignição.");
          sendGSCommand(CMD_ARM);
          g_missionState = STATE_ARMED;
        } else {
          // Alerta periódico no console de que o GPS não fixou
          static unsigned long lastGpsWarning = 0;
          if (millis() - lastGpsWarning > 3000) {
            Serial.println("[ARM_ALERT] Não é possível armar: Foguete (ESP1) sem GPS fix!");
            lastGpsWarning = millis();
          }
        }
      }
      break;

    case STATE_ARMED:
      // Desarmar sistema se a chave física for desativada
      if (!armSwitch) {
        Serial.println("[DISARM] Chave de armar desativada. Retornando para Standby.");
        sendGSCommand(CMD_DISARM);
        g_missionState = STATE_STANDBY;
      }

      // Acionamento da Ignição
      if (ignitionBtn) {
        Serial.println("[IGNITION] Comando de ignição enviado!");
        sendGSCommand(CMD_IGNITION);
        g_missionState = STATE_IGNITION;
        g_ignitionStartTime = millis();
      }
      break;

    case STATE_IGNITION:
      // Transiciona para voo livre se o foguete reportar mudança no estado de voo 
      // ou ultrapassar um limiar de aceleração vertical Z
      if (g_lastRocketPkt.flight_state == 0x02 || g_lastRocketPkt.accel_z > 15.0f) {
        Serial.println("[FLIGHT] Voo ativo detectado por aceleração! Central em modo voo.");
        g_missionState = STATE_FLIGHT;
      }

      // Se passar 5 segundos e o motor não acender (ausência de aceleração), desarma
      if (millis() - g_ignitionStartTime > 5000) {
        Serial.println("[IGNITION_TIMEOUT] Falha no acendimento do motor. Desarmando...");
        sendGSCommand(CMD_DISARM);
        g_missionState = STATE_ARMED;
      }
      break;

    case STATE_FLIGHT:
      // Monitoramento apenas. Não atua em relés exceto se houver aborto de emergência.
      break;

    case STATE_ABORT:
      // Mantém a purga aberta continuamente enviando o sinal de aborto
      static unsigned long lastAbortSend = 0;
      if (millis() - lastAbortSend > 500) {
        sendGSCommand(CMD_ABORT);
        lastAbortSend = millis();
      }
      break;
  }
}

// =============================================================================
// ENVIO SEGURO DE COMANDOS PARA A GROUND STATION (ESP2)
// =============================================================================
bool sendGSCommand(CommandID cmd) {
  CommandPacket pkt;
  pkt.packet_type = PKT_COMMAND;
  pkt.command_id  = cmd;
  pkt.crc8        = calcCRC8((uint8_t*)&pkt, sizeof(pkt) - 1);

#if SIMULATION_MODE
  // Em modo simulação, o comando apenas simula que chegou na GS
  Serial.print("[SIM_TX] Enviado comando: 0x");
  Serial.println(cmd, HEX);
  return true;
#else
  if (g_gsCommState == COMM_RADIO) {
    // Tenta enviar com rádio LoRa (espera ACK)
    bool ok = g_manager.sendtoWait((uint8_t*)&pkt, sizeof(pkt), NODE_ESP2_GS);
    if (ok) {
      g_lastESP2PacketTime = millis();
      return true;
    } else {
      // Falha no LoRa: força transição para cabo imediatamente e reenvia
      Serial.println("[COMM_TX_ERROR] Erro no envio LoRa. Chaveando para cabo...");
      g_gsCommState = COMM_CABLE;
      g_lastESP2PacketTime = millis();
      Serial2.write((uint8_t*)&pkt, sizeof(pkt));
      return true;
    }
  } 
  else if (g_gsCommState == COMM_CABLE) {
    // Envia direto na UART redundante (Serial2)
    Serial2.write((uint8_t*)&pkt, sizeof(pkt));
    return true;
  }
  return false;
#endif
}

// =============================================================================
// ATIVAÇÃO DO ABORTO GERAL
// =============================================================================
void triggerAbort() {
  g_missionState = STATE_ABORT;
  sendGSCommand(CMD_ABORT);
}

// =============================================================================
// IMPRESSÃO DE LINHA CSV NO FORMATO COMPARTILHADO
// =============================================================================
void printCSVLine() {
  Serial.print(stateNames[g_missionState]);       Serial.print(",");
  Serial.print(millis());                         Serial.print(",");
  Serial.print(g_lastRocketPkt.altitude_m, 1);    Serial.print(",");
  Serial.print(g_lastRocketPkt.accel_z, 2);       Serial.print(",");
  Serial.print(g_lastRocketPkt.pressure_hpa, 2);  Serial.print(",");
  Serial.print(g_lastGSPkt.tank_mass_kg, 4);      Serial.print(",");
  Serial.print(g_lastGSPkt.manifold_bar, 2);      Serial.print(",");
  
  switch (g_gsCommState) {
    case COMM_RADIO: Serial.println("COMM_RADIO"); break;
    case COMM_CABLE: Serial.println("COMM_CABLE"); break;
    case COMM_ABORT: Serial.println("COMM_ABORT"); break;
    default:         Serial.println("UNKNOWN");    break;
  }
}

// =============================================================================
// LÓGICA DO SIMULADOR (INTERATIVIDADE VIA PORTA SERIAL)
// =============================================================================
#if SIMULATION_MODE
void printSimulationHelp() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("    CRT SOFTWARE SIMULADOR CENTRAL (ESP3) ATIVADO       "));
  Serial.println(F("========================================================"));
  Serial.println(F("Comandos interativos (envie um caractere pelo terminal):"));
  Serial.println(F("  'f' - Abrir válvula de enchimento e iniciar (FILLING)"));
  Serial.println(F("  'a' - Chave ARM (LIGADA / DESLIGADA)"));
  Serial.println(F("  'i' - Pressionar botão de Ignição (quando ARMED)"));
  Serial.println(F("  'x' - Pressionar botão de Aborto"));
  Serial.println(F("  '1' - Desconectar/Conectar LoRa do Foguete (ESP1)"));
  Serial.println(F("  '2' - Desconectar/Conectar LoRa do ESP2 (Ground Station)"));
  Serial.println(F("  '3' - Desconectar/Conectar Cabo do ESP2 (Ground Station)"));
  Serial.println(F("  'g' - Ligar/Desligar Fix do GPS no Foguete"));
  Serial.println(F("  'r' - Resetar simulação para o estado Standby"));
  Serial.println(F("========================================================\n"));
}

void updateSimulation() {
  unsigned long now = millis();
  static unsigned long lastSimUpdate = 0;
  
  // Atualização física e lógica simulada a 10 Hz (cada 100ms)
  if (now - lastSimUpdate < 100) return;
  lastSimUpdate = now;

  // 1. Processamento de comandos digitados no console serial
  if (Serial.available() > 0) {
    char key = Serial.read();
    
    // Descartar quebras de linha pendentes
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
      Serial.read();
    }

    switch (key) {
      case 'f':
        if (g_missionState == STATE_STANDBY) {
          g_missionState = STATE_FILLING;
          g_fillValveClosedSent = false;
          Serial.println(F("[SIM_EVENT] Operador iniciou o enchimento (FILLING)."));
        }
        break;
      case 'a':
        g_simArmSwitch = !g_simArmSwitch;
        Serial.print(F("[SIM_EVENT] Chave ARM física setada para: "));
        Serial.println(g_simArmSwitch ? F("ARMED (LIGADA)") : F("DISARMED (DESLIGADA)"));
        break;
      case 'i':
        g_simIgnitionBtn = true;
        Serial.println(F("[SIM_EVENT] Botão físico de ignição pressionado!"));
        break;
      case 'x':
        g_simAbortBtn = true;
        Serial.println(F("[SIM_EVENT] Botão físico de ABORTO emergencial pressionado!"));
        break;
      case '1':
        g_simESP1Connected = !g_simESP1Connected;
        Serial.print(F("[SIM_EVENT] Status do rádio do Foguete (ESP1): "));
        Serial.println(g_simESP1Connected ? F("ONLINE") : F("OFFLINE"));
        break;
      case '2':
        g_simESP2LoRaConnected = !g_simESP2LoRaConnected;
        Serial.print(F("[SIM_EVENT] Link LoRa com ESP2 (Ground Station): "));
        Serial.println(g_simESP2LoRaConnected ? F("ONLINE") : F("OFFLINE"));
        break;
      case '3':
        g_simESP2CableConnected = !g_simESP2CableConnected;
        Serial.print(F("[SIM_EVENT] Cabo Serial2 físico com ESP2: "));
        Serial.println(g_simESP2CableConnected ? F("CONECTADO") : F("DESCONECTADO"));
        break;
      case 'g':
        g_simGpsFix = !g_simGpsFix;
        Serial.print(F("[SIM_EVENT] Sinal de GPS do foguete: "));
        Serial.println(g_simGpsFix ? F("COM SINAL (FIX)") : F("SEM SINAL"));
        break;
      case 'r':
        g_missionState = STATE_STANDBY;
        g_gsCommState = COMM_RADIO;
        g_rocketLinkState = LINK_OK;
        g_simArmSwitch = false;
        g_simIgnitionBtn = false;
        g_simAbortBtn = false;
        g_simESP1Connected = true;
        g_simESP2LoRaConnected = true;
        g_simESP2CableConnected = true;
        g_simTankMass = TANK_EMPTY_KG;
        g_simManifoldPres = 0.0f;
        g_simAltitude = 0.0f;
        g_simAccelZ = -9.81f;
        g_simGpsFix = true;
        g_lastESP1PacketTime = now;
        g_lastESP2PacketTime = now;
        g_fillValveClosedSent = false;
        Serial.println(F("[SIM_EVENT] Simulador resetado para Standby inicial."));
        break;
    }
  }

  // 2. Simulação da telemetria do Foguete (ESP1)
  if (g_simESP1Connected) {
    g_lastRocketPkt.packet_type = PKT_TELEMETRY_ROCKET;
    g_lastRocketPkt.timestamp_ms = now;

    if (g_simGpsFix) {
      g_lastRocketPkt.gps_lat = -22.9068f; // Iacanga-SP
      g_lastRocketPkt.gps_lon = -47.0616f;
    } else {
      g_lastRocketPkt.gps_lat = 0.0f;
      g_lastRocketPkt.gps_lon = 0.0f;
    }

    if (g_missionState == STATE_FLIGHT) {
      static unsigned long flightStartTime = 0;
      if (flightStartTime == 0) flightStartTime = now;

      float t = (now - flightStartTime) / 1000.0f; // tempo de voo em segundos
      if (t < 5.0f) {
        g_lastRocketPkt.flight_state = 0x02; // BURN
        g_simAccelZ = 28.0f - (t * 2.0f);
        g_simAltitude += g_simAccelZ * 0.1f * 10.0f;
      } else if (t < 15.0f) {
        g_lastRocketPkt.flight_state = 0x03; // COASTING
        g_simAccelZ = -9.81f;
        float velocity = 28.0f - (t - 5.0f) * 9.81f;
        if (velocity > 0) g_simAltitude += velocity * 0.1f;
      } else {
        g_lastRocketPkt.flight_state = 0x05; // DESCENT
        g_simAccelZ = -1.5f;
        if (g_simAltitude > 0.0f) {
          g_simAltitude -= 4.0f;
        } else {
          g_simAltitude = 0.0f;
          g_missionState = STATE_STANDBY; // Retorna para standby após pouso
          flightStartTime = 0;
        }
      }
    } else {
      g_simAltitude = 0.0f;
      g_simAccelZ = -9.81f;
      g_lastRocketPkt.flight_state = 0x01; // STANDBY_ON_PAD
    }

    g_lastRocketPkt.altitude_m = g_simAltitude;
    g_lastRocketPkt.accel_z = g_simAccelZ;
    g_lastRocketPkt.pressure_hpa = 1013.25f - (g_simAltitude / 8.3f);
    
    // Atualiza watchdog do ESP1
    g_lastESP1PacketTime = now;
  }

  // 3. Simulação da telemetria da Ground Station (ESP2)
  bool canReceiveFromESP2 = false;
  if (g_gsCommState == COMM_RADIO && g_simESP2LoRaConnected) canReceiveFromESP2 = true;
  if (g_gsCommState == COMM_CABLE && g_simESP2CableConnected) canReceiveFromESP2 = true;

  if (canReceiveFromESP2) {
    g_lastGSPkt.packet_type = PKT_TELEMETRY_GS;

    if (g_missionState == STATE_FILLING) {
      // Simulação do tanque enchendo lentamente
      if (g_simTankMass - TANK_EMPTY_KG < MASS_TARGET_KG) {
        g_simTankMass += 0.08f;     // Enchimento a 80g por ciclo
        g_simManifoldPres += 1.5f;   // Pressão subindo no preenchimento
      }
    } else if (g_missionState == STATE_ABORT) {
      // Esvaziamento rápido pós-aborto
      if (g_simTankMass > TANK_EMPTY_KG) g_simTankMass -= 0.3f;
      if (g_simTankMass < TANK_EMPTY_KG) g_simTankMass = TANK_EMPTY_KG;
      if (g_simManifoldPres > 0.0f)      g_simManifoldPres -= 4.5f;
      if (g_simManifoldPres < 0.0f)      g_simManifoldPres = 0.0f;
    }

    g_lastGSPkt.tank_mass_kg = g_simTankMass;
    g_lastGSPkt.oxidant_mass_kg = g_simTankMass - TANK_EMPTY_KG;
    g_lastGSPkt.manifold_bar = g_simManifoldPres;
    g_lastGSPkt.gs_state = (uint8_t)g_missionState;
    g_lastGSPkt.comm_state = (uint8_t)g_gsCommState;

    // Atualiza watchdog do ESP2
    g_lastESP2PacketTime = now;
  }

  // Reseta pulso de clique rápido dos botões
  g_simIgnitionBtn = false;
  g_simAbortBtn = false;
}
#endif
