// =============================================================================
// Foguete_ESP1.ino — Computador de Bordo do Foguete
// Capital Rocket Team — Software Trainee 2026
//
// Responsabilidades:
//   - Ler sensores: BMP280 (barômetro), MPU-6050 (IMU), GY-NEO6MV2 (GPS)
//   - Detectar apogeu via Moving Average da altitude
//   - Acionar servo (paraquedas via CO2) no apogeu
//   - Empacotar e enviar telemetria via LoRa para ESP3
//   - Gerenciar State Machine de voo
//
// Pinos: ver Shared_Config/config.h
// Protocolo: ver Shared_Config/packet_protocol.h
// =============================================================================

#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP280.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

#include "../../Shared_Config/config.h"
#include "../../Shared_Config/packet_protocol.h"

// =============================================================================
// STATE MACHINE — Estados de Voo da ESP1
// =============================================================================
typedef enum : uint8_t {
    STATE_STANDBY         = 0x01,  // Aguardando ARM + GPS fix
    STATE_ARMED_ON_PAD    = 0x02,  // Armado, aguardando ignição
    STATE_BURN            = 0x03,  // Motor ativo (aceleração positiva)
    STATE_COASTING        = 0x04,  // Fase balística (motor apagou)
    STATE_APOGEE_EJECTION = 0x05,  // Apogeu detectado — acionando servo
    STATE_DESCENT         = 0x06,  // Descida com paraquedas
    STATE_ABORT           = 0xFF,  // Aborto
} FlightState;

// Nomes para debug serial
const char* stateNames[] = {
    "UNKNOWN",
    "STANDBY",
    "ARMED_ON_PAD",
    "BURN",
    "COASTING",
    "APOGEE_EJECTION",
    "DESCENT",
    "ABORT"
};

// =============================================================================
// OBJETOS GLOBAIS
// =============================================================================

// LoRa
RH_RF95            rf95(PIN_LORA_CS, PIN_LORA_DIO0);
RHReliableDatagram loraManager(rf95, NODE_ESP1_ROCKET);

// Sensores
Adafruit_BMP280 bmp;
MPU6050         mpu;
TinyGPSPlus     gps;
Servo           paraquedas;

// =============================================================================
// VARIÁVEIS DE ESTADO
// =============================================================================
FlightState currentState = STATE_STANDBY;

// --- Dados dos Sensores ---
float altitude_m      = 0.0f;
float pressure_hpa    = 1013.25f;
float temperature_c   = 25.0f;
float accel_x         = 0.0f;
float accel_y         = 0.0f;
float accel_z         = -9.81f;
float gyro_x          = 0.0f;
float gyro_y          = 0.0f;
float gyro_z          = 0.0f;
float gps_lat         = 0.0f;
float gps_lon         = 0.0f;
float gps_alt_m       = 0.0f;
uint8_t gps_satellites = 0;
int8_t  lastRssi       = 0;

// --- Altitude de referência (calibrada no STANDBY) ---
float altitudeRef_m = 0.0f;
bool  altRefSet     = false;

// --- Detecção de Apogeu (Moving Average) ---
float  altBuffer[APOGEE_MA_WINDOW];
int    altIndex      = 0;
float  prevMA        = 0.0f;
int    descentCount  = 0;
bool   apogeeDetected = false;

// --- Servo ---
bool servoFired = false;
#define SERVO_CLOSED_DEG  0    // posição de repouso
#define SERVO_OPEN_DEG  180    // posição de disparo (perfura CO2)

// --- Timing ---
unsigned long lastTelemetryMs = 0;

// --- Threshold de ignição (accel_z > 0 = subindo com aceleração positiva) ---
#define ACCEL_IGNITION_THRESHOLD   5.0f   // m/s² acima de g (foguete acelerando)
#define ACCEL_BURNOUT_THRESHOLD   -5.0f   // m/s² (motor apagou, só gravidade)
#define ALTITUDE_LANDING_M        30.0f   // altitude (relativa) para declarar pouso

// =============================================================================
// PROTÓTIPOS
// =============================================================================
void initLoRa();
void initBMP280();
void initMPU6050();
void initGPS();
void initServo();
void readBMP280();
void readMPU6050();
void readGPS();
bool detectApogee(float newAlt);
void fireParachute();
void sendTelemetry();
void checkIncomingPackets();
void stateMachineLoop();
void calibrateAltitudeRef();
void debugPrintState();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println(F("=== ESP1 - Computador de Bordo CRT ==="));
    Serial.println(F("Inicializando subsistemas..."));

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    initBMP280();
    initMPU6050();
    initGPS();
    initServo();
    initLoRa();

    // Inicializar buffer de altitude com zero
    for (int i = 0; i < APOGEE_MA_WINDOW; i++) altBuffer[i] = 0.0f;

    Serial.println(F("ESP1 pronto. Estado: STANDBY"));
    Serial.println(F("Aguardando GPS fix e CMD_ARM..."));
}

// =============================================================================
// LOOP PRINCIPAL
// =============================================================================
void loop() {
    readBMP280();
    readMPU6050();
    readGPS();
    checkIncomingPackets();
    stateMachineLoop();

    // Enviar telemetria periodicamente
    if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        sendTelemetry();
        lastTelemetryMs = millis();
    }
}

// =============================================================================
// STATE MACHINE
// =============================================================================
void stateMachineLoop() {
    switch (currentState) {

        // ------------------------------------------------------------------ //
        case STATE_STANDBY:
            // Calibrar referência de altitude uma única vez com GPS fix
            if (!altRefSet && gps_satellites >= 4) {
                calibrateAltitudeRef();
            }
            // Transição para ARMED vem via CMD_ARM (tratado em checkIncomingPackets)
            break;

        // ------------------------------------------------------------------ //
        case STATE_ARMED_ON_PAD:
            // Aguarda CMD_IGNITION — transição em checkIncomingPackets
            // Monitorar pressão ambiente para referência
            if (!altRefSet) calibrateAltitudeRef();
            break;

        // ------------------------------------------------------------------ //
        case STATE_BURN:
            // Detectar fim da queima: aceleração cai abaixo do threshold
            // accel_z em m/s² (positivo = foguete subindo sob propulsão)
            if (accel_z < ACCEL_BURNOUT_THRESHOLD) {
                currentState = STATE_COASTING;
                Serial.println(F("[STATE] BURN → COASTING (motor apagou)"));
            }
            break;

        // ------------------------------------------------------------------ //
        case STATE_COASTING:
            // Detectar apogeu via Moving Average da altitude
            if (detectApogee(altitude_m)) {
                currentState = STATE_APOGEE_EJECTION;
                Serial.println(F("[STATE] COASTING → APOGEE_EJECTION"));
            }
            break;

        // ------------------------------------------------------------------ //
        case STATE_APOGEE_EJECTION:
            // Acionar paraquedas (apenas uma vez)
            if (!servoFired) {
                fireParachute();
            }
            // Após acionamento, passar para DESCENT
            if (servoFired) {
                currentState = STATE_DESCENT;
                Serial.println(F("[STATE] APOGEE_EJECTION → DESCENT"));
            }
            break;

        // ------------------------------------------------------------------ //
        case STATE_DESCENT:
            // Monitorar altitude para detectar pouso
            // altitude_m aqui é relativa à referência de lançamento
            if (altitude_m <= ALTITUDE_LANDING_M && altitude_m >= 0.0f) {
                Serial.println(F("[STATE] DESCENT → POUSO DETECTADO. Missão encerrada."));
                // Permanecer em DESCENT (estado final)
            }
            break;

        // ------------------------------------------------------------------ //
        case STATE_ABORT:
            // Estado terminal — aguarda reset físico da placa
            // Garantir que servo esteja em posição segura (não disparado em aborto pré-voo)
            // Nota: em aborto durante COASTING ou após, o paraquedas JÁ pode ter sido acionado.
            break;

        default:
            break;
    }
}

// =============================================================================
// DETECÇÃO DE APOGEU — Moving Average (apenas no estado COASTING)
// =============================================================================
bool detectApogee(float newAlt) {
    // Atualizar buffer circular
    altBuffer[altIndex] = newAlt;
    altIndex = (altIndex + 1) % APOGEE_MA_WINDOW;

    // Calcular média móvel
    float sum = 0.0f;
    for (int i = 0; i < APOGEE_MA_WINDOW; i++) sum += altBuffer[i];
    float currentMA = sum / (float)APOGEE_MA_WINDOW;

    // Detectar descida consistente
    if (currentMA < prevMA) {
        descentCount++;
    } else {
        descentCount = 0;  // Resetar se subir novamente (turbulência)
    }
    prevMA = currentMA;

    Serial.printf("[APOGEE] MA=%.2f  prev=%.2f  descentCount=%d\n",
                  currentMA, prevMA, descentCount);

    // Confirmar apogeu após N leituras consecutivas de descida
    if (descentCount >= APOGEE_CONFIRM_COUNT) {
        apogeeDetected = true;
        Serial.println(F("[APOGEE] APOGEU CONFIRMADO!"));
        return true;
    }
    return false;
}

// =============================================================================
// ACIONAMENTO DO PARAQUEDAS (Servo — perfuração do cilindro de CO2)
// =============================================================================
void fireParachute() {
    Serial.println(F("[SERVO] Acionando paraquedas — perfurando CO2!"));
    paraquedas.write(SERVO_OPEN_DEG);
    delay(500);  // Pulso de acionamento
    // Manter posição aberta (o CO2 já foi perfurado)
    servoFired = true;
    Serial.println(F("[SERVO] Paraquedas ACIONADO com sucesso."));
}

// =============================================================================
// ENVIO DE TELEMETRIA VIA LoRa → ESP3
// =============================================================================
void sendTelemetry() {
    RocketTelemetryPacket pkt;

    pkt.packet_type    = PKT_TELEMETRY_ROCKET;
    pkt.timestamp_ms   = millis();
    pkt.altitude_m     = altitude_m;
    pkt.pressure_hpa   = pressure_hpa;
    pkt.temperature_c  = temperature_c;
    pkt.accel_x        = accel_x;
    pkt.accel_y        = accel_y;
    pkt.accel_z        = accel_z;
    pkt.gyro_x         = gyro_x;
    pkt.gyro_y         = gyro_y;
    pkt.gyro_z         = gyro_z;
    pkt.gps_lat        = gps_lat;
    pkt.gps_lon        = gps_lon;
    pkt.gps_alt_m      = gps_alt_m;
    pkt.gps_satellites = gps_satellites;
    pkt.flight_state   = (uint8_t)currentState;
    pkt.rssi           = lastRssi;
    pkt.crc8           = calcCRC8((uint8_t*)&pkt, sizeof(pkt) - 1);

    bool ok = loraManager.sendtoWait(
        (uint8_t*)&pkt, sizeof(pkt),
        NODE_ESP3_CENTRAL
    );

    if (ok) {
        lastRssi = (int8_t)rf95.lastRssi();
        Serial.printf("[LORA] TX OK | Alt=%.1fm | AccZ=%.2f | State=%s | RSSI=%d\n",
                      altitude_m, accel_z, stateNames[currentState], lastRssi);
    } else {
        Serial.println(F("[LORA] TX FALHOU — sem ACK do ESP3"));
    }
}

// =============================================================================
// RECEPÇÃO DE PACOTES (comandos vindos do ESP3)
// =============================================================================
void checkIncomingPackets() {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;

    if (!loraManager.recvfromAckTimeout(buf, &len, 50, &from)) return;

    // Verificar CRC
    if (calcCRC8(buf, len - 1) != buf[len - 1]) {
        Serial.println(F("[LORA] RX — CRC inválido, pacote descartado"));
        return;
    }

    uint8_t pktType = buf[0];

    if (pktType == PKT_COMMAND && from == NODE_ESP3_CENTRAL) {
        CommandPacket* cmd = (CommandPacket*)buf;
        processCommand(cmd->command_id);
    } else if (pktType == PKT_PING) {
        // Heartbeat do ESP3 — responder automaticamente (o RadioHead já faz ACK)
        Serial.println(F("[LORA] RX — PING recebido do ESP3"));
    }
}

// =============================================================================
// PROCESSAMENTO DE COMANDOS RECEBIDOS
// =============================================================================
void processCommand(uint8_t cmdId) {
    Serial.printf("[CMD] Comando recebido: 0x%02X\n", cmdId);

    switch (cmdId) {

        case CMD_ARM:
            if (currentState == STATE_STANDBY) {
                if (gps_satellites >= 4) {
                    currentState = STATE_ARMED_ON_PAD;
                    Serial.println(F("[STATE] STANDBY → ARMED_ON_PAD"));
                } else {
                    Serial.println(F("[CMD] ARM rejeitado — GPS fix insuficiente!"));
                }
            } else {
                Serial.println(F("[CMD] ARM ignorado — estado inválido"));
            }
            break;

        case CMD_DISARM:
            if (currentState == STATE_ARMED_ON_PAD) {
                currentState = STATE_STANDBY;
                Serial.println(F("[STATE] ARMED_ON_PAD → STANDBY (DISARM)"));
            }
            break;

        case CMD_IGNITION:
            if (currentState == STATE_ARMED_ON_PAD) {
                currentState = STATE_BURN;
                Serial.println(F("[STATE] ARMED_ON_PAD → BURN (IGNIÇÃO!)"));
                // Resetar referência de apogeu para a altitude atual
                for (int i = 0; i < APOGEE_MA_WINDOW; i++) altBuffer[i] = altitude_m;
                prevMA       = altitude_m;
                descentCount = 0;
                apogeeDetected = false;
            } else {
                Serial.println(F("[CMD] IGNITION ignorado — não está ARMED"));
            }
            break;

        case CMD_ABORT:
            currentState = STATE_ABORT;
            Serial.println(F("[STATE] *** ABORTO RECEBIDO *** → STATE_ABORT"));
            break;

        default:
            // ESP1 não processa comandos de válvulas (são apenas para ESP2)
            Serial.printf("[CMD] Comando 0x%02X ignorado (não aplicável ao ESP1)\n", cmdId);
            break;
    }
}

// =============================================================================
// CALIBRAÇÃO DE ALTITUDE DE REFERÊNCIA
// =============================================================================
void calibrateAltitudeRef() {
    // Média de 20 leituras para minimizar ruído
    float sum = 0.0f;
    int   valid = 0;
    for (int i = 0; i < 20; i++) {
        float alt = bmp.readAltitude(1013.25f);
        if (!isnan(alt)) {
            sum += alt;
            valid++;
        }
        delay(50);
    }
    if (valid > 0) {
        altitudeRef_m = sum / (float)valid;
        altRefSet     = true;
        Serial.printf("[CALIB] Altitude de referência = %.2f m\n", altitudeRef_m);
    }
}

// =============================================================================
// LEITURA DO BMP280
// =============================================================================
void readBMP280() {
    float raw_alt = bmp.readAltitude(1013.25f);
    float raw_prs = bmp.readPressure() / 100.0f;  // Pa → hPa
    float raw_tmp = bmp.readTemperature();

    if (!isnan(raw_alt) && !isnan(raw_prs)) {
        pressure_hpa  = raw_prs;
        temperature_c = raw_tmp;
        // Altitude relativa ao ponto de lançamento
        altitude_m = altRefSet ? (raw_alt - altitudeRef_m) : raw_alt;
    }
}

// =============================================================================
// LEITURA DO MPU-6050
// =============================================================================
void readMPU6050() {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw,
                   &gx_raw, &gy_raw, &gz_raw);

    // Converter para m/s² (sensibilidade padrão ±2g = 16384 LSB/g)
    const float ACCEL_SCALE = 9.81f / 16384.0f;
    accel_x = (float)ax_raw * ACCEL_SCALE;
    accel_y = (float)ay_raw * ACCEL_SCALE;
    accel_z = (float)az_raw * ACCEL_SCALE;

    // Converter para °/s (sensibilidade padrão ±250°/s = 131 LSB/°s)
    const float GYRO_SCALE = 1.0f / 131.0f;
    gyro_x = (float)gx_raw * GYRO_SCALE;
    gyro_y = (float)gy_raw * GYRO_SCALE;
    gyro_z = (float)gz_raw * GYRO_SCALE;
}

// =============================================================================
// LEITURA DO GPS (GY-NEO6MV2) — Serial1 em GPIO25/26
// =============================================================================
void readGPS() {
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    if (gps.location.isUpdated()) {
        gps_lat = (float)gps.location.lat();
        gps_lon = (float)gps.location.lng();
    }
    if (gps.altitude.isUpdated()) {
        gps_alt_m = (float)gps.altitude.meters();
    }
    if (gps.satellites.isUpdated()) {
        gps_satellites = (uint8_t)gps.satellites.value();
    }
}

// =============================================================================
// INICIALIZAÇÃO DO LoRa (SX1278 / RA-02)
// =============================================================================
void initLoRa() {
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    pinMode(PIN_LORA_RST, OUTPUT);

    // Reset hardware do módulo
    digitalWrite(PIN_LORA_RST, LOW);
    delay(10);
    digitalWrite(PIN_LORA_RST, HIGH);
    delay(10);

    if (!loraManager.init()) {
        Serial.println(F("[ERRO] LoRa init FALHOU! Verifique cabeamento SPI."));
        while (true) delay(1000);
    }

    rf95.setFrequency(LORA_FREQUENCY / 1e6f);
    rf95.setSpreadingFactor(LORA_SPREADING_FACTOR);
    rf95.setSignalBandwidth(LORA_BANDWIDTH);
    rf95.setCodingRate4(LORA_CODING_RATE);
    rf95.setTxPower(LORA_TX_POWER, false);
    loraManager.setRetries(RH_MAX_RETRIES);
    loraManager.setTimeout(RH_TIMEOUT_MS);

    Serial.printf("[OK] LoRa iniciado — %.0f MHz, SF%d, %d dBm\n",
                  LORA_FREQUENCY / 1e6f, LORA_SPREADING_FACTOR, LORA_TX_POWER);
}

// =============================================================================
// INICIALIZAÇÃO DO BMP280
// =============================================================================
void initBMP280() {
    if (!bmp.begin(0x76)) {  // Endereço padrão BMP280; tente 0x77 se falhar
        Serial.println(F("[ERRO] BMP280 não encontrado! Verifique I2C."));
        while (true) delay(1000);
    }

    // Configuração para alta precisão em ambiente de voo
    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,    // temperatura
        Adafruit_BMP280::SAMPLING_X16,   // pressão
        Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_0_5
    );

    Serial.println(F("[OK] BMP280 iniciado"));
}

// =============================================================================
// INICIALIZAÇÃO DO MPU-6050
// =============================================================================
void initMPU6050() {
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println(F("[ERRO] MPU-6050 não encontrado! Verifique I2C."));
        while (true) delay(1000);
    }

    // Faixa de aceleração: ±16g para suportar aceleração do motor
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    // Faixa de giroscópio: ±500°/s
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

    Serial.println(F("[OK] MPU-6050 iniciado (±16g, ±500°/s)"));
}

// =============================================================================
// INICIALIZAÇÃO DO GPS (Serial1 remapeado para GPIO25/26)
// =============================================================================
void initGPS() {
    Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial.println(F("[OK] GPS iniciado em Serial1 (GPIO25=RX, GPIO26=TX)"));
    Serial.println(F("     Aguardando fix de satélites..."));
}

// =============================================================================
// INICIALIZAÇÃO DO SERVO
// =============================================================================
void initServo() {
    paraquedas.attach(PIN_SERVO);
    paraquedas.write(SERVO_CLOSED_DEG);  // Posição de repouso (fechado)
    delay(200);
    Serial.println(F("[OK] Servo iniciado — posição fechada (paraquedas recolhido)"));
}
