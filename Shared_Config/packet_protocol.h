// =============================================================================
// packet_protocol.h — Protocolo de Comunicação LoRa e Cabo Serial
// Capital Rocket Team — Software Trainee 2026
//
// Define a estrutura de todos os pacotes trocados entre os 3 ESPs.
// REGRA: Nunca criar estruturas de pacote fora deste arquivo.
//
// Estrutura de cada pacote:
//   [NODE_ID: 1 byte][PACKET_TYPE: 1 byte][PAYLOAD: N bytes][CRC8: 1 byte]
//
// O RadioHead (RHReliableDatagram) cuida do endereçamento de nó e ACK.
// Este arquivo define apenas o conteúdo do campo de dados (payload).
// =============================================================================

#pragma once
#include <stdint.h>

// =============================================================================
// TIPOS DE PACOTE
// =============================================================================
typedef enum : uint8_t {
  PKT_TELEMETRY_ROCKET  = 0x01,  // ESP1 → ESP3: dados de voo
  PKT_TELEMETRY_GS      = 0x02,  // ESP2 → ESP3: dados de abastecimento
  PKT_COMMAND           = 0x03,  // ESP3 → ESP2: comando de atuação
  PKT_ACK               = 0x04,  // Resposta genérica de confirmação
  PKT_PING              = 0x05,  // Heartbeat / verificação de link
  PKT_SYNC              = 0x06,  // Sincronização inicial no boot
} PacketType;

// =============================================================================
// COMANDOS (usados no payload de PKT_COMMAND)
// =============================================================================
typedef enum : uint8_t {
  CMD_OPEN_VENT         = 0x10,  // Abrir válvula Vent
  CMD_CLOSE_VENT        = 0x11,  // Fechar válvula Vent
  CMD_OPEN_PURGA        = 0x12,  // Abrir Purga do tanque (NF)
  CMD_CLOSE_PURGA       = 0x13,  // Fechar Purga do tanque (NF)
  CMD_OPEN_FILL         = 0x14,  // Abrir válvula de enchimento (iniciar abastecimento)
  CMD_CLOSE_FILL        = 0x15,  // Fechar válvula de enchimento
  CMD_OPEN_VENT_MANIFOLD = 0x16, // Abrir Vent do manifold
  CMD_CLOSE_VENT_MANIFOLD= 0x17, // Fechar Vent do manifold
  CMD_ARM               = 0x20,  // Armar sistema (habilita ignição)
  CMD_DISARM            = 0x21,  // Desarmar sistema
  CMD_IGNITION          = 0x30,  // Pulso de ignição (1 segundo no relé)
  CMD_ABORT             = 0xFF,  // Aborto geral — máxima prioridade
} CommandID;

// =============================================================================
// ESTADOS DE COMUNICAÇÃO (campo comm_state na telemetria)
// =============================================================================
typedef enum : uint8_t {
  COMM_RADIO            = 0x01,  // Comunicação normal via LoRa
  COMM_CABLE            = 0x02,  // Fallback: usando cabo Serial2
  COMM_ABORT            = 0x03,  // Ambos falharam — aborto automático
} CommState;

// =============================================================================
// PAYLOAD: Telemetria do ESP1 (Foguete) → ESP3
//
// O Integrante 1 deve preencher os campos que conseguir implementar.
// Campos marcados como [OBRIGATÓRIO] são essenciais para a saída CSV.
// Campos marcados como [OPCIONAL] enriquecem os dados pós-voo.
// =============================================================================
typedef struct __attribute__((packed)) {
  uint8_t  packet_type;     // Sempre PKT_TELEMETRY_ROCKET
  uint32_t timestamp_ms;    // [OBRIGATÓRIO] millis() no momento do envio

  // --- BMP280 (Barômetro) ---
  float    altitude_m;      // [OBRIGATÓRIO] Altitude calculada pelo barômetro (metros)
  float    pressure_hpa;    // [OBRIGATÓRIO] Pressão atmosférica (hPa)
  float    temperature_c;   // [OPCIONAL]   Temperatura (°C)

  // --- MPU-6050 (IMU) ---
  float    accel_x;         // [OPCIONAL] Aceleração X (m/s²)
  float    accel_y;         // [OPCIONAL] Aceleração Y (m/s²)
  float    accel_z;         // [OBRIGATÓRIO] Aceleração Z (m/s²) — eixo principal
  float    gyro_x;          // [OPCIONAL] Velocidade angular X (°/s)
  float    gyro_y;          // [OPCIONAL] Velocidade angular Y (°/s)
  float    gyro_z;          // [OPCIONAL] Velocidade angular Z (°/s)

  // --- GPS (GY-NEO6MV2) ---
  float    gps_lat;         // [OBRIGATÓRIO] Latitude
  float    gps_lon;         // [OBRIGATÓRIO] Longitude
  float    gps_alt_m;       // [OPCIONAL]   Altitude GPS (metros)
  uint8_t  gps_satellites;  // [OPCIONAL]   Número de satélites

  // --- Estado de Voo ---
  uint8_t  flight_state;    // [OBRIGATÓRIO] Estado atual da State Machine do ESP1
  int8_t   rssi;            // [OPCIONAL]   RSSI do sinal recebido (dBm)

  uint8_t  crc8;            // Checksum — sempre o último campo
} RocketTelemetryPacket;

// =============================================================================
// PAYLOAD: Telemetria do ESP2 (Ground Station) → ESP3
//
// O Integrante 2 deve preencher os campos que conseguir implementar.
// =============================================================================
typedef struct __attribute__((packed)) {
  uint8_t  packet_type;     // Sempre PKT_TELEMETRY_GS

  // --- HX711 (Célula de Carga) ---
  float    tank_mass_kg;    // [OBRIGATÓRIO] Massa atual do tanque (kg)
  float    oxidant_mass_kg; // [OBRIGATÓRIO] Massa de oxidante adicionado (kg) = tank_mass_kg - TANK_EMPTY_KG

  // --- Barômetro do Manifold ---
  float    manifold_bar;    // [OBRIGATÓRIO] Pressão do manifold (bar) — limite: 58 bar

  // --- Estado das Válvulas (bitmask) ---
  // bit 0: Purga NA (1=fechada/energizada, 0=aberta/natural)
  // bit 1: Vent (1=aberta, 0=fechada)
  // bit 2: Purga NF (1=aberta, 0=fechada)
  // bit 3: Válvula Fill (1=aberta, 0=fechada)
  // bit 4: Vent Manifold (1=aberta, 0=fechada)
  uint8_t  valve_state;     // [OBRIGATÓRIO] Bitmask do estado das válvulas

  // --- ACS712 (Sensor de Corrente) ---
  float    current_a;       // [OPCIONAL] Corrente nos atuadores (A)

  // --- Estado do Sistema ---
  uint8_t  gs_state;        // [OBRIGATÓRIO] Estado atual da State Machine do ESP2
  uint8_t  comm_state;      // [OBRIGATÓRIO] COMM_RADIO / COMM_CABLE / COMM_ABORT

  uint8_t  crc8;            // Checksum — sempre o último campo
} GSTelemetryPacket;

// =============================================================================
// PAYLOAD: Comando ESP3 → ESP2
// =============================================================================
typedef struct __attribute__((packed)) {
  uint8_t  packet_type;     // Sempre PKT_COMMAND
  uint8_t  command_id;      // Um dos valores de CommandID
  uint8_t  crc8;
} CommandPacket;

// =============================================================================
// PAYLOAD: Ping / Heartbeat (sem dados adicionais)
// =============================================================================
typedef struct __attribute__((packed)) {
  uint8_t  packet_type;     // PKT_PING ou PKT_SYNC
  uint32_t timestamp_ms;
  uint8_t  crc8;
} PingPacket;

// =============================================================================
// FUNÇÃO CRC8
// Algoritmo: CRC-8/MAXIM (polinômio 0x31)
// Usar para calcular e verificar o campo crc8 de qualquer pacote.
//
// Uso:
//   packet.crc8 = calcCRC8((uint8_t*)&packet, sizeof(packet) - 1);
//   if (calcCRC8(buf, len - 1) != buf[len - 1]) { // pacote corrompido }
// =============================================================================
inline uint8_t calcCRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// =============================================================================
// FORMATO CSV — Saída Serial do ESP3 (Central de Controle)
// Para uso direto no Serial Plotter da Arduino IDE
//
// Header (imprimir uma vez no setup):
//   Estado,Tempo,Altitude_m,AceleracaoZ,Pressao_hPa,MassaTanque_kg,Manifold_bar,COMM_STATE
//
// Exemplo de linha:
//   FILLING,12500,0.0,-9.81,1013.25,0.732,42.1,COMM_RADIO
// =============================================================================
#define CSV_HEADER "Estado,Tempo,Altitude_m,AceleracaoZ,Pressao_hPa,MassaTanque_kg,Manifold_bar,COMM_STATE"
