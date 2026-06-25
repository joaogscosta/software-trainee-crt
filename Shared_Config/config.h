// =============================================================================
// config.h — Mapeamento Central de Pinos e Constantes
// Capital Rocket Team — Software Trainee 2026
//
// REGRA: Nenhum pino ou constante crítica deve ser definido fora deste arquivo.
// REGRA: Qualquer alteração deve ser comunicada à equipe e feita via
//        branch dev/shared-config antes de ser mergeada.
// =============================================================================

#pragma once

// =============================================================================
// IDENTIFICADORES DOS NÓS (RadioHead)
// =============================================================================
#define NODE_ESP3_CENTRAL     0x01   // Central de Controle (Launch Station)
#define NODE_ESP2_GS          0x02   // Ground Station (Caixa na Rampa)
#define NODE_ESP1_ROCKET      0x03   // Foguete (Computador de Bordo)

// =============================================================================
// CONFIGURAÇÃO LORA (SX1278 RA-02)
// ATENÇÃO: Todos os 3 ESPs DEVEM usar exatamente os mesmos valores.
//          Qualquer diferença impede a comunicação.
// =============================================================================
#define LORA_FREQUENCY        433E6  // 433 MHz
#define LORA_SPREADING_FACTOR 7      // SF7 — maior velocidade, ~1km de alcance
#define LORA_BANDWIDTH        125E3  // 125 kHz (padrão)
#define LORA_CODING_RATE      5      // 4/5 (padrão)
#define LORA_TX_POWER         17     // dBm (máximo seguro para SX1278)

// =============================================================================
// PINOS LORA SPI — IGUAIS NOS 3 ESPs
// =============================================================================
#define PIN_LORA_SCK          18
#define PIN_LORA_MISO         19
#define PIN_LORA_MOSI         23
#define PIN_LORA_CS           5
#define PIN_LORA_RST          14
#define PIN_LORA_DIO0         2

// =============================================================================
// PINOS I2C — IGUAIS NOS 3 ESPs (MPU6050 + BMP280 compartilham o barramento)
// =============================================================================
#define PIN_I2C_SDA           21
#define PIN_I2C_SCL           22

// =============================================================================
// PINOS GPS UART — APENAS ESP1 (Foguete)
// Usa Serial1 remapeado para GPIO25/26 para evitar conflito com Serial2 (cabo)
// =============================================================================
#define PIN_GPS_RX            25     // GPS TX → ESP32 GPIO25
#define PIN_GPS_TX            26     // GPS RX ← ESP32 GPIO26
#define GPS_BAUD_RATE         9600

// =============================================================================
// PINOS CABO FÍSICO DE REDUNDÂNCIA UART — ESP2 e ESP3
// Serial2 padrão do ESP32: GPIO16 (RX) e GPIO17 (TX)
// Conexão cruzada: ESP3.GPIO17 (TX) → ESP2.GPIO16 (RX) e vice-versa
// =============================================================================
#define PIN_CABLE_RX          16     // Serial2 RX
#define PIN_CABLE_TX          17     // Serial2 TX
#define CABLE_BAUD_RATE       115200

// =============================================================================
// PINOS RELÉS — ESP2 (Ground Station)
// ATENÇÃO: A definir com o grupo de hardware. Valores abaixo são provisórios.
// Evitar pinos de strapping: 0, 2, 4, 5, 12, 15
// Evitar flash interno: 6, 7, 8, 9, 10, 11
// Evitar input-only: 34, 35, 36, 39
// =============================================================================
#define PIN_RELE_PURGA_NA     27     // Válvula Purga Tanque (NA - Normalmente Aberta)
#define PIN_RELE_VENT         32     // Válvula Vent (NF)
#define PIN_RELE_PURGA_NF     33     // Válvula Purga Tanque (NF)
#define PIN_RELE_VALVULA_FILL 25     // Válvula Principal de Enchimento (NF)
// TODO: Confirmar pinos com o grupo de hardware antes de soldar

// =============================================================================
// PINOS HX711 (Célula de Carga) — ESP2 (Ground Station)
// =============================================================================
#define PIN_HX711_DOUT        13
#define PIN_HX711_SCK         12

// =============================================================================
// PINOS SENSOR DE CORRENTE ACS712 — ESP2 (Ground Station)
// Leitura analógica
// =============================================================================
#define PIN_ACS712            34     // GPIO34 — input only, adequado para ADC

// =============================================================================
// PINOS SERVO MOTOR — ESP1 (Foguete)
// Responsável pela perfuração do cilindro de CO2 no apogeu
// =============================================================================
#define PIN_SERVO             13

// =============================================================================
// PINOS CONTROLES FÍSICOS — ESP3 (Central de Controle)
// Botões e chaves da maleta de lançamento
// ATENÇÃO: A definir com o grupo de hardware.
// =============================================================================
#define PIN_CHAVE_ARM         27     // Chave física de armar
#define PIN_BTN_IGNITION      32     // Botão de ignição
#define PIN_BTN_ABORT         33     // Botão de aborto manual
// TODO: Confirmar com hardware

// =============================================================================
// CONSTANTES CRÍTICAS DE MISSÃO
// Fonte: Documentos de propulsão e alimentação (Junho/2026)
// =============================================================================
#define MASS_TARGET_KG        1.464f // Massa alvo de oxidante N2O (kg)
#define PRESSURE_MAX_BAR      58.0f  // Pressão máxima do manifold (bar)
#define TANK_EMPTY_KG         18.3175f // Massa do tanque vazio (kg)
#define TANK_FULL_KG          20.0f  // Massa do tanque cheio (kg)

// =============================================================================
// CONSTANTES DE TEMPORIZAÇÃO
// =============================================================================
#define WATCHDOG_ESP1_MS      5000UL  // Timeout sem pacote do foguete → ROCKET_LOST
#define WATCHDOG_ESP2_MS      3000UL  // Timeout sem ACK do GS → tentar cabo
#define CABLE_TIMEOUT_MS      3000UL  // Timeout do cabo → COMM_ABORT
#define CABLE_RETRY_LORA_MS   30000UL // Intervalo para tentar reestabelecer LoRa via cabo
#define IGNITION_PULSE_MS     1000UL  // Duração do pulso de ignição (1 segundo)
#define TELEMETRY_INTERVAL_MS 300UL   // Intervalo mínimo entre envios de telemetria (ESP1)
#define GS_TELEMETRY_INTERVAL 500UL   // Intervalo mínimo entre envios de telemetria (ESP2)

// =============================================================================
// CONSTANTES DE COMUNICAÇÃO
// =============================================================================
#define SERIAL_BAUD_RATE      115200  // Baud rate USB/debug de todos os ESPs
#define RH_MAX_RETRIES        3       // Número máximo de retentativas RadioHead
#define RH_TIMEOUT_MS         200     // Timeout de ACK RadioHead (ms)

// =============================================================================
// CONSTANTES DE VOO (ESP1)
// =============================================================================
#define APOGEE_MA_WINDOW      10      // Tamanho da janela da Moving Average (leituras)
#define APOGEE_CONFIRM_COUNT  5       // Confirmações consecutivas de descida para apogeu
