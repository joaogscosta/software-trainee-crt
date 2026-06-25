#pragma once

// Configurações Globais exigidas no Alinhamento Técnico
#define BAUD_RATE 115200

// Pinos da ESP 2 (Ground Station)
#define HX711_DOUT 32
#define HX711_SCK 33
#define RELE_PURGA_PIN 25
#define RELE_VENT_PIN 26

// Limites de Segurança
#define PRESSAO_CRITICA 58.0f // bar - corte autônomo

// Define o peso em KG (Usamos 5.0kg)
const float PESO_ALVO = 5.0f;

// ==========================================
// PINOS DA ESP 3 (Maleta de Controle)
// ==========================================
// Escolhemos pinos livres e seguros comuns no ESP32 para os botões
#define BTN_ARM_PIN   14
#define BTN_PURGA_PIN 27
#define BTN_VENT_PIN  12
#define BTN_ABORT_PIN 13