# 🚀 CRT Software — Trainee 2026

Sistema embarcado de telemetria e controle do foguete híbrido da **Capital Rocket Team**, desenvolvido no processo trainee 2026.

**Apogeu alvo:** 1 km | **Propelente:** Parafina + N₂O | **Lançamento previsto:** 05/11/2026, Iacanga-SP

---

## Visão Geral do Sistema

O ecossistema eletrônico é composto por **3 ESPs32-WROOM** comunicando-se via **LoRa SX1278 @ 433 MHz**, com redundância física por cabo Serial2 entre ESP2 e ESP3.

```
  [ESP1 - Foguete]                          [ESP3 - Central de Controle]
  Sensores de voo    ────────────────────▶  Exibe telemetria CSV
  BMP280, MPU6050         LoRa (RF)          Lê botões físicos
  GPS, Servo         ◀────────────────────  (sem comandos ao foguete)
                              │
                         LoRa (RF)
                       + cabo Serial2
                              │
                    [ESP2 - Ground Station]
                      Célula de carga
                      Barômetro manifold
                      Relés das válvulas
```

### Fluxo de Comunicação
| Direção | Canal | Descrição |
|---|---|---|
| ESP1 → ESP3 | LoRa | Telemetria de voo (altitude, IMU, GPS) |
| ESP2 → ESP3 | LoRa / cabo | Telemetria de abastecimento (massa, pressão) |
| ESP3 → ESP2 | LoRa / cabo | Comandos (válvulas, ARM, ignição, ABORT) |

---

## Estrutura do Repositório

```
software-trainee-crt/
├── README.md                  ← Este arquivo
├── GUIA_TECNICO.md            ← Leia antes de começar a codar!
├── CONVENCOES.md              ← Regras obrigatórias de código
├── .gitignore
│
├── Shared_Config/
│   ├── config.h               ← TODOS os pinos e constantes
│   └── packet_protocol.h      ← Definição dos pacotes LoRa/cabo
│
├── Foguete_ESP1/
│   └── Foguete_ESP1/
│       └── Foguete_ESP1.ino   ← Integrante 1
│
├── GroundStation_ESP2/
│   └── GroundStation_ESP2/
│       └── GroundStation_ESP2.ino  ← Integrante 2
│
└── CentralControle_ESP3/
    └── CentralControle_ESP3/
        └── CentralControle_ESP3.ino  ← Integrante 3
```

---

## Quick Start

### 1. Clone o repositório
```bash
git clone https://github.com/joaogscosta/software-trainee-crt.git
cd software-trainee-crt
```

### 2. Configure o ambiente
**Leia o [GUIA_TECNICO.md](GUIA_TECNICO.md)** — ele contém o passo a passo completo para configurar a Arduino IDE ou o VSCode+PlatformIO, instalar as bibliotecas e entender o protocolo de comunicação.

### 3. Abra seu subsistema
Na Arduino IDE, use *File → Open* e navegue até a pasta do seu subsistema (ex: `Foguete_ESP1/Foguete_ESP1/Foguete_ESP1.ino`).

### 4. Copie os arquivos compartilhados
Copie `Shared_Config/config.h` e `Shared_Config/packet_protocol.h` para dentro da pasta do seu `.ino`.

---

## Bibliotecas Necessárias

| Biblioteca | Instalar como |
|---|---|
| RadioHead | `RadioHead` by Mike McCauley |
| GPS | `TinyGPSPlus` by Mikal Hart |
| Barômetro | `Adafruit BMP280 Library` + `Adafruit Unified Sensor` |
| IMU | `MPU6050` by ElectronicCats |

---

## Responsabilidades

| Integrante | Subsistema | Arquivo principal |
|---|---|---|
| Integrante 1 | Foguete (ESP1) | `Foguete_ESP1.ino` |
| Integrante 2 | Ground Station (ESP2) | `GroundStation_ESP2.ino` |
| Integrante 3 | Central de Controle (ESP3) + Comunicação | `CentralControle_ESP3.ino` |

---

## Links Rápidos

- 📖 [Guia Técnico Completo](GUIA_TECNICO.md)
- 📋 [Convenções de Código](CONVENCOES.md)
- 🔧 [Mapeamento de Pinos](Shared_Config/config.h)
- 📡 [Protocolo de Comunicação](Shared_Config/packet_protocol.h)
