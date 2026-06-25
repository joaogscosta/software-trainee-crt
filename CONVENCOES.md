# Convenções de Código — CRT Software Trainee 2026

Este documento define as **regras obrigatórias** de código para todos os integrantes do subgrupo de Software. O não cumprimento dessas regras causará conflitos de merge e falhas de integração entre os subsistemas.

---

## 1. Arquitetura: State Machine Obrigatória

**É proibido usar `if/else` avulso no `loop()` para controlar o comportamento do sistema.**

Todo o fluxo de decisão deve ser controlado por uma máquina de estados. O padrão adotado é o `switch/case` com um `enum` de estados.

```cpp
// ✅ CORRETO
switch (currentState) {
  case STANDBY:
    handleStandby();
    break;
  case FILLING:
    handleFilling();
    break;
  // ...
}

// ❌ ERRADO — nunca faça isso no loop()
if (pressao > 58) {
  abort();
} else if (massaTanque >= 1.464) {
  // ...
}
```

---

## 2. Pinos: Sempre via `config.h`

**É proibido "chumar" (hardcode) qualquer número de pino diretamente no código.**

Todos os pinos estão definidos em `Shared_Config/config.h`. Copie o arquivo para sua pasta e use as macros.

```cpp
// ✅ CORRETO
#include "../Shared_Config/config.h"
digitalWrite(PIN_RELE_VENT, HIGH);

// ❌ ERRADO
digitalWrite(27, HIGH);
```

---

## 3. Temporização: Nunca Use `delay()`

**`delay()` trava o processador inteiro** e impede que a State Machine reaja a eventos (pressão acima do limite, perda de comunicação, etc.) durante a espera.

Use sempre `millis()` para temporização não-bloqueante:

```cpp
// ✅ CORRETO
unsigned long agora = millis();
if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
  ultimaLeitura = agora;
  fazerLeitura();
}

// ❌ ERRADO
fazerLeitura();
delay(500);
```

---

## 4. Nomenclatura

| Tipo | Convenção | Exemplo |
|---|---|---|
| Estados do enum | `UPPER_SNAKE_CASE` | `TANK_FULL`, `COMM_CABLE` |
| Pinos (em config.h) | `PIN_` + descrição | `PIN_RELE_VENT`, `PIN_SERVO` |
| Constantes | `UPPER_SNAKE_CASE` | `MASS_TARGET_KG`, `WATCHDOG_ESP2_MS` |
| Funções | `camelCase` | `readLoadCell()`, `sendTelemetry()` |
| Variáveis locais | `camelCase` | `pressaoAtual`, `massaTanque` |
| Variáveis globais | `g_` + `camelCase` | `g_currentState`, `g_lastPacketTime` |

---

## 5. Comentários

Comente **o porquê**, não o que o código faz (o código já diz o que faz).

```cpp
// ✅ CORRETO — explica a razão
// Aplica media movel para suavizar ruido do BMP280 antes de derivar
float altitudeFiltrada = movingAverage(altitudeBruta);

// ❌ RUIM — descreve o óbvio
// Calcula a media movel
float altitudeFiltrada = movingAverage(altitudeBruta);
```

---

## 6. Arquivo Compartilhado `config.h` — Regra de Ouro

- **Nunca editar sem comunicar a equipe** no grupo
- Toda mudança de pino ou constante crítica vai para o `config.h` da branch `dev/shared-config`
- Após aprovado e mergeado, cada integrante atualiza a cópia local

---

## 7. Pacotes LoRa — Usar Apenas as Structs de `packet_protocol.h`

Não criar estruturas de pacote próprias. Usar sempre as definidas em `Shared_Config/packet_protocol.h` para garantir compatibilidade entre os 3 ESPs.

---

## 8. Git

- **Nunca commitar diretamente na `main`**
- Mensagens de commit no formato: `tipo(escopo): descrição` — ver `GUIA_TECNICO.md`
- Não commitar arquivos `*.bin`, `*.elf` ou `build/` (cobertos pelo `.gitignore`)
