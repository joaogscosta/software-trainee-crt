#include <Arduino.h>
#include "../Shared/config.h" // Os pinos dos botões vêm daqui (ex: BTN_ARM_PIN)

// Protótipos das funções
bool lerBotaoComDebounce(int pino, bool &ultimoEstado, unsigned long &ultimoTempo);
void enviarComandoLora(String comando);

// Tempo de espera para eliminar o ruído elétrico dos botões (em milissegundos)
const unsigned long tempoDebounce = 50; 

// Variáveis de controle de tempo e estado para o Debounce de cada botão
bool ultimoEstadoArm = HIGH;
unsigned long ultimoTempoArm = 0;

bool ultimoEstadoPurga = HIGH;
unsigned long ultimoTempoPurga = 0;

bool ultimoEstadoVent = HIGH;
unsigned long ultimoTempoVent = 0;

bool ultimoEstadoAbort = HIGH;
unsigned long ultimoTempoAbort = 0;

void setup() {
    Serial.begin(BAUD_RATE); // 115200 baud
    
    // Configura os pinos dos botões com Pull-Up interno
    // (O circuito lê HIGH solto e LOW quando você pressiona o botão)
    pinMode(BTN_ARM_PIN, INPUT_PULLUP);   
    pinMode(BTN_PURGA_PIN, INPUT_PULLUP); 
    pinMode(BTN_VENT_PIN, INPUT_PULLUP);  
    pinMode(BTN_ABORT_PIN, INPUT_PULLUP); 
    
    // // TODO: O Integrante 3 vai injetar a inicialização do LoRa.begin() aqui
}

void loop() {
    // 1. LEITURA DOS BOTÕES COM DEBOUNCE EFETIVO
    
    // Botão de Aborto (Prioridade Máxima da Maleta)
    if (lerBotaoComDebounce(BTN_ABORT_PIN, ultimoEstadoAbort, ultimoTempoAbort)) {
        Serial.println("Comando acionado: ABORTO!"); 
        enviarComandoLora("ABORT");
    }
    
    // Chave de Armamento
    if (lerBotaoComDebounce(BTN_ARM_PIN, ultimoEstadoArm, ultimoTempoArm)) {
        Serial.println("Comando acionado: ARMAMENTO!");
        enviarComandoLora("ARM");
    }
    
    // Botão da Válvula de Purga (Abastecimento)
    if (lerBotaoComDebounce(BTN_PURGA_PIN, ultimoEstadoPurga, ultimoTempoPurga)) {
        Serial.println("Comando acionado: ALTERAR PURGA!");
        enviarComandoLora("TOGGLE_PURGA");
    }
    
    // Botão da Válvula Vent (Alívio)
    if (lerBotaoComDebounce(BTN_VENT_PIN, ultimoEstadoVent, ultimoTempoVent)) {
        Serial.println("Comando acionado: ALTERAR VENT!");
        enviarComandoLora("TOGGLE_VENT");
    }
    
    // // TODO: O Integrante 3 vai ler os dados LoRa recebidos da ESP2 e ESP1 
    // // e printar em formato CSV aqui para o Serial Plotter rodar no PC.
}

// Função de Debounce por Software (Evita leituras falsas por ruído mecânico)
bool lerBotaoComDebounce(int pino, bool &ultimoEstado, unsigned long &ultimoTempo) {
    bool leituraAtual = digitalRead(pino);
    bool pressionado = false;
    
    if (leituraAtual != ultimoEstado) {
        ultimoTempo = millis(); // Reseta o cronômetro se o pino oscilou
    }
    
    if ((millis() - ultimoTempo) > tempoDebounce) {
        // Se o sinal estabilizou em LOW (pressionado) vindo de um HIGH (solto)
        if (leituraAtual == LOW && ultimoEstado == HIGH) {
            pressionado = true; 
        }
        ultimoEstado = leituraAtual;
    }
    
    return pressionado;
}

void enviarComandoLora(String comando) {
    // TODO: O Integrante 3 vai implementar o envio do pacote de rádio real aqui
}