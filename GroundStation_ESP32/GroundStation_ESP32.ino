#include <Arduino.h>
#include <HX711.h>
#include "../Shared/config.h" // Importa pinos e constantes globais (como PRESSAO_CRITICA)

HX711 escala;

// Definição da Máquina de Estados do Solo (conforme o documento técnico)
enum EstadoSolo { STANDBY, ABASTECIMENTO, LIMITE_ATINGIDO, ABORTO };
EstadoSolo estadoAtual = STANDBY;

// Protótipos das funções
float lerPressao();
float lerPeso();
void receberComandoLora();
void handleStandby();
void handleAbastecimento();
void handleLimite();
void handleAborto();

void setup() {
    Serial.begin(BAUD_RATE); // BAUD_RATE definido como 115200 no config.h
    
    // Configuração dos pinos de saída para os relés das válvulas
    pinMode(RELE_PURGA_PIN, OUTPUT);
    pinMode(RELE_VENT_PIN, OUTPUT);
    
    // Estado inicial seguro (Válvulas fechadas / Relés desligados)
    digitalWrite(RELE_PURGA_PIN, LOW);
    digitalWrite(RELE_VENT_PIN, LOW);

 // Inicializar a biblioteca do HX711 (Célula de carga) usando HX711_DOUT e HX711_SCK
    escala.begin(HX711_DOUT, HX711_SCK);

    escala.tare();

}

void loop() {
    // 1. SEGURANÇA AUTÔNOMA: Deve ser a PRIMEIRA verificação do loop, antes de tudo!
    if (lerPressao() > PRESSAO_CRITICA) { // PRESSAO_CRITICA é 58.0f bar
        estadoAtual = ABORTO;
        digitalWrite(RELE_PURGA_PIN, LOW);  // Corta o abastecimento/purga imediatamente
        digitalWrite(RELE_VENT_PIN, HIGH);  // Abre a válvula vent para aliviar a pressão do tanque
        return; // Interrompe o loop aqui. Nada mais importa neste ciclo!
    }
    
    // 2. COMUNICAÇÃO: Atualiza mensagens e comandos via rádio
    receberComandoLora();
    
    // 3. MÁQUINA DE ESTADOS: Executa o comportamento baseado no estado atual
    switch (estadoAtual) {
        case STANDBY:
            handleStandby();
            break;
        case ABASTECIMENTO:
            handleAbastecimento();
            break;
        case LIMITE_ATINGIDO:
            handleLimite();
            break;
        case ABORTO:
            handleAborto();
            break;
    }
}

// --- IMPLEMENTAÇÃO DAS FUNÇÕES COLOQUE SEU CÓDIGO AQUI ---

float lerPressao() {
    // Como o documento não especificou um pino para o barômetro no config.h,
    // assumimos um pino analógico livre (ex: GPIO 34) ou mude para o pino correto do circuito.
    int pinoBarometro = 34; 
    
    // Lê o valor bruto do conversor analógico-digital (0 a 4095 no ESP32)
    int leituraBruta = analogRead(pinoBarometro);
    
    // Converte a leitura de tensão (0 a 3.3V) para a escala de pressão (0 a 100 bar) do manifold
    // Nota: Essa equação matemática exata depende de como o circuito elétrico foi montado!
    float tensao = (leituraBruta / 4095.0f) * 3.3f;
    float pressao_bar = (tensao / 3.3f) * 100.0f; // Ajuste o cálculo conforme calibração da equipe
    
    return pressao_bar; 
}

float lerPeso() {
    // Verifica se o módulo HX711 está respondendo antes de ler
    if (escala.is_ready()) {
        // Retorna a leitura calibrada diretamente em kg
        return escala.get_units(5); // Faz a média de 5 leituras para ficar estável
    } else {
        Serial.println("Erro: HX711 não detectado!");
        return 0.0f; // Retorna 0 se houver falha de conexão física
    }
}

void receberComandoLora() {
    // TODO: O Integrante 3 vai configurar a recepção de pacotes da ESP 3 aqui.
    // Quando a ESP 3 mandar o comando de Abastecer, essa função mudará o estadoAtual para ABASTECIMENTO.
}

void handleStandby() {
    // Sistema inicializado. Válvulas fechadas e relés desligados.
    digitalWrite(RELE_PURGA_PIN, LOW);
    digitalWrite(RELE_VENT_PIN, LOW);
}


void handleAbastecimento() {
    // 1. ATUAÇÃO: Abre a válvula de purga/abastecimento e garante que o respiro (vent) está fechado
    digitalWrite(RELE_PURGA_PIN, HIGH); 
    digitalWrite(RELE_VENT_PIN, LOW);
    
    // 2. MONITORAMENTO E TRANSIÇÃO AUTÔNOMA:
    // Pega o peso atual lido pela célula de carga (HX711) que configuramos antes
    float pesoAtual = lerPeso(); 
    
    // Se o peso do combustível atingir ou passar do limite definido pela equipe...
    if (pesoAtual >= PESO_ALVO) { 
        // ... a máquina muda automaticamente para o estado LIMITE_ATINGIDO
        estadoAtual = LIMITE_ATINGIDO; 
        
        // Imprime um aviso no monitor para os operadores na maleta
        Serial.println("Aviso: Peso alvo atingido! Encerrando abastecimento."); 
    }
}

void handleLimite() {
    // Peso alvo atingido. Fecha a válvula de purga automaticamente e aguarda confirmação humana.
    digitalWrite(RELE_PURGA_PIN, LOW);
}

void handleAborto() {
    // Estado de emergência. Garante o corte total da purga e a abertura total do vent.
    digitalWrite(RELE_PURGA_PIN, LOW);
    digitalWrite(RELE_VENT_PIN, HIGH);
}