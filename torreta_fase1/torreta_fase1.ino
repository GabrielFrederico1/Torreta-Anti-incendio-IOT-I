/*
  ============================================================
  TORRETA DE COMBATE A INCÊNDIO - FASE 1
  ============================================================
  Escopo desta fase (propositalmente simplificado):
    - Servo PAN (horizontal) ativo
    - Servo TILT (vertical)  -> IGNORADO nesta fase
    - Display OLED           -> IGNORADO nesta fase
    - Conexão Wi-Fi          -> IGNORADO nesta fase

  Fluxo:
    1. Servo pan parte de 90°
    2. Varre até 0° (esquerda)
    3. Varre até 180° (direita)
       -> durante toda a varredura, guarda o ângulo onde o
          sinal analógico do KY-026 foi o MENOR (= chama mais
          forte/mais próxima, pois o KY-026 reduz a leitura
          analógica quanto maior a intensidade de chama)
    4. Retorna o servo ao ângulo salvo (ponto do incêndio)
    5. Mede distância com o HC-SR04
    6. Verifica nível de água pela chave bóia
    7. Se houver água -> aciona o relé/bomba por um tempo
    8. Desliga a bomba e retorna o servo a 90° (neutro)

  Mensagens de status são enviadas ao Monitor Serial em cada
  etapa.

  IMPORTANTE (hardware):
    - ECHO do HC-SR04 precisa de divisor resistivo ou
      conversor de nível lógico antes do GPIO do ESP32
      (TRIG pode ir direto, sem conversão).
    - Confirme se o KY-026 é alimentado em 3,3V ou se a saída
      analógica também precisa de adequação de nível.
    - Ajuste o sentido lógico da chave bóia (CHEIO_NIVEL_LOGICO)
      conforme o modelo físico do sensor (NA ou NF).
  ============================================================
*/

#include <ESP32Servo.h>

// ---------------------- PINAGEM ----------------------------
// Evitados os pinos sensíveis de boot (0, 2, 12, 15) e os
// ADC2 (conflitam com Wi-Fi quando ele for ligado em fases futuras).

const int PIN_SERVO_PAN   = 18;   // Servo horizontal (PWM)
const int PIN_KY026_AO    = 34;   // Saída analógica do KY-026 (ADC1, input-only)
const int PIN_TRIG        = 5;    // TRIG do HC-SR04 (saída do ESP32, 3,3V ok)
const int PIN_ECHO        = 19;   // ECHO do HC-SR04 (precisa de divisor/level shifter!)
const int PIN_RELE        = 23;   // IN do módulo relé (aciona a bomba)
const int PIN_BOIA        = 27;   // Chave bóia (nível de água)

// ---------------------- PARÂMETROS --------------------------

const int ANGULO_NEUTRO   = 90;
const int ANGULO_MIN      = 0;
const int ANGULO_MAX      = 180;
const int PASSO_VARREDURA = 2;     // graus por passo da varredura
const int DELAY_PASSO_MS  = 40;    // tempo de assentamento do servo por passo

const int LEITURAS_POR_PASSO = 5;  // amostras de média por ângulo (reduz ruído)

const unsigned long TEMPO_BOMBA_MS = 4000; // tempo de acionamento da bomba

// Ajuste conforme a lógica física da bóia:
// Se a bóia fecha o contato com GND quando HÁ água -> LOW = água presente
const int NIVEL_AGUA_PRESENTE = LOW;

Servo servoPan;

int anguloAlvo = ANGULO_NEUTRO;
int menorLeituraKY = 4095; // valor inicial alto (ADC de 12 bits no ESP32: 0-4095)

// =============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== TORRETA - FASE 1 (sem tilt, sem OLED, sem Wi-Fi) ===");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_RELE, OUTPUT);
  pinMode(PIN_BOIA, INPUT_PULLUP);
  pinMode(PIN_KY026_AO, INPUT);

  digitalWrite(PIN_RELE, LOW); // bomba desligada por segurança
  digitalWrite(PIN_TRIG, LOW);

  servoPan.setPeriodHertz(50);
  servoPan.attach(PIN_SERVO_PAN, 500, 2400);
  servoPan.write(ANGULO_NEUTRO);
  Serial.println("[INIT] Servo pan posicionado em 90° (neutro).");
  delay(500);
}

// =============================================================
void loop() {
  executarCicloDeBusca();

  Serial.println("[FIM DE CICLO] Aguardando 5s antes do proximo ciclo...");
  Serial.println("---------------------------------------------------");
  delay(5000);
}

// =============================================================
// Executa um ciclo completo: varredura -> mira -> distancia ->
// agua -> disparo -> retorno ao neutro.
// =============================================================
void executarCicloDeBusca() {

  menorLeituraKY = 4095;
  anguloAlvo = ANGULO_NEUTRO;

  Serial.println("[ETAPA 1] Iniciando varredura horizontal...");

  // ---- Varre de 90 -> 0 (esquerda) ----
  Serial.println("[VARREDURA] Indo para a esquerda (90 -> 0)...");
  for (int ang = ANGULO_NEUTRO; ang >= ANGULO_MIN; ang -= PASSO_VARREDURA) {
    avaliarAngulo(ang);
  }

  // ---- Varre de 0 -> 180 (direita), cobrindo todo o range ----
  Serial.println("[VARREDURA] Indo para a direita (0 -> 180)...");
  for (int ang = ANGULO_MIN; ang <= ANGULO_MAX; ang += PASSO_VARREDURA) {
    avaliarAngulo(ang);
  }

  Serial.print("[ETAPA 1] Varredura concluida. Menor leitura KY-026 = ");
  Serial.print(menorLeituraKY);
  Serial.print(" no angulo ");
  Serial.print(anguloAlvo);
  Serial.println(" graus.");

  // ---- Retorna ao ponto do incêndio ----
  Serial.print("[ETAPA 2] Retornando ao ponto do incendio (");
  Serial.print(anguloAlvo);
  Serial.println(" graus)...");
  moverServoSuave(anguloAlvo);
  delay(300);

  // ---- Mede distância ----
  Serial.println("[ETAPA 3] Medindo distancia com HC-SR04...");
  float distanciaCm = medirDistanciaCm();
  if (distanciaCm < 0) {
    Serial.println("[ETAPA 3] Falha na leitura do HC-SR04 (sem eco / fora de alcance).");
  } else {
    Serial.print("[ETAPA 3] Distancia ao alvo: ");
    Serial.print(distanciaCm);
    Serial.println(" cm.");
  }

  // ---- Verifica nivel de agua ----
  Serial.println("[ETAPA 4] Verificando nivel de agua na boia...");
  bool temAgua = (digitalRead(PIN_BOIA) == NIVEL_AGUA_PRESENTE);
  if (temAgua) {
    Serial.println("[ETAPA 4] Agua disponivel no reservatorio.");
  } else {
    Serial.println("[ETAPA 4] Reservatorio SEM agua suficiente.");
  }

  // ---- Decisao de disparo ----
  if (temAgua) {
    Serial.println("[ETAPA 5] Condicoes atendidas. Acionando bomba...");
    acionarBomba(TEMPO_BOMBA_MS);
  } else {
    Serial.println("[ETAPA 5] Disparo CANCELADO: nivel de agua insuficiente.");
  }

  // ---- Retorna ao neutro ----
  Serial.println("[ETAPA 6] Retornando servo ao angulo neutro (90 graus)...");
  moverServoSuave(ANGULO_NEUTRO);
  Serial.println("[ETAPA 6] Servo em posicao neutra.");
}

// =============================================================
// Move o servo um passo, le o KY-026 (com media de amostras) e
// atualiza o angulo de menor leitura (= chama mais forte).
// =============================================================
void avaliarAngulo(int angulo) {
  servoPan.write(angulo);
  delay(DELAY_PASSO_MS);

  long soma = 0;
  for (int i = 0; i < LEITURAS_POR_PASSO; i++) {
    soma += analogRead(PIN_KY026_AO);
    delay(2);
  }
  int leituraMedia = soma / LEITURAS_POR_PASSO;

  if (leituraMedia < menorLeituraKY) {
    menorLeituraKY = leituraMedia;
    anguloAlvo = angulo;
    Serial.print("  [VARREDURA] Novo pico de chama detectado -> angulo ");
    Serial.print(angulo);
    Serial.print(", leitura KY-026 = ");
    Serial.println(leituraMedia);
  }
}

// =============================================================
// Move o servo de forma suave (passo a passo) ate o angulo alvo,
// evitando movimentos bruscos que possam gerar picos de corrente.
// =============================================================
void moverServoSuave(int anguloDestino) {
  int anguloAtual = servoPan.read();
  int passo = (anguloDestino >= anguloAtual) ? PASSO_VARREDURA : -PASSO_VARREDURA;

  for (int ang = anguloAtual; (passo > 0) ? (ang <= anguloDestino) : (ang >= anguloDestino); ang += passo) {
    servoPan.write(ang);
    delay(DELAY_PASSO_MS);
  }
  servoPan.write(anguloDestino);
}

// =============================================================
// Mede a distancia em cm usando o HC-SR04.
// Retorna -1 em caso de timeout (sem leitura de eco).
// =============================================================
float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duracao = pulseIn(PIN_ECHO, HIGH, 30000UL); // timeout 30ms (~5m)

  if (duracao == 0) {
    return -1.0;
  }

  // Velocidade do som ~ 0,0343 cm/us, dividido por 2 (ida e volta)
  float distancia = (duracao * 0.0343) / 2.0;
  return distancia;
}

// =============================================================
// Liga o rele/bomba pelo tempo especificado e depois desliga.
// =============================================================
void acionarBomba(unsigned long tempoMs) {
  digitalWrite(PIN_RELE, HIGH);
  Serial.println("[BOMBA] Bomba LIGADA.");
  delay(tempoMs);
  digitalWrite(PIN_RELE, LOW);
  Serial.println("[BOMBA] Bomba DESLIGADA.");
}
