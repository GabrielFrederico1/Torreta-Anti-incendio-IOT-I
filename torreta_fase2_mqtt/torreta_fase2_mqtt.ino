/*
  ============================================================
  TORRETA DE COMBATE A INCÊNDIO - FASE 1 + MQTT
  ============================================================
  Escopo desta fase:
    - Servo PAN (horizontal) ativo
    - Servo TILT (vertical)  -> IGNORADO nesta fase
    - Display OLED           -> IGNORADO nesta fase
    - Conexão Wi-Fi + MQTT   -> ATIVO (telemetria + comandos remotos)

  Fluxo do ciclo (igual à fase 1, agora com publicação MQTT):
    1. Servo pan parte de 90°
    2. Varre até 0° (esquerda)
    3. Varre até 180° (direita)
       -> guarda o ângulo onde o sinal analógico do KY-026 foi
          o MENOR (= chama mais forte/mais próxima)
    4. Retorna o servo ao ângulo salvo (ponto do incêndio)
    5. Mede distância com o HC-SR04
    6. Verifica nível de água pela chave bóia
    7. Se houver água -> aciona o relé/bomba por um tempo
    8. Desliga a bomba e retorna o servo a 90° (neutro)

  MQTT:
    - Publica telemetria de cada etapa em tópicos próprios.
    - Aceita comandos remotos no tópico de comando:
        "INICIAR"     -> força o início de um novo ciclo agora
        "BOMBA_ON"    -> liga a bomba manualmente (override)
        "BOMBA_OFF"   -> desliga a bomba manualmente
        "PARAR"       -> cancela o ciclo de busca em andamento
                         (a varredura é interrompida no próximo passo)

  IMPORTANTE (hardware - mesmo da fase 1):
    - ECHO do HC-SR04 precisa de divisor resistivo ou
      conversor de nível lógico antes do GPIO do ESP32
      (TRIG pode ir direto, sem conversão).
    - Confirme se o KY-026 é alimentado em 3,3V ou se a saída
      analógica também precisa de adequação de nível.
    - Ajuste o sentido lógico da chave bóia (NIVEL_AGUA_PRESENTE)
      conforme o modelo físico do sensor (NA ou NF).

  IMPORTANTE (rede/MQTT):
    - broker.emqx.io é um broker PÚBLICO de testes: qualquer
      pessoa pode publicar/assinar nos mesmos tópicos. Troque
      o prefixo dos tópicos (PREFIXO_TOPICO) para algo único
      do seu grupo, e troque também o ID_MQTT (client id),
      pois IDs duplicados no mesmo broker derrubam a conexão
      um do outro.
    - Em produção, prefira um broker próprio/autenticado
      (usuário+senha ou TLS), já que o broker.emqx.io não deve
      ser usado para controlar um atuador (bomba d'água) de
      verdade fora de um ambiente de testes.
  ============================================================
*/

#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---------------------- PINAGEM ----------------------------
// Evitados os pinos sensíveis de boot (0, 2, 12, 15) e os
// ADC2 (conflitam com Wi-Fi).

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

const unsigned long TEMPO_BOMBA_MS = 4000; // tempo de acionamento da bomba (modo automático)

// Se a bóia fecha o contato com GND quando HÁ água -> LOW = água presente
const int NIVEL_AGUA_PRESENTE = LOW;

const unsigned long INTERVALO_ENTRE_CICLOS_MS = 5000;

// ---------------------- WI-FI --------------------------------

const char* WIFI_SSID     = "NOME_DA_SUA_REDE";
const char* WIFI_PASSWORD = "SENHA_DA_SUA_REDE";

// ---------------------- MQTT -----------------------------------

const char* BROKER_MQTT = "broker.emqx.io";
const int   BROKER_PORT = 1883;

// TROQUE este prefixo por algo único do seu grupo/projeto
const char* PREFIXO_TOPICO = "puc_sg_torreta_incendio_grupoX";

// ID precisa ser único no broker (evite colisão com outros dispositivos)
const char* ID_MQTT = "puc_sg_torreta_incendio_grupoX_esp32";

String topicoPubStatus;
String topicoPubDeteccao;
String topicoPubDistancia;
String topicoPubAgua;
String topicoPubBomba;
String topicoSubComando;

WiFiClient    espClient;
PubSubClient  MQTT(espClient);

// ---------------------- ESTADO GLOBAL ---------------------------

Servo servoPan;

int anguloAlvo = ANGULO_NEUTRO;
int menorLeituraKY = 4095; // valor inicial alto (ADC de 12 bits no ESP32: 0-4095)

volatile bool comandoIniciarCiclo = false; // INICIAR via MQTT
volatile bool comandoPararCiclo   = false; // PARAR via MQTT
volatile bool bombaManualOn       = false; // BOMBA_ON via MQTT
volatile bool bombaManualOff      = false; // BOMBA_OFF via MQTT

unsigned long ultimoCicloMs = 0;

// ---------------------- PROTOTYPES -------------------------------

void initWiFi();
void reconnectWiFi();
void initMQTT();
void reconnectMQTT();
void verificaConexoesWifiEMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void montarTopicos();
void publicarStatus(const String &topico, const String &mensagem);

void executarCicloDeBusca();
void avaliarAngulo(int angulo);
void moverServoSuave(int anguloDestino);
float medirDistanciaCm();
void acionarBomba(unsigned long tempoMs);

// =============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== TORRETA - FASE 1 + MQTT ===");

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

  montarTopicos();
  initWiFi();
  initMQTT();
}

// =============================================================
void loop() {
  verificaConexoesWifiEMqtt();
  MQTT.loop();

  executarCicloDeBusca();

  Serial.println("[FIM DE CICLO] Aguardando antes do proximo ciclo...");
  Serial.println("---------------------------------------------------");

  // Aguarda entre ciclos sem travar o MQTT, e permite que um
  // comando "INICIAR" recebido durante a espera antecipe o ciclo.
  unsigned long inicioEspera = millis();
  while (millis() - inicioEspera < INTERVALO_ENTRE_CICLOS_MS) {
    verificaConexoesWifiEMqtt();
    MQTT.loop();
    if (comandoIniciarCiclo) {
      comandoIniciarCiclo = false;
      break;
    }
    delay(50);
  }
}

// =============================================================
// Monta os nomes completos dos tópicos a partir do prefixo.
// =============================================================
void montarTopicos() {
  topicoPubStatus    = String(PREFIXO_TOPICO) + "/status";
  topicoPubDeteccao  = String(PREFIXO_TOPICO) + "/deteccao";
  topicoPubDistancia = String(PREFIXO_TOPICO) + "/distancia";
  topicoPubAgua      = String(PREFIXO_TOPICO) + "/agua";
  topicoPubBomba     = String(PREFIXO_TOPICO) + "/bomba";
  topicoSubComando   = String(PREFIXO_TOPICO) + "/comando";
}

// =============================================================
// Wi-Fi
// =============================================================
void initWiFi() {
  Serial.println("------ Conexao WI-FI ------");
  Serial.print("Conectando-se na rede: ");
  Serial.println(WIFI_SSID);
  reconnectWiFi();
}

void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("[WIFI] Conectado!");
  Serial.print("[WIFI] IP: ");
  Serial.println(WiFi.localIP());
}

// =============================================================
// MQTT
// =============================================================
void initMQTT() {
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.setCallback(mqttCallback);
}

void reconnectMQTT() {
  while (!MQTT.connected()) {
    Serial.print("[MQTT] Tentando conectar ao broker: ");
    Serial.println(BROKER_MQTT);

    if (MQTT.connect(ID_MQTT)) {
      Serial.println("[MQTT] Conectado com sucesso!");
      MQTT.subscribe(topicoSubComando.c_str());
      Serial.print("[MQTT] Inscrito em: ");
      Serial.println(topicoSubComando);
    } else {
      Serial.print("[MQTT] Falha, rc=");
      Serial.print(MQTT.state());
      Serial.println(" tentando novamente em 2s...");
      delay(2000);
    }
  }
}

void verificaConexoesWifiEMqtt() {
  reconnectWiFi();
  if (!MQTT.connected()) {
    reconnectMQTT();
  }
}

// =============================================================
// Callback MQTT - trata comandos recebidos no topico de comando
// =============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("[MQTT] Mensagem recebida [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  if (String(topic) != topicoSubComando) return;

  msg.trim();
  msg.toUpperCase();

  if (msg == "INICIAR") {
    comandoIniciarCiclo = true;
  } else if (msg == "PARAR") {
    comandoPararCiclo = true;
  } else if (msg == "BOMBA_ON") {
    bombaManualOn = true;
  } else if (msg == "BOMBA_OFF") {
    bombaManualOff = true;
  } else {
    Serial.println("[MQTT] Comando nao reconhecido.");
  }
}

// =============================================================
// Publica no Serial e no MQTT ao mesmo tempo.
// =============================================================
void publicarStatus(const String &topico, const String &mensagem) {
  Serial.println(mensagem);
  if (MQTT.connected()) {
    MQTT.publish(topico.c_str(), mensagem.c_str());
  }
}

// =============================================================
// Executa um ciclo completo: varredura -> mira -> distancia ->
// agua -> disparo -> retorno ao neutro.
// =============================================================
void executarCicloDeBusca() {

  menorLeituraKY = 4095;
  anguloAlvo = ANGULO_NEUTRO;
  comandoPararCiclo = false;

  publicarStatus(topicoPubStatus, "[ETAPA 1] Iniciando varredura horizontal...");

  // ---- Varre de 90 -> 0 (esquerda) ----
  publicarStatus(topicoPubStatus, "[VARREDURA] Indo para a esquerda (90 -> 0)...");
  for (int ang = ANGULO_NEUTRO; ang >= ANGULO_MIN; ang -= PASSO_VARREDURA) {
    avaliarAngulo(ang);
    MQTT.loop();
    if (comandoPararCiclo) {
      publicarStatus(topicoPubStatus, "[VARREDURA] Ciclo cancelado por comando remoto.");
      moverServoSuave(ANGULO_NEUTRO);
      return;
    }
  }

  // ---- Varre de 0 -> 180 (direita), cobrindo todo o range ----
  publicarStatus(topicoPubStatus, "[VARREDURA] Indo para a direita (0 -> 180)...");
  for (int ang = ANGULO_MIN; ang <= ANGULO_MAX; ang += PASSO_VARREDURA) {
    avaliarAngulo(ang);
    MQTT.loop();
    if (comandoPararCiclo) {
      publicarStatus(topicoPubStatus, "[VARREDURA] Ciclo cancelado por comando remoto.");
      moverServoSuave(ANGULO_NEUTRO);
      return;
    }
  }

  {
    String msg = "[ETAPA 1] Varredura concluida. Menor leitura KY-026 = "
                 + String(menorLeituraKY) + " no angulo " + String(anguloAlvo) + " graus.";
    publicarStatus(topicoPubDeteccao, msg);
  }

  // ---- Retorna ao ponto do incêndio ----
  publicarStatus(topicoPubStatus, "[ETAPA 2] Retornando ao ponto do incendio ("
                 + String(anguloAlvo) + " graus)...");
  moverServoSuave(anguloAlvo);
  delay(300);

  // ---- Mede distância ----
  publicarStatus(topicoPubStatus, "[ETAPA 3] Medindo distancia com HC-SR04...");
  float distanciaCm = medirDistanciaCm();
  if (distanciaCm < 0) {
    publicarStatus(topicoPubDistancia, "[ETAPA 3] Falha na leitura do HC-SR04 (sem eco / fora de alcance).");
  } else {
    publicarStatus(topicoPubDistancia, "[ETAPA 3] Distancia ao alvo: " + String(distanciaCm) + " cm.");
  }

  // ---- Verifica nivel de agua ----
  publicarStatus(topicoPubStatus, "[ETAPA 4] Verificando nivel de agua na boia...");
  bool temAgua = (digitalRead(PIN_BOIA) == NIVEL_AGUA_PRESENTE);
  if (temAgua) {
    publicarStatus(topicoPubAgua, "[ETAPA 4] Agua disponivel no reservatorio.");
  } else {
    publicarStatus(topicoPubAgua, "[ETAPA 4] Reservatorio SEM agua suficiente.");
  }

  // ---- Override manual via MQTT tem prioridade sobre a logica automatica ----
  if (bombaManualOff) {
    bombaManualOff = false;
    digitalWrite(PIN_RELE, LOW);
    publicarStatus(topicoPubBomba, "[BOMBA] Desligada manualmente via MQTT.");
  } else if (bombaManualOn) {
    bombaManualOn = false;
    publicarStatus(topicoPubStatus, "[ETAPA 5] Acionamento manual via MQTT (ignora checagem de agua).");
    acionarBomba(TEMPO_BOMBA_MS);
  } else if (temAgua) {
    publicarStatus(topicoPubStatus, "[ETAPA 5] Condicoes atendidas. Acionando bomba...");
    acionarBomba(TEMPO_BOMBA_MS);
  } else {
    publicarStatus(topicoPubStatus, "[ETAPA 5] Disparo CANCELADO: nivel de agua insuficiente.");
  }

  // ---- Retorna ao neutro ----
  publicarStatus(topicoPubStatus, "[ETAPA 6] Retornando servo ao angulo neutro (90 graus)...");
  moverServoSuave(ANGULO_NEUTRO);
  publicarStatus(topicoPubStatus, "[ETAPA 6] Servo em posicao neutra.");
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
    String msg = "  [VARREDURA] Novo pico de chama detectado -> angulo "
                 + String(angulo) + ", leitura KY-026 = " + String(leituraMedia);
    Serial.println(msg);
    // Nao publica cada pico no MQTT para nao floodar o broker;
    // apenas o resultado final da varredura e publicado.
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
// Mantem o MQTT respirando durante a espera (delay de 4s).
// =============================================================
void acionarBomba(unsigned long tempoMs) {
  digitalWrite(PIN_RELE, HIGH);
  publicarStatus(topicoPubBomba, "[BOMBA] Bomba LIGADA.");

  unsigned long inicio = millis();
  while (millis() - inicio < tempoMs) {
    MQTT.loop();
    if (bombaManualOff) {
      bombaManualOff = false;
      break; // permite cortar a bomba manualmente antes do tempo acabar
    }
    delay(50);
  }

  digitalWrite(PIN_RELE, LOW);
  publicarStatus(topicoPubBomba, "[BOMBA] Bomba DESLIGADA.");
}
