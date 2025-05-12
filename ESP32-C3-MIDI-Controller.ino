#include <Arduino.h>
#include <BLEMidi.h>
#include "esp_bt.h"

// Definições dos faders
#define FADER_1_PIN 4         // GPIO 4 (ADC1_CHANNEL_3) - Fader 1
#define FADER_2_PIN 20        // GPIO 20 (ADC1_CHANNEL_2) - Fader 2
#define MIDI_CHANNEL 0        // Canal MIDI (0-15)
#define MIDI_CC_FADER_1 1     // Controlador MIDI CC para Fader 1 (Modulation Wheel)
#define MIDI_CC_FADER_2 2     // Controlador MIDI CC para Fader 2
#define VALUE_THRESHOLD 2     // Diferença mínima para enviar nova mensagem MIDI
#define FADER_MIN_RAW 500     // Valor mínimo do ADC (acima da posição mais baixa)
#define FADER_MAX_RAW 3500    // Valor máximo do ADC (abaixo da posição mais alta)

// Definições dos botões
#define BUTTON_1_PIN 2        // GPIO 2 (Botão 1)
#define BUTTON_2_PIN 3        // GPIO 3 (Botão 2)
#define BUTTON_3_PIN 0        // GPIO 0 (Botão 3)
#define BUTTON_4_PIN 1        // GPIO 1 (Botão 4)
#define BUTTON_5_PIN 5        // GPIO 5 (Botão 5)
#define BUTTON_6_PIN 6        // GPIO 6 (Botão 6)
#define BUTTON_7_PIN 7        // GPIO 7 (Botão 7)
#define BUTTON_8_PIN 9        // GPIO 9 (Botão 8)
#define BUTTON_9_PIN 10       // GPIO 10 (Botão 9)
#define DEBOUNCE_TIME 50      // Tempo de debounce em ms
#define MIDI_CC_BUTTON_1 17   // Controlador MIDI CC para Botão 1
#define MIDI_CC_BUTTON_2 18   // Controlador MIDI CC para Botão 2
#define MIDI_CC_BUTTON_3 19   // Controlador MIDI CC para Botão 3
#define MIDI_CC_BUTTON_4 20   // Controlador MIDI CC para Botão 4
#define MIDI_CC_BUTTON_5 21   // Controlador MIDI CC para Botão 5
#define MIDI_CC_BUTTON_6 22   // Controlador MIDI CC para Botão 6
#define MIDI_CC_BUTTON_7 23   // Controlador MIDI CC para Botão 7
#define MIDI_CC_BUTTON_8 24   // Controlador MIDI CC para Botão 8
#define MIDI_CC_BUTTON_9 25   // Controlador MIDI CC para Botão 9
#define BLE_OFF_HOLD_TIME 2000 // Tempo para desligar o BLE (ms)
#define BLE_ON_HOLD_TIME 1500  // Tempo para ligar o BLE (ms)

// Definições do LED
#define LED_PIN 8             // GPIO 8 do LED azul (lógica invertida apenas para BLE desativado: HIGH apaga)
#define LED_ON_TIME 100       // Tempo que o LED fica aceso para MIDI (ms)
#define LED_BLINK_INTERVAL 250 // Intervalo de piscada (ms, 4 vezes por segundo)
#define LED_BLINK_ON_TIME 100 // Tempo aceso por piscada (ms)

// Variáveis para os faders
int lastFader1Value = -1;     // Último valor enviado do Fader 1
int lastFader2Value = -1;     // Último valor enviado do Fader 2

// Variáveis para debounce dos botões
int lastButtonStates[9] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Estados anteriores dos botões
unsigned long lastDebounceTimes[9] = {0}; // Tempos de debounce para cada botão
const int buttonPins[9] = {BUTTON_1_PIN, BUTTON_2_PIN, BUTTON_3_PIN, BUTTON_4_PIN, BUTTON_5_PIN, BUTTON_6_PIN, BUTTON_7_PIN, BUTTON_8_PIN, BUTTON_9_PIN};
const int buttonCCs[9] = {MIDI_CC_BUTTON_1, MIDI_CC_BUTTON_2, MIDI_CC_BUTTON_3, MIDI_CC_BUTTON_4, MIDI_CC_BUTTON_5, MIDI_CC_BUTTON_6, MIDI_CC_BUTTON_7, MIDI_CC_BUTTON_8, MIDI_CC_BUTTON_9};

// Variáveis para o LED
unsigned long ledTurnOnTime = 0; // Quando o LED foi ligado (para eventos MIDI)
unsigned long lastBlinkTime = 0; // Última piscada (para estado desconectado)
bool ledIsOn = false;           // Estado do LED para eventos MIDI
bool blinkState = false;        // Estado da piscada (aceso/apagado)

// Variáveis para controle do BLE e botões duplos
bool bleEnabled = true;         // Estado do BLE (ligado/desligado)
unsigned long bothButtonsPressTime = 0; // Quando ambos os botões começaram a ser pressionados
bool bothButtonsPressed = false; // Ambos os botões 1 e 2 estão pressionados
bool actionTriggered = false;    // Controle para evitar múltiplos acionamentos

void sendMidiCC(uint8_t channel, uint8_t number, uint8_t value) {
  // Envia por BLE se habilitado e conectado
  if (bleEnabled && BLEMidiServer.isConnected()) {
    BLEMidiServer.controlChange(channel, number, value);
  }
  // Envia sempre pela serial
  uint8_t statusByte = 0xB0 | (channel & 0x0F); // Control Change no canal (0-15)
  Serial.write(statusByte);
  Serial.write(number & 0x7F); // Controlador (0-127)
  Serial.write(value & 0x7F);  // Valor (0-127)
}

void setup() {
  Serial.begin(38400); // Baud rate ajustado para compatibilidade com ttymidi
  delay(1000); // Aguarda a serial estabilizar
  Serial.println("Inicializando bluetooth e serial...");

  // Inicializa o servidor MIDI BLE
  BLEMidiServer.begin("Basic MIDI device");
  Serial.println("Aguardando conexões BLE...");
  Serial.print("Memória livre após inicialização: ");
  Serial.println(ESP.getFreeHeap());

  // Configura os pinos dos faders como entrada
  pinMode(FADER_1_PIN, INPUT);
  pinMode(FADER_2_PIN, INPUT);

  // Configura os pinos dos botões com pull-up interno
  for (int i = 0; i < 9; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Configura o pino do LED como saída
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED começa apagado (lógica padrão)
}

void loop() {
  // Verifica conexão BLE (apenas se BLE estiver habilitado)
  static bool wasConnected = false;
  bool isConnected = bleEnabled && BLEMidiServer.isConnected();

  if (isConnected && !wasConnected) {
    Serial.println("Conectado via BLE!");
    wasConnected = true;
    digitalWrite(LED_PIN, LOW); // Garante que o LED não pisca ao conectar
    ledIsOn = false;
    blinkState = false;
  } else if (!isConnected && wasConnected) {
    Serial.println("Desconectado!");
    wasConnected = false;
  }

  // Gerencia o LED para eventos MIDI
  if (ledIsOn && (millis() - ledTurnOnTime >= LED_ON_TIME)) {
    digitalWrite(LED_PIN, LOW); // Desliga o LED após 100 ms (lógica padrão)
    ledIsOn = false;
  }

  // Gerencia o LED: prioridade para BLE desativado
  if (!bleEnabled) {
    digitalWrite(LED_PIN, HIGH); // Força o LED apagado (lógica invertida)
    ledIsOn = false;
    blinkState = false;
  } else if (bleEnabled && !isConnected) {
    // Piscadas quando BLE habilitado e desconectado
    if (millis() - lastBlinkTime >= (blinkState ? LED_BLINK_ON_TIME : LED_BLINK_INTERVAL - LED_BLINK_ON_TIME)) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState ? HIGH : LOW); // HIGH acende, LOW apaga
      lastBlinkTime = millis();
    }
  } else if (isConnected && !ledIsOn) {
    // Garante que o LED esteja apagado quando conectado, exceto para eventos MIDI
    digitalWrite(LED_PIN, LOW); // Lógica padrão
    blinkState = false;
  }

  // --- Leitura dos Botões ---
  int button1State = digitalRead(BUTTON_1_PIN);
  int button2State = digitalRead(BUTTON_2_PIN);

  // Detecta pressionamento conjunto dos dois primeiros botões para BLE
  if (button1State == LOW && button2State == LOW) {
    if (!bothButtonsPressed) {
      bothButtonsPressed = true;
      bothButtonsPressTime = millis(); // Inicia contagem do tempo
      actionTriggered = false; // Reseta o controle de acionamento
    }
    // Verifica se o tempo de pressionamento foi atingido
    unsigned long holdTime = bleEnabled ? BLE_OFF_HOLD_TIME : BLE_ON_HOLD_TIME;
    if (bothButtonsPressed && !actionTriggered && (millis() - bothButtonsPressTime >= holdTime)) {
      if (bleEnabled) {
        BLEMidiServer.end(); // Desliga o BLE
        btStop(); // Para o rádio Bluetooth
        Serial.println("BLE desativado!");
        bleEnabled = false;
        digitalWrite(LED_PIN, HIGH); // Apaga o LED (lógica invertida)
        ledIsOn = false;
        blinkState = false;
        wasConnected = false;
      } else {
        btStart(); // Reinicia o rádio Bluetooth
        BLEMidiServer.begin("Basic MIDI device"); // Religar o BLE
        Serial.println("BLE ativado, aguardando conexões...");
        bleEnabled = true;
      }
      actionTriggered = true; // Marca que a ação foi executada
    }
  } else {
    bothButtonsPressed = false; // Reseta quando os botões são soltos
    actionTriggered = false; // Permite nova ação no próximo pressionamento
  }

  // --- Leitura dos Faders ---
  // Fader 1
  int fader1Raw = analogRead(FADER_1_PIN); // Lê valor analógico (0-4095)
  fader1Raw = constrain(fader1Raw, FADER_MIN_RAW, FADER_MAX_RAW);
  int fader1Value = map(fader1Raw, FADER_MIN_RAW, FADER_MAX_RAW, 0, 127);

  if (abs(fader1Value - lastFader1Value) > VALUE_THRESHOLD) {
    sendMidiCC(MIDI_CHANNEL, MIDI_CC_FADER_1, fader1Value);
    Serial.print("Fader 1 CC #");
    Serial.print(MIDI_CC_FADER_1);
    Serial.print(" = ");
    Serial.println(fader1Value);
    digitalWrite(LED_PIN, HIGH); // Lógica padrão
    ledTurnOnTime = millis();
    ledIsOn = true;
    lastFader1Value = fader1Value;
  }

  // Fader 2
  int fader2Raw = analogRead(FADER_2_PIN); // Lê valor analógico (0-4095)
  fader2Raw = constrain(fader2Raw, FADER_MIN_RAW, FADER_MAX_RAW);
  int fader2Value = map(fader2Raw, FADER_MIN_RAW, FADER_MAX_RAW, 0, 127);

  if (abs(fader2Value - lastFader2Value) > VALUE_THRESHOLD) {
    sendMidiCC(MIDI_CHANNEL, MIDI_CC_FADER_2, fader2Value);
    Serial.print("Fader 2 CC #");
    Serial.print(MIDI_CC_FADER_2);
    Serial.print(" = ");
    Serial.println(fader2Value);
    digitalWrite(LED_PIN, HIGH); // Lógica padrão
    ledTurnOnTime = millis();
    ledIsOn = true;
    lastFader2Value = fader2Value;
  }

  // --- Leitura dos Botões Individuais com Debounce ---
  for (int i = 0; i < 9; i++) {
    int buttonState = digitalRead(buttonPins[i]);
    if (buttonState != lastButtonStates[i]) {
      if (buttonState == LOW && (millis() - lastDebounceTimes[i] > DEBOUNCE_TIME)) {
        // Evita enviar MIDI dos botões 1 e 2 durante o pressionamento conjunto
        if ((i == 0 || i == 1) && bothButtonsPressed) continue;
        sendMidiCC(MIDI_CHANNEL, buttonCCs[i], 127);
        Serial.print("Botão ");
        Serial.print(i + 1);
        Serial.print(" pressionado: B0 ");
        Serial.print(buttonCCs[i], HEX);
        Serial.println(" 7F");
        digitalWrite(LED_PIN, HIGH); // Lógica padrão
        ledTurnOnTime = millis();
        ledIsOn = true;
        lastDebounceTimes[i] = millis();
      }
      lastButtonStates[i] = buttonState;
    }
  }

  // Atraso para aliviar a pilha BLE
  delay(20);
}
