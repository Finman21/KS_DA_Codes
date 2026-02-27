#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_TLC5947.h>

Adafruit_MCP23X17 mcp;
Adafruit_TLC5947 tlc = Adafruit_TLC5947(2, 3, 2, 4);

#define INTA_PIN 5
#define INTB_PIN 6

const uint8_t buttonPins[12] = {
  3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15
};

const int ledChannels[36] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
  12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35
};

const uint8_t btnR[12] = { 0, 3, 6, 9, 23, 20, 17, 14, 35, 29, 32, 26 };
const uint8_t btnG[12] = { 1, 4, 7, 10, 22, 19, 16, 13, 34, 28, 31, 25 };
const uint8_t btnB[12] = { 2, 5, 8, 11, 21, 18, 15, 12, 33, 27, 30, 24 };

unsigned long startMillis[12] = {0};
int buttonState[12] = {0};
int ledState[36] = {0};

unsigned long lastIrqMillis[12] = {0};
const unsigned long debounceMs = 30;

void handleMCPInterrupt(char portName);
void setLED(int index, int state);

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Interrupt-Reaktionsspiel gestartet...");

  if (!mcp.begin_I2C(0x27)) {
    Serial.println("Fehler: MCP23017 nicht gefunden!");
    while (1);
  }

  mcp.setupInterrupts(false, false, LOW);

  for (int i = 0; i < 12; i++) {
    mcp.pinMode(buttonPins[i], INPUT_PULLUP);
    mcp.setupInterruptPin(buttonPins[i], CHANGE);
  }

  pinMode(INTA_PIN, INPUT_PULLUP);
  pinMode(INTB_PIN, INPUT_PULLUP);
  mcp.clearInterrupts();

  tlc.begin();
  tlc.write();

  Serial.println("Setup abgeschlossen – warte auf Ereignisse...");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    int ledIndex, val;
    if (sscanf(cmd.c_str(), "LED%d=%d", &ledIndex, &val) == 2) {
      if (ledIndex >= 1 && ledIndex <= 36) {
        setLED(ledIndex - 1, val);
      }
    }
  }

  if (digitalRead(INTA_PIN) == LOW) {
    handleMCPInterrupt('A');
    delay(2);
  }

  if (digitalRead(INTB_PIN) == LOW) {
    handleMCPInterrupt('B');
    delay(2);
  }
}

void handleMCPInterrupt(char portName) {
  uint8_t intPin = mcp.getLastInterruptPin();
  if (intPin == 255) {
    mcp.clearInterrupts();
    return;
  }

  int btnIndex = -1;
  for (int i = 0; i < 12; i++) {
    if (buttonPins[i] == intPin) {
      btnIndex = i;
      break;
    }
  }

  if (btnIndex == -1) {
    mcp.clearInterrupts();
    return;
  }

  uint8_t pinVal = mcp.digitalRead(intPin);
  int current = (pinVal == LOW) ? 1 : 0;

  unsigned long now = millis();
  if (now - lastIrqMillis[btnIndex] < debounceMs) {
    mcp.clearInterrupts();
    return;
  }
  lastIrqMillis[btnIndex] = now;

  int prev = buttonState[btnIndex];
  buttonState[btnIndex] = current;

  if (!(prev == 0 && current == 1)) {
    mcp.clearInterrupts();
    return;
  }

  unsigned long reaction = 0;

  uint8_t r = btnR[btnIndex];
  uint8_t g = btnG[btnIndex];
  uint8_t b = btnB[btnIndex];

  bool anyOn = (ledState[r] || ledState[g] || ledState[b]);

  if (anyOn && startMillis[btnIndex] > 0) {
    reaction = millis() - startMillis[btnIndex];

    setLED(r, 0);
    setLED(g, 0);
    setLED(b, 0);
  }

  Serial.print("BTN");
  Serial.print(btnIndex + 1);
  Serial.print("=");
  Serial.print(1);
  Serial.print(";RT");
  Serial.print(btnIndex + 1);
  Serial.print("=");
  Serial.println(reaction);

  mcp.clearInterrupts();
}

void setLED(int index, int state) {
  if (index < 0 || index >= 36) 
    return;

  int on = (state > 0) ? 1 : 0;

  ledState[index] = on;
  tlc.setPWM(ledChannels[index], on ? 4095 : 0);
  tlc.write();

  for (int btn = 0; btn < 12; btn++) {
    if (btnR[btn] == index || btnG[btn] == index || btnB[btn] == index) {

      if (on == 1 && startMillis[btn] == 0) {
        startMillis[btn] = millis();
      }

      uint8_t r = btnR[btn];
      uint8_t g = btnG[btn];
      uint8_t b = btnB[btn];

      if (ledState[r] == 0 && ledState[g] == 0 && ledState[b] == 0) {
        startMillis[btn] = 0;
      }

      break;
    }
  }
}