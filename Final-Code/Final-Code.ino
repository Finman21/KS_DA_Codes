#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_TLC5947.h>

Adafruit_MCP23X17 mcp;
Adafruit_TLC5947 tlc = Adafruit_TLC5947(2, 3, 2, 4);

#define INTA_PIN 5
#define INTB_PIN 6
#define SEND_OK 1

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
int lastPwm[36];

unsigned long lastIrqMillis[12] = {0};
const unsigned long debounceMs = 20;

unsigned long lastReleasePoll = 0;
const unsigned long releasePollMs = 2;

static bool tlcDirty = false;
static unsigned long lastTlcWriteMicros = 0;
static const unsigned long tlcWriteIntervalUs = 1500;

static char rxBuf[64];
static uint8_t rxLen = 0;

static inline bool isDigitC(char c) {
  return (c >= '0' && c <= '9');
}

void markDirtyWriteSoon() {
  tlcDirty = true;
}

void flushTlcIfNeeded() {
  if (!tlcDirty) return;

  unsigned long now = micros();
  if (now - lastTlcWriteMicros < tlcWriteIntervalUs) return;

  tlc.write();
  lastTlcWriteMicros = now;
  tlcDirty = false;
}

void setLED_fast(int index, int pwm) {
  if (index < 0 || index >= 36) return;

  if (pwm < 0) pwm = 0;
  if (pwm > 4095) pwm = 4095;

  if (lastPwm[index] == pwm) return;
  lastPwm[index] = pwm;

  ledState[index] = (pwm > 0) ? 1 : 0;
  tlc.setPWM(ledChannels[index], pwm);
  markDirtyWriteSoon();

  for (int btn = 0; btn < 12; btn++) {
    if (btnR[btn] == index || btnG[btn] == index || btnB[btn] == index) {
      if (pwm > 0 && startMillis[btn] == 0) {
        startMillis[btn] = millis();
      }

      uint8_t r = btnR[btn];
      uint8_t g = btnG[btn];
      uint8_t b = btnB[btn];

      if (!ledState[r] && !ledState[g] && !ledState[b]) {
        startMillis[btn] = 0;
      }
      break;
    }
  }
}

void setBTN_fast(int btnIndex, int pwm) {
  if (btnIndex < 0 || btnIndex >= 12) return;

  setLED_fast(btnR[btnIndex], pwm);
  setLED_fast(btnG[btnIndex], pwm);
  setLED_fast(btnB[btnIndex], pwm);
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxLen == 0) continue;
      rxBuf[rxLen] = '\0';

      if (rxBuf[0] == 'A' && rxBuf[1] == 'L' && rxBuf[2] == 'L' && rxBuf[3] == '=') {
        int i = 4;
        int val = 0;
        while (isDigitC(rxBuf[i])) {
          val = val * 10 + (rxBuf[i] - '0');
          i++;
        }

        if (val == 0) {
          for (int k = 0; k < 36; k++) setLED_fast(k, 0);
#if SEND_OK
          Serial.println("OK");
#endif
        }
        rxLen = 0;
        continue;
      }

      if (rxBuf[0] == 'L' && rxBuf[1] == 'E' && rxBuf[2] == 'D') {
        int i = 3;
        int ledIndex = 0;
        while (isDigitC(rxBuf[i])) {
          ledIndex = ledIndex * 10 + (rxBuf[i] - '0');
          i++;
        }

        if (rxBuf[i] == '=') {
          i++;
          int val = 0;
          while (isDigitC(rxBuf[i])) {
            val = val * 10 + (rxBuf[i] - '0');
            i++;
          }

          if (ledIndex >= 1 && ledIndex <= 36) {
            setLED_fast(ledIndex - 1, val);
#if SEND_OK
            Serial.println("OK");
#endif
          }
        }
        rxLen = 0;
        continue;
      }

      if (rxBuf[0] == 'B' && rxBuf[1] == 'T' && rxBuf[2] == 'N') {
        int i = 3;
        int btnIndex = 0;
        while (isDigitC(rxBuf[i])) {
          btnIndex = btnIndex * 10 + (rxBuf[i] - '0');
          i++;
        }

        if (rxBuf[i] == '=') {
          i++;
          int val = 0;
          while (isDigitC(rxBuf[i])) {
            val = val * 10 + (rxBuf[i] - '0');
            i++;
          }

          if (btnIndex >= 1 && btnIndex <= 12) {
            setBTN_fast(btnIndex - 1, val);
#if SEND_OK
            Serial.println("OK");
#endif
          }
        }
        rxLen = 0;
        continue;
      }

      rxLen = 0;
      continue;
    }

    if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!mcp.begin_I2C(0x27)) {
    while (1) { }
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

  for (int i = 0; i < 36; i++) {
    lastPwm[i] = -1;
    tlc.setPWM(ledChannels[i], 0);
    ledState[i] = 0;
    lastPwm[i] = 0;
  }

  tlc.write();
  tlcDirty = false;
  lastTlcWriteMicros = micros();
}

void loop() {
  handleSerial();

  if (digitalRead(INTA_PIN) == LOW || digitalRead(INTB_PIN) == LOW) {
    handleMCPInterruptDrain();
  }

  updateButtonReleases();
  flushTlcIfNeeded();
}

void handleMCPInterruptDrain() {
  while (true) {
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
      continue;
    }

    uint16_t cap = mcp.getCapturedInterrupt();
    int current = (((cap >> intPin) & 0x01) == 0) ? 1 : 0;
    if (current != 1) {
      mcp.clearInterrupts();
      continue;
    }

    unsigned long now = millis();
    if (now - lastIrqMillis[btnIndex] < debounceMs) {
      mcp.clearInterrupts();
      continue;
    }
    lastIrqMillis[btnIndex] = now;

    if (buttonState[btnIndex] == 1) {
      mcp.clearInterrupts();
      continue;
    }
    buttonState[btnIndex] = 1;

    unsigned long reaction = 0;

    uint8_t r = btnR[btnIndex];
    uint8_t g = btnG[btnIndex];
    uint8_t b = btnB[btnIndex];

    if ((ledState[r] || ledState[g] || ledState[b]) && startMillis[btnIndex] > 0) {
      reaction = millis() - startMillis[btnIndex];
      setLED_fast(r, 0);
      setLED_fast(g, 0);
      setLED_fast(b, 0);
    }

    Serial.print("BTN");
    Serial.print(btnIndex + 1);
    Serial.print("=1;RT");
    Serial.print(btnIndex + 1);
    Serial.print("=");
    Serial.println(reaction);

    mcp.clearInterrupts();
  }
}

void updateButtonReleases() {
  unsigned long now = millis();
  if (now - lastReleasePoll < releasePollMs) return;

  lastReleasePoll = now;
  uint16_t gpio = mcp.readGPIOAB();

  for (int i = 0; i < 12; i++) {
    if (((gpio >> buttonPins[i]) & 0x01) == 1) {
      buttonState[i] = 0;
    }
  }
}