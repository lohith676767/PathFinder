// Runs on the SECOND ESP32 - replaces the WiFi web UI with a 0.96" SSD1306
// OLED + 2 push buttons (or a jumper wire touched to GND, electrically the
// same thing on an INPUT_PULLUP pin). No WiFi at all.
//
// Wiring (common GND with the main ESP required):
//   OLED   SDA = GPIO21   SCL = GPIO22   (I2C)
//   Button NEXT   = GPIO26
//   Button SELECT = GPIO23
//   UART   TX = GPIO19  ---->  Main ESP  RX2 = GPIO4
//   UART   RX = GPIO18  <----  Main ESP  TX2 = GPIO27
//
// UI is a small 2-button menu over 3 items (Kp, Kd, Speed):
//   MENU mode  - NEXT: move the highlighted item to the next one (wraps
//                       Kp -> Kd -> Speed -> Kp -> ...)
//                SELECT: enter EDIT mode on the highlighted item
//   EDIT mode  - SELECT: increase that item's value, wrapping back to its
//                        minimum once past the max
//                NEXT: back out to MENU mode (without changing item)
//   Either button, LONG-PRESSED (>=600ms): toggle Start/Stop, regardless
//   of MENU/EDIT mode.
//
// Each button pin is wired to GND through the switch (or jumper) and uses
// INPUT_PULLUP, so a press reads LOW.
//
// Library needed: "Adafruit SSD1306" (and its dependency "Adafruit GFX
// Library") - install both via Library Manager.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

// Persists Kp/Kd/Speed to flash (NVS) so they survive a power cycle.
// botRunning is intentionally NOT persisted - always boot stopped, so the
// bot never powers on already "running" from before it lost power.
Preferences prefs;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int BTN_NEXT = 26;
const int BTN_SELECT = 23;

const unsigned long DEBOUNCE_MS = 40;
const unsigned long LONG_PRESS_MS = 600;

HardwareSerial UARTLink(2); // RX=18, TX=19 (avoids 16/17 - see line_follower.ino wiring notes)

// Live-tunable values, cycled by the buttons and forwarded to the main ESP.
int masterSpeed = 100;
float Kp = 0.18;
float Kd = 0.0;
bool botRunning = false;

// Cycling ranges/steps - wraps back to the lower bound once past the max.
const int SPEED_MIN = 60, SPEED_MAX = 220, SPEED_STEP = 10;
const float KP_MIN = 0.00f, KP_MAX = 0.30f, KP_STEP = 0.02f;
const float KD_MIN = 0.00f, KD_MAX = 3.00f, KD_STEP = 0.10f;

bool isCalibrated = false;   // last known state from main ESP
bool remoteRunning = false;  // last confirmed run state from main ESP

// Menu state: which of the 3 items is highlighted/being edited, and whether
// we're currently browsing the menu or editing the highlighted item.
enum UIMode { MODE_MENU, MODE_EDIT };
UIMode uiMode = MODE_MENU;
int selectedVar = 0; // 0 = Kp, 1 = Kd, 2 = Speed

const int NUM_SENSORS = 8;
int sensorValues[NUM_SENSORS] = {0}; // latest normalized (0=white..1000=black) readings

char uartBuffer[96];
int uartLen = 0;

bool displayDirty = true;      // redraw requested (button press or new data)
unsigned long lastDisplayDraw = 0;

// -------------------------------------------------------------
// Button handling: a short press drives the menu/edit navigation (see
// loop()), while a long press on either button toggles start/stop instead.
// Each button gets its own small debounced state machine.
// -------------------------------------------------------------
struct ButtonState {
  int pin;
  bool lastStable = true;   // true = released (pulled up)
  bool lastRaw = true;
  unsigned long lastChangeTime = 0;
  unsigned long pressStartTime = 0;
};

ButtonState btnNext = { BTN_NEXT };
ButtonState btnSelect = { BTN_SELECT };

void cycleKp() {
  Kp += KP_STEP;
  if (Kp > KP_MAX + 0.001f) Kp = KP_MIN;
  displayDirty = true;
}

void cycleKd() {
  Kd += KD_STEP;
  if (Kd > KD_MAX + 0.001f) Kd = KD_MIN;
  displayDirty = true;
}

void cycleSpeed() {
  masterSpeed += SPEED_STEP;
  if (masterSpeed > SPEED_MAX) masterSpeed = SPEED_MIN;
  displayDirty = true;
}

// Increases whichever of the 3 items is currently highlighted/being edited.
void incrementSelected() {
  switch (selectedVar) {
    case 0: cycleKp(); break;
    case 1: cycleKd(); break;
    case 2: cycleSpeed(); break;
  }
  saveSettings();
}

void loadSettings() {
  prefs.begin("linefollow", false);
  masterSpeed = prefs.getInt("speed", masterSpeed);
  Kp = prefs.getFloat("kp", Kp);
  Kd = prefs.getFloat("kd", Kd);
}

void saveSettings() {
  prefs.putInt("speed", masterSpeed);
  prefs.putFloat("kp", Kp);
  prefs.putFloat("kd", Kd);
}

void nextMenuItem() {
  selectedVar = (selectedVar + 1) % 3;
  displayDirty = true;
}

void toggleRun() {
  if (!isCalibrated) return; // ignore start attempts before calibration finishes
  botRunning = !botRunning;
  displayDirty = true;
}

// Returns true exactly once per completed short press (release before
// LONG_PRESS_MS). Long presses are reported via `longPressOut`.
bool pollButton(ButtonState &b, bool &longPressOut) {
  longPressOut = false;
  bool raw = digitalRead(b.pin); // HIGH = released, LOW = pressed

  if (raw != b.lastRaw) {
    b.lastChangeTime = millis();
    b.lastRaw = raw;
  }

  if ((millis() - b.lastChangeTime) > DEBOUNCE_MS && raw != b.lastStable) {
    b.lastStable = raw;
    if (raw == LOW) {
      // just pressed
      b.pressStartTime = millis();
    } else {
      // just released - decide short vs long press
      unsigned long heldFor = millis() - b.pressStartTime;
      if (heldFor >= LONG_PRESS_MS) {
        longPressOut = true;
      } else {
        return true; // short press
      }
    }
  }
  return false;
}

// -------------------------------------------------------------
// UART: same "C,<speed>,<kp>,<kd>,<run>\n" / "S,.." / "D,.." protocol as
// the WiFi version used, so the main ESP (line_follower.ino) needs no
// changes at all.
// -------------------------------------------------------------
void sendCommand() {
  UARTLink.print("C,");
  UARTLink.print(masterSpeed);
  UARTLink.print(",");
  UARTLink.print(Kp, 3);
  UARTLink.print(",");
  UARTLink.print(Kd, 3);
  UARTLink.print(",");
  UARTLink.println(botRunning ? 1 : 0);
}

void handleUARTLine(char* line) {
  if (line[0] == 'S' && line[1] == ',') {
    char* save;
    char* tok = strtok_r(line + 2, ",", &save);
    if (!tok) return;
    int cal = atoi(tok);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) return;
    int run = atoi(tok);

    isCalibrated = (cal == 1);
    remoteRunning = (run == 1);
    displayDirty = true;
  } else if (line[0] == 'D' && line[1] == ',') {
    char* save;
    char* tok = strtok_r(line + 2, ",", &save);
    int i = 0;
    while (tok && i < NUM_SENSORS) {
      sensorValues[i++] = atoi(tok);
      tok = strtok_r(NULL, ",", &save);
    }
    displayDirty = true;
  }
}

void readUARTStatus() {
  while (UARTLink.available()) {
    char c = UARTLink.read();
    if (c == '\n') {
      uartBuffer[uartLen] = '\0';
      handleUARTLine(uartBuffer);
      uartLen = 0;
    } else if (c != '\r') {
      if (uartLen < (int)sizeof(uartBuffer) - 1) {
        uartBuffer[uartLen++] = c;
      } else {
        uartLen = 0; // overflow guard: discard oversized/garbled line
      }
    }
  }
}

// -------------------------------------------------------------
// OLED drawing: 8 black/white boxes for the sensor line (no numeric
// values), plus the current Kp/Kd/Speed/run-state below.
// -------------------------------------------------------------
void drawDisplay() {
  display.clearDisplay();

  // 8 sensor boxes, left-to-right = D1..D8, filled = line detected (black),
  // outline-only = no line (white). Same detection threshold as the main
  // ESP's noise filter (normVal < 200 is treated as no-line).
  const int boxW = 14, boxH = 14, gap = 2, startX = 2, startY = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = startX + i * (boxW + gap);
    bool lineHere = sensorValues[i] > 500;
    if (lineHere) {
      display.fillRect(x, startY, boxW, boxH, SSD1306_WHITE);
    } else {
      display.drawRect(x, startY, boxW, boxH, SSD1306_WHITE);
    }
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Menu list: a ">" marks the highlighted item in MENU mode, or the item
  // is wrapped in [brackets] while actively being edited in EDIT mode.
  const char* labels[3] = {"Kp: ", "Kd: ", "Speed: "};
  int rowY[3] = {22, 34, 46};
  for (int i = 0; i < 3; i++) {
    display.setCursor(0, rowY[i]);
    bool isSelected = (i == selectedVar);
    bool editingThis = isSelected && (uiMode == MODE_EDIT);

    display.print(editingThis ? "[" : (isSelected ? "> " : "  "));
    display.print(labels[i]);
    switch (i) {
      case 0: display.print(Kp, 2); break;
      case 1: display.print(Kd, 2); break;
      case 2: display.print(masterSpeed); break;
    }
    if (editingThis) display.print("]");
  }

  display.setCursor(0, 56);
  if (!isCalibrated) {
    display.print("CALIBRATING...");
  } else if (remoteRunning) {
    display.print("RUNNING");
  } else {
    display.print("READY / PAUSED");
  }

  display.display();
}

void setup() {
  Serial.begin(115200);
  UARTLink.begin(115200, SERIAL_8N1, 18, 19); // RX=GPIO18, TX=GPIO19

  loadSettings(); // restore Kp/Kd/Speed saved before the last power-off

  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  Wire.begin(21, 22); // SDA=GPIO21, SCL=GPIO22

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (true) delay(1000);
  }
  display.clearDisplay();
  display.display();

  sendCommand(); // push the restored Kp/Kd/Speed to the main ESP right away
}

void loop() {
  readUARTStatus();

  bool nextLong, selectLong;
  bool nextShort = pollButton(btnNext, nextLong);
  bool selectShort = pollButton(btnSelect, selectLong);

  if (nextLong || selectLong) {
    // Either button, long-pressed: start/stop, regardless of menu/edit mode.
    toggleRun();
    sendCommand();
  } else {
    if (nextShort) {
      if (uiMode == MODE_MENU) {
        nextMenuItem();
      } else {
        uiMode = MODE_MENU; // back out of editing without changing item
        displayDirty = true;
      }
    }

    if (selectShort) {
      if (uiMode == MODE_MENU) {
        uiMode = MODE_EDIT;
        displayDirty = true;
      } else {
        incrementSelected();
        sendCommand();
      }
    }
  }

  if (displayDirty && millis() - lastDisplayDraw > 100) {
    drawDisplay();
    lastDisplayDraw = millis();
    displayDirty = false;
  }
}
