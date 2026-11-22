// NEXUS OS + NEXUS PET integration.
// Uses FluxGarage RoboEyes by Dennis Hoelscher (GPL-3.0):
// https://github.com/FluxGarage/RoboEyes
// Install "FluxGarage RoboEyes" from Arduino IDE Library Manager before compiling.
// Modified project integration; RoboEyes library source is not bundled in this sketch.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_random.h>
#include <esp_mac.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BleCombo.h>

// ================= ЗАСТАВКА (КОТИК 128x64) =================
const unsigned char myBitmap[] PROGMEM = {
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xef, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xd7, 0xff, 0xff, 0xff, 0xff, 0xcb, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xdb, 0xff, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xbd, 0xff, 0xff, 0xff, 0xfe, 0x7d, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xbc, 0xff, 0xff, 0xff, 0xfc, 0xfd, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0x7e, 0xff, 0xff, 0xff, 0xf9, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0xff, 0xff, 0xf3, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0x7f, 0xbf, 0xff, 0xff, 0xe7, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0xff, 0x9f, 0xff, 0xff, 0xef, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0xff, 0xdf, 0x87, 0xff, 0xdf, 0xfe, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0xff, 0xef, 0xbc, 0x7f, 0xbf, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfc, 0xff, 0xf7, 0xbf, 0x3f, 0xbf, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xf3, 0xbf, 0xcf, 0x7f, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xfb, 0xdf, 0xe7, 0x7f, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xf2, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xfe, 0xcf, 0xf8, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xfe, 0x1f, 0xfd, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xf8, 0x03, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0xf8, 0x00, 0x7f, 0xe0, 0x01, 0xfd, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0x7e, 0xe0, 0x3f, 0xe0, 0x77, 0xfb, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0x7c, 0xe0, 0x3f, 0xe0, 0x7b, 0xf3, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0x3d, 0xe0, 0x3f, 0xe0, 0x7b, 0xe7, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xbd, 0xe0, 0x3f, 0xe0, 0x7b, 0xc0, 0x7f, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xf0, 0x1d, 0xe0, 0x7f, 0xe0, 0x7b, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xf7, 0xfd, 0xe0, 0x7f, 0xe0, 0x7b, 0xfe, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfb, 0xfd, 0xf0, 0x7f, 0xe0, 0x7b, 0xfd, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xf9, 0xfe, 0xf0, 0x7f, 0xf0, 0xfb, 0xf3, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xf1, 0xff, 0xff, 0xef, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0x73, 0xff, 0xff, 0xff, 0xe3, 0xe7, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0x68, 0xff, 0xff, 0xff, 0xf7, 0xf7, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0xfb, 0xff, 0xfb, 0xef, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xf0, 0x0f, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x41, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xfc, 0x01, 0xff, 0xff, 0xff, 0xf3, 0xbf, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0x87, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xd0, 0x7f, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xef, 0xff, 0xff, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xef, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xe1, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00
};

// ================= НОВАЯ РАСПИНОВКА =================
#define OLED_SDA 1
#define OLED_SCL 2

const int IR_RECV_PIN = 8;
const int IR_SEND_PIN = 9;

// Кнопки
const int BTN_UP = 4, BTN_LEFT = 7, BTN_RIGHT = 6, BTN_DOWN = 5, BTN_SELECT = 3;
const int BTN_PINS[] = {BTN_LEFT, BTN_SELECT, BTN_RIGHT, BTN_UP, BTN_DOWN};
#define NUM_BTNS 5
#define B_LEFT 0
#define B_SEL  1
#define B_RIGHT 2
#define B_UP   3
#define B_DOWN 4

Adafruit_SSD1306 display(128, 64, &Wire, -1);
RoboEyes<Adafruit_SSD1306> roboEyes(display);

#define PET_SHAKE_PIN 10
IRrecv irrecv(IR_RECV_PIN, 1024, 15, true);
IRsend irsend(IR_SEND_PIN);
decode_results results;
Preferences preferences;
WebServer server(80);
bool wifiServerActive = false;

int currentMode = 0;
bool inMenu = true;

// menuModeIds: добавлены 29 (IR RANDOM) и 30 (WIFI DEAUTH)
const int menuModeIds[] = {
  0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
  12, 13, 14, 15, 16, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30
};
const int totalModes = sizeof(menuModeIds) / sizeof(menuModeIds[0]);

const char* modeNames[] = {
  "IR RECORDER", "IR SAVED", "SYSTEM INFO", "WIFI SERVER", "BLE SCAN",
  "3D CUBE", "SNAKE", "TETRIS", "PONG", "FLAPPY", "GEO DASH",
  "INVADERS", "STACK TOWER", "PULSE DODGE", "SIMON", "T-REX",
  "BREAKOUT", "2048", "MAZE", "WHACK-A-MOLE", "3D MAZE", "TV REMOTE", "BADUSB LAB", "NEXUS PET", "BLE HID",
  "IR RANDOM", "WIFI DEAUTH"
};

const char* modeDesc[] = {
  "Learn a remote button", "Saved remote buttons", "Device information",
  "Local status web page", "Nearby BLE devices", "3D animation",
  "Classic snake game", "Block puzzle", "Arcade game", "Tap to fly",
  "Platform runner", "Arcade shooter", "Build a tower", "Dodge falling pulses",
  "Memory game", "Dinosaur runner", "Breakout game", "Number puzzle",
  "Find the exit", "Reaction game", "First-person maze", "TV remote learning", "Safe USB/HID info screen", "Your pocket companion", "Manual Bluetooth keyboard",
  "One IR test per UP press", "Nearby WiFi network scanner"
};

bool btnCur[NUM_BTNS]={false}, btnPrev[NUM_BTNS]={false}, btnLongFired[NUM_BTNS]={false};
bool btnPressed(int i);
void petNoteActivity();
void petWake(bool shaken);
uint32_t btnPressStart[NUM_BTNS]={0}, btnLastRelease[NUM_BTNS]={0};

// ================= ЗВУК ОТКЛЮЧЕН =================
void beepAsync(int freq, int dur) {}
void beepUpdate() {}
void sfxClick() {}
void sfxEnter() {}
void sfxEat() {}
void sfxShoot() {}
void sfxHit() {}
void sfxJump() {}
void sfxDeath() {}
void sfxWin() {}
void sfxLine() {}

void buttonsScan() {
  uint32_t now = millis();
  for (int i = 0; i < NUM_BTNS; i++) {
    btnPrev[i] = btnCur[i];
    bool raw = (digitalRead(BTN_PINS[i]) == LOW);
    if (raw != btnCur[i]) {
      if (raw) {
        if (now - btnLastRelease[i] > 30) {
          btnCur[i] = true; btnPressStart[i] = now; btnLongFired[i] = false;
        }
      } else {
        btnCur[i] = false; btnLastRelease[i] = now;
      }
    }
  }
  for (int i = 0; i < NUM_BTNS; i++) if (btnPressed(i)) { petNoteActivity(); break; }
}
bool btnPressed(int i) { return btnCur[i] && !btnPrev[i]; }
bool btnHeld(int i) { return btnCur[i]; }
bool btnLongPress(int i, uint32_t ms) {
  if (btnCur[i] && !btnLongFired[i] && millis() - btnPressStart[i] >= ms) {
    btnLongFired[i] = true; return true;
  }
  return false;
}
bool wantExit() { return btnLongPress(B_SEL, 2000); }

// ================= NEXUS PET =================
int petEnergy = 60;
uint32_t petXP = 0;
uint32_t petLastDrainMs = 0;
uint32_t petLastActivityMs = 0;
uint32_t petLastShakeMs = 0;
uint32_t petShakeWindowMs = 0;
uint32_t petReactionUntil = 0;
uint32_t petNextMoodMs = 0;
uint8_t petShakeCount = 0;
bool petSleeping = false;
bool petShakePrev = HIGH;
bool petRoboReady = false;
bool petSweatActive = false;
bool petLaunched = false;

void petNoteActivity() {
  petLastActivityMs = millis();
  if (petSleeping) {
    petSleeping = false;
    if (petRoboReady) {
      roboEyes.open();
      roboEyes.setMood(DEFAULT);
      roboEyes.setAutoblinker(ON, 3, 2);
      roboEyes.setIdleMode(ON, 2, 2);
    }
  }
}

void petSave() {
  preferences.begin("nexus_pet", false);
  preferences.putUChar("energy", (uint8_t)constrain(petEnergy, 0, 100));
  preferences.putUInt("xp", petXP);
  preferences.end();
}

void petLoad() {
  preferences.begin("nexus_pet", true);
  petEnergy = preferences.getUChar("energy", 60);
  petXP = preferences.getUInt("xp", 0);
  preferences.end();
  petEnergy = constrain(petEnergy, 0, 100);
}

void petAward(int energy, uint32_t xp) {
  petEnergy = constrain(petEnergy + energy, 0, 100);
  petXP += xp;
  petLastActivityMs = millis();
  if (petSleeping) petWake(false);
  petSave();
}

void petInit() {
  pinMode(PET_SHAKE_PIN, INPUT_PULLUP);
  petShakePrev = digitalRead(PET_SHAKE_PIN);
  petLastActivityMs = millis();
  petLastDrainMs = millis();
  petNextMoodMs = millis() + 5000;
  randomSeed((uint32_t)esp_random());

  roboEyes.begin(128, 64, 60);
  roboEyes.setWidth(38, 38);
  roboEyes.setHeight(38, 38);
  roboEyes.setBorderradius(12, 12);
  roboEyes.setSpacebetween(10);
  roboEyes.setCuriosity(ON);
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  roboEyes.setMood(DEFAULT);
  roboEyes.setPosition(DEFAULT);
  petRoboReady = true;
}

void petSetNormal() {
  if (!petRoboReady) return;
  roboEyes.setSweat(OFF);
  roboEyes.setHFlicker(OFF);
  roboEyes.setVFlicker(OFF);
  roboEyes.setMood(DEFAULT);
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  petSweatActive = false;
}

void petWake(bool shaken) {
  petSleeping = false;
  petLastActivityMs = millis();
  petReactionUntil = millis() + 1800;
  if (petRoboReady) {
    roboEyes.open();
    petSetNormal();
    if (shaken) {
      roboEyes.setMood(HAPPY);
      roboEyes.anim_laugh();
    }
  }
}

void petReactToShake(uint8_t count) {
  if (!petRoboReady) return;
  uint32_t now = millis();
  petLastActivityMs = now;
  petReactionUntil = now + 2200;
  roboEyes.setPosition(DEFAULT);
  roboEyes.setSweat(OFF);
  petSweatActive = false;

  if (count <= 1) {
    roboEyes.setMood(DEFAULT);
    roboEyes.blink();
    roboEyes.setPosition(NE);
  } else if (count == 2) {
    roboEyes.setMood(TIRED);
    roboEyes.anim_confused();
  } else if (count == 3) {
    roboEyes.setMood(HAPPY);
    roboEyes.anim_laugh();
  } else {
    roboEyes.setMood(ANGRY);
    roboEyes.setHFlicker(ON, 2);
    roboEyes.setSweat(ON);
    petSweatActive = true;
  }
}

void petUpdateBackground() {
  uint32_t now = millis();
  bool shakeNow = digitalRead(PET_SHAKE_PIN);

  if (petShakePrev == HIGH && shakeNow == LOW && now - petLastShakeMs > 130) {
    petLastShakeMs = now;
    petLastActivityMs = now;
    if (petSleeping) {
      petWake(true);
      petShakeCount = 0;
      petShakeWindowMs = 0;
    } else {
      if (petShakeWindowMs == 0 || now - petShakeWindowMs > 750) {
        petShakeCount = 0;
        petShakeWindowMs = now;
      }
      if (petShakeCount < 5) petShakeCount++;
    }
  }
  petShakePrev = shakeNow;

  if (!petSleeping && petShakeCount > 0 && now - petLastShakeMs > 260) {
    petReactToShake(petShakeCount);
    petShakeCount = 0;
    petShakeWindowMs = 0;
  }

  if (!petSleeping && now >= petNextMoodMs && now >= petReactionUntil) {
    uint8_t mood = random(0, 5);
    switch (mood) {
      case 0: roboEyes.setMood(DEFAULT); break;
      case 1: roboEyes.setMood(HAPPY); break;
      case 2: roboEyes.setMood(DEFAULT); roboEyes.setPosition(W); break;
      case 3: roboEyes.setMood(DEFAULT); roboEyes.setPosition(E); break;
      default: roboEyes.setMood(DEFAULT); roboEyes.blink(); break;
    }
    petNextMoodMs = now + (uint32_t)random(4500, 9500);
  }

  if (petReactionUntil && now >= petReactionUntil) {
    petReactionUntil = 0;
    petSetNormal();
  }

  if (now - petLastDrainMs >= 30UL * 60UL * 1000UL) {
    petLastDrainMs = now;
    if (petEnergy > 0) { petEnergy--; petSave(); }
  }

  if (!petSleeping && now - petLastActivityMs >= 10UL * 60UL * 1000UL) {
    petSleeping = true;
    petReactionUntil = 0;
    if (petRoboReady) {
      roboEyes.setIdleMode(OFF, 0, 0);
      roboEyes.setAutoblinker(OFF, 0, 0);
      roboEyes.setSweat(OFF);
      roboEyes.setMood(TIRED);
      roboEyes.close();
    }
  }
}

void drawPetMenuEasterEgg() {
  uint32_t phase = (millis() / 7000UL) % 5;
  if (phase == 0 || phase == 1) {
    int y = 3;
    display.drawRoundRect(105, y, 7, 8, 3, SSD1306_WHITE);
    display.drawRoundRect(116, y, 7, 8, 3, SSD1306_WHITE);
    if ((millis() / 180UL) % 13 == 0) {
      display.drawLine(105, y+4, 111, y+4, SSD1306_WHITE);
      display.drawLine(116, y+4, 122, y+4, SSD1306_WHITE);
    } else {
      display.drawPixel(108, y+4, SSD1306_WHITE);
      display.drawPixel(119, y+4, SSD1306_WHITE);
    }
  }
}

void drawPetStatsScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("NEXUS PET / STATUS");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 17); display.print("ENERGY");
  display.drawRect(48, 17, 76, 8, SSD1306_WHITE);
  int barWidth = map(constrain(petEnergy, 0, 100), 0, 100, 0, 72);
  if (barWidth > 0) display.fillRect(50, 19, barWidth, 4, SSD1306_WHITE);
  display.setCursor(0, 31); display.print(petEnergy); display.print(" / 100");

  display.setCursor(0, 42); display.print("XP: "); display.print(petXP);
  display.setCursor(72, 42); display.print("LVL: "); display.print((petXP / 100) + 1);
  display.drawLine(0, 53, 127, 53, SSD1306_WHITE);
  display.setCursor(0, 56); display.print("TAP SELECT: START");
  display.display();
}

void drawNexusPet() {
  buttonsScan();
  if (wantExit()) {
    petLaunched = false;
    inMenu = true;
    delay(150);
    return;
  }

  if (!petLaunched) {
    if (btnPressed(B_SEL)) {
      petLaunched = true;
      petNoteActivity();
      delay(100);
      return;
    }
    drawPetStatsScreen();
    delay(12);
    return;
  }

  if (btnPressed(B_LEFT)) { if (!petSleeping) roboEyes.setPosition(W); petLastActivityMs = millis(); }
  if (btnPressed(B_RIGHT)) { if (!petSleeping) roboEyes.setPosition(E); petLastActivityMs = millis(); }
  if (btnPressed(B_UP)) { if (!petSleeping) roboEyes.setPosition(N); petLastActivityMs = millis(); }
  if (btnPressed(B_DOWN)) { if (!petSleeping) roboEyes.setPosition(S); petLastActivityMs = millis(); }
  if (btnPressed(B_SEL)) {
    if (petSleeping) petWake(false);
    else {
      roboEyes.setMood(HAPPY);
      roboEyes.anim_laugh();
      petReactionUntil = millis() + 1800;
      petAward(2, 3);
    }
  }
  if (petSleeping && (btnPressed(B_LEFT) || btnPressed(B_RIGHT) || btnPressed(B_UP) || btnPressed(B_DOWN))) petWake(false);

  if (petSleeping) {
    display.clearDisplay();
    display.drawRoundRect(23, 23, 34, 8, 4, SSD1306_WHITE);
    display.drawRoundRect(71, 23, 34, 8, 4, SSD1306_WHITE);
    display.drawLine(27, 27, 53, 27, SSD1306_WHITE);
    display.drawLine(75, 27, 101, 27, SSD1306_WHITE);
    display.setTextSize(1); display.setCursor(0, 56); display.print("NEXUS PET  zzz");
    display.display();
  } else {
    roboEyes.update();
  }
}

// ================= IR & FLASH =================
static const uint16_t IR_RAW_MAX = 300;
struct SavedCode { char name[16]; uint16_t rawData[IR_RAW_MAX]; uint16_t rawLen; uint64_t value; uint16_t protocol; };
SavedCode savedCodes[10];
int savedCount = 0;
uint16_t currentRaw[IR_RAW_MAX];
uint16_t currentRawLen = 0;
uint64_t currentDecodedValue = 0;
uint16_t currentDecodedProtocol = 0;
bool irCodePrinted = false;
char nextIRName[16] = "";
int irState = 0;
uint32_t irLastAction = 0;

void irSendRaw(uint16_t* data, uint16_t len) {
  if (!data || len == 0 || len > IR_RAW_MAX) return;
  irrecv.disableIRIn();
  delay(5);
  irsend.sendRaw(data, len, 38);
  delay(40);
  irrecv.enableIRIn();
}

void saveCodesToFlash() {
  preferences.begin("nexus_ir", false);
  preferences.putInt("count", savedCount);
  for (int i = 0; i < savedCount; i++) {
    char k[16];
    snprintf(k, 16, "name_%d", i); preferences.putString(k, savedCodes[i].name);
    snprintf(k, 16, "len_%d", i); preferences.putUShort(k, savedCodes[i].rawLen);
    snprintf(k, 16, "data_%d", i); preferences.putBytes(k, savedCodes[i].rawData, savedCodes[i].rawLen * 2);
    snprintf(k, 16, "value_%d", i); preferences.putULong64(k, savedCodes[i].value);
    snprintf(k, 16, "proto_%d", i); preferences.putUShort(k, savedCodes[i].protocol);
  }
  preferences.end();
}
void loadCodesFromFlash() {
  preferences.begin("nexus_ir", true);
  savedCount = preferences.getInt("count", 0);
  if (savedCount < 0) savedCount = 0;
  if (savedCount > 10) savedCount = 10;
  for (int i = 0; i < savedCount; i++) {
    char k[16];
    snprintf(k, 16, "name_%d", i); String n = preferences.getString(k, "Unknown"); strncpy(savedCodes[i].name, n.c_str(), 15);
    snprintf(k, 16, "len_%d", i); savedCodes[i].rawLen = preferences.getUShort(k, 0);
    if (savedCodes[i].rawLen > IR_RAW_MAX) savedCodes[i].rawLen = 0;
    snprintf(k, 16, "data_%d", i); preferences.getBytes(k, savedCodes[i].rawData, savedCodes[i].rawLen * 2);
    snprintf(k, 16, "value_%d", i); savedCodes[i].value = preferences.getULong64(k, 0);
    snprintf(k, 16, "proto_%d", i); savedCodes[i].protocol = preferences.getUShort(k, 0);
  }
  preferences.end();
}
void deleteSavedCode(int idx) {
  if (idx < 0 || idx >= savedCount) return;
  for (int i = idx; i < savedCount - 1; i++) {
    strcpy(savedCodes[i].name, savedCodes[i+1].name);
    memcpy(savedCodes[i].rawData, savedCodes[i+1].rawData, savedCodes[i+1].rawLen * 2);
    savedCodes[i].rawLen = savedCodes[i+1].rawLen;
    savedCodes[i].value = savedCodes[i+1].value;
    savedCodes[i].protocol = savedCodes[i+1].protocol;
  }
  savedCount--;
  saveCodesToFlash();
}

void startIRRecord() {
  irState = 1; irLastAction = millis(); currentRawLen = 0;
  currentDecodedValue = 0; currentDecodedProtocol = 0; irCodePrinted = false;
  irrecv.enableIRIn();
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("IR RECORDER"); display.setCursor(0, 20); display.print("Press remote..."); display.display();
  sfxClick();
}
void stopIRRecord() {
  if (currentRawLen > 0) {
    if (savedCount < 10) {
      if (nextIRName[0]) {
        strncpy(savedCodes[savedCount].name, nextIRName, 15);
        savedCodes[savedCount].name[15] = '\0';
        nextIRName[0] = '\0';
      } else snprintf(savedCodes[savedCount].name, 16, "Code %d", savedCount + 1);
      memcpy(savedCodes[savedCount].rawData, currentRaw, currentRawLen * 2);
      savedCodes[savedCount].rawLen = currentRawLen;
      savedCodes[savedCount].value = currentDecodedValue;
      savedCodes[savedCount].protocol = currentDecodedProtocol;
      savedCount++; saveCodesToFlash(); petAward(15, 10); irState = 2; sfxEat();
    } else irState = 3;
  } else irState = 0;
  irLastAction = millis();
}

void drawIRRecorder() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  if (irState == 0) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
    display.print("IR RECORDER"); display.setCursor(0, 20); display.print("SEL=Start");
    display.setCursor(0, 40); display.print("Saved: "); display.print(savedCount);
    display.setCursor(0, 56); display.print("Max: 10"); display.display();
    if (btnPressed(B_SEL)) startIRRecord();
  } else if (irState == 1) {
    if (irrecv.decode(&results)) {
      if (results.rawlen > 1) {
        uint16_t count = results.rawlen - 1;
        if (count > IR_RAW_MAX) count = IR_RAW_MAX;
        currentRawLen = count;
        for (uint16_t i = 0; i < count; i++) {
          uint32_t usec = (uint32_t)results.rawbuf[i + 1] * kRawTick;
          currentRaw[i] = (uint16_t)min((uint32_t)65535, usec);
        }
        currentDecodedValue = results.value;
        currentDecodedProtocol = (uint16_t)results.decode_type;
        stopIRRecord();
      }
      irrecv.resume();
    }
    if (btnPressed(B_SEL) || (millis() - irLastAction > 10000)) stopIRRecord();
  } else if (irState == 2) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("IR CODE SAVED");
    display.setCursor(0, 16); display.print("HEX:");
    if (currentDecodedValue) {
      String hex = uint64ToString(currentDecodedValue, 16);
      display.print("0x"); display.print(hex);
      if (!irCodePrinted) { Serial.print("IR decoded code: 0x"); Serial.println(hex); irCodePrinted = true; }
    } else {
      display.print("RAW ");
      display.print(currentRawLen);
      display.print(" pulses");
      display.setCursor(0, 27);
      uint16_t shownA = min((uint16_t)7, currentRawLen);
      for (uint16_t i = 0; i < shownA; i++) {
        display.print(currentRaw[i]);
        if (i + 1 < shownA) display.print(",");
      }
      display.setCursor(0, 39);
      uint16_t endB = min((uint16_t)14, currentRawLen);
      for (uint16_t i = 7; i < endB; i++) {
        display.print(currentRaw[i]);
        if (i + 1 < endB) display.print(",");
      }
      if (!irCodePrinted) {
        Serial.print("IR RAW us (");
        Serial.print(currentRawLen);
        Serial.print("): ");
        for (uint16_t i = 0; i < currentRawLen; i++) {
          Serial.print(currentRaw[i]);
          if (i + 1 < currentRawLen) Serial.print(',');
        }
        Serial.println();
        irCodePrinted = true;
      }
    }
    if (currentDecodedValue) {
      display.setCursor(0, 36); display.print("Protocol: "); display.print(typeToString((decode_type_t)currentDecodedProtocol));
      display.setCursor(0, 52); display.print("Saved in Flash");
    } else {
      display.setCursor(0, 54); display.print("Full RAW -> Serial");
    }
    display.display();
    if (millis() - irLastAction > 1500) irState = 0;
  } else if (irState == 3) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 20); display.print("Memory Full!");
    display.display(); if (millis() - irLastAction > 1500) irState = 0;
  }
}

const char* guessProtocol(uint16_t len) {
  if (len > 60) return "NEC/RAW?";
  if (len > 40) return "Samsung?";
  if (len > 20) return "RC5/RC6?";
  return "short/raw";
}

void drawIRSaved() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static int idx = 0;
  if (savedCount == 0) { display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 20); display.print("No codes saved"); display.display(); return; }
  if (btnPressed(B_LEFT)) { idx--; if (idx < 0) idx = savedCount - 1; sfxClick(); }
  if (btnPressed(B_RIGHT)) { idx++; if (idx >= savedCount) idx = 0; sfxClick(); }
  if (btnPressed(B_SEL)) { irSendRaw(savedCodes[idx].rawData, savedCodes[idx].rawLen); petAward(3, 4); sfxShoot(); return; }
  if (btnLongPress(B_DOWN, 1000)) {
    deleteSavedCode(idx);
    if (idx >= savedCount && savedCount > 0) idx = savedCount - 1;
    if (savedCount == 0) idx = 0;
    sfxDeath();
    return;
  }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("IR SAVED "); display.print(idx + 1); display.print("/"); display.print(savedCount);
  display.setCursor(0, 20); display.print(savedCodes[idx].name);
  display.setCursor(0, 34); display.print("HEX: ");
  if (savedCodes[idx].value) { display.print("0x"); display.print(uint64ToString(savedCodes[idx].value, 16)); }
  else display.print("RAW signal");
  display.setCursor(0, 44); display.print("P:"); display.print(typeToString((decode_type_t)savedCodes[idx].protocol));
  display.setCursor(0, 56); display.print("SEL=Send DN hold=DEL");
  display.display();
}

// ================= TV REMOTE PROFILES =================
const char* tvBrands[] = {"STARWIND", "SAMSUNG", "LG", "SONY", "PHILIPS", "TCL", "HAIER", "HISENSE", "DEXP"};
const uint8_t TV_BRAND_COUNT = sizeof(tvBrands) / sizeof(tvBrands[0]);
const char* tvLearnKeys[] = {"POWER", "VOL+", "VOL-", "MUTE", "INPUT", "CH+", "CH-", "MENU", "OK"};
const uint8_t TV_KEY_COUNT = sizeof(tvLearnKeys) / sizeof(tvLearnKeys[0]);
int tvLearnIndex = 0;
int tvBrandIndex = 0;

void drawTvCodes() {
  buttonsScan();
  if (wantExit()) { inMenu = true; delay(150); return; }
  if (btnPressed(B_LEFT)) tvBrandIndex = (tvBrandIndex + TV_BRAND_COUNT - 1) % TV_BRAND_COUNT;
  if (btnPressed(B_RIGHT)) tvBrandIndex = (tvBrandIndex + 1) % TV_BRAND_COUNT;
  if (btnPressed(B_UP)) tvLearnIndex = (tvLearnIndex + TV_KEY_COUNT - 1) % TV_KEY_COUNT;
  if (btnPressed(B_DOWN)) tvLearnIndex = (tvLearnIndex + 1) % TV_KEY_COUNT;
  if (btnPressed(B_SEL)) {
    char commandName[16];
    snprintf(commandName, sizeof(commandName), "%.7s %.8s", tvBrands[tvBrandIndex], tvLearnKeys[tvLearnIndex]);
    for (int i = 0; i < savedCount; i++) {
      if (strcmp(savedCodes[i].name, commandName) == 0 && savedCodes[i].rawLen > 0) {
        irSendRaw(savedCodes[i].rawData, savedCodes[i].rawLen);
        return;
      }
    }
    strncpy(nextIRName, commandName, sizeof(nextIRName) - 1);
    nextIRName[sizeof(nextIRName) - 1] = '\0';
    startIRRecord();
    currentMode = 0;
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("TV REMOTE");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 17); display.print("BRAND < ");
  display.print(tvBrands[tvBrandIndex]);
  display.print(" >");
  display.setCursor(0, 32); display.print("BUTTON");
  display.setCursor(0, 43); display.print("> "); display.print(tvLearnKeys[tvLearnIndex]);
  display.drawLine(0, 55, 127, 55, SSD1306_WHITE);
  display.setCursor(0, 57); display.print("SEL send / learn");
  display.display();
}

void drawSystemInfo() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("SYSTEM INFORMATION");
  display.setCursor(0, 12); display.print("OS: NEXUS OS");
  display.setCursor(0, 22); display.print("CPU: ESP32-S3");
  display.setCursor(0, 32); display.print("FREQ: "); display.print(getCpuFrequencyMhz()); display.print(" MHz");
  display.setCursor(0, 42); display.print("HEAP: "); display.print(ESP.getFreeHeap() / 1024); display.print(" KB");
  display.setCursor(0, 52); display.print("IR Saved: "); display.print(savedCount);
  display.display(); delay(300);
}

// ================= WIFI SERVER =================
String getConnectedClientsHtml() {
  wifi_sta_list_t staList;
  memset(&staList, 0, sizeof(staList));
  esp_err_t err = esp_wifi_ap_get_sta_list(&staList);
  if (err != ESP_OK) return "<p>Could not read client list.</p>";
  String out = "<p><b>Connected clients:</b> " + String(staList.num) + "</p><ul>";
  for (int i = 0; i < staList.num; i++) {
    char mac[18];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
      staList.sta[i].mac[0], staList.sta[i].mac[1], staList.sta[i].mac[2],
      staList.sta[i].mac[3], staList.sta[i].mac[4], staList.sta[i].mac[5]);
    out += "<li>" + String(mac) + "</li>";
  }
  out += "</ul>";
  if (staList.num == 0) out += "<p>No devices connected right now.</p>";
  return out;
}

void handleWifiRoot() {
  String html = "<!doctype html><html><head><meta charset='utf-8'><meta http-equiv='refresh' content='3'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'><title>NEXUS OS</title>"
    "<style>body{background:#111;color:#ddd;font-family:monospace;padding:20px;max-width:700px;margin:auto}"
    "h1{color:#0f0}li{padding:6px 0;color:#8f8}</style></head><body><h1>NEXUS OS</h1><hr>";
  html += "<p><b>Access point:</b> NEXUS_OS</p><p><b>IP:</b> " + WiFi.softAPIP().toString() + "</p>";
  html += getConnectedClientsHtml();
  html += "<p><b>Saved IR:</b> " + String(savedCount) + "</p><p><b>Free Heap:</b> " + String(ESP.getFreeHeap() / 1024) + " KB</p>";
  html += "<small>Refreshes every 3 seconds. Only clients of this ESP32 access point are listed.</small></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

void drawWifiServer() {
  buttonsScan();
  if (wantExit()) {
    if (wifiServerActive) { server.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); wifiServerActive = false; }
    inMenu = true; delay(150); return;
  }
  if (!wifiServerActive && btnPressed(B_SEL)) {
    sfxEnter();
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 20); display.print("Starting AP..."); display.display();
    WiFi.mode(WIFI_AP); WiFi.softAP("NEXUS_OS", "12345678");
    server.on("/", handleWifiRoot);
    server.begin(); wifiServerActive = true;
  }
  if (wifiServerActive) {
    server.handleClient();
    wifi_sta_list_t staList; memset(&staList, 0, sizeof(staList));
    int clients = (esp_wifi_ap_get_sta_list(&staList) == ESP_OK) ? staList.num : 0;
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("WIFI: ACTIVE");
    display.setCursor(0, 12); display.print("SSID: NEXUS_OS");
    display.setCursor(0, 22); display.print("PASS: 12345678");
    display.setCursor(0, 33); display.print("IP: "); display.print(WiFi.softAPIP());
    display.setCursor(0, 44); display.print("CLIENTS: "); display.print(clients);
    if (clients > 0) {
      display.setCursor(0, 54); display.print("Last: ");
      int n = clients - 1;
      char shortMac[9];
      snprintf(shortMac, sizeof(shortMac), "%02X:%02X:%02X", staList.sta[n].mac[3], staList.sta[n].mac[4], staList.sta[n].mac[5]);
      display.print(shortMac);
    } else { display.setCursor(0, 56); display.print("No clients connected"); }
  } else {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("WIFI SERVER");
    display.setCursor(0, 20); display.print("SEL=Start AP"); display.setCursor(0, 40); display.print("HOLD SEL=Exit");
  }
  display.display(); delay(30);
}

// ================= BADUSB LAB =================
void drawUsbLab() {
  buttonsScan();
  if (wantExit()) { inMenu = true; delay(150); return; }
  display.clearDisplay();
  display.setTextSize(1); display.setCursor(0, 0); display.print("BADUSB LAB");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 16); display.print("SAFE MODE");
  display.setCursor(0, 28); display.print("No auto-typing or");
  display.setCursor(0, 38); display.print("command injection.");
  display.setCursor(0, 50); display.print("HOLD SEL = EXIT");
  display.display();
}

// ================= BLE HID =================
bool hidStarted = false;
uint32_t hidUiTick = 0;

void drawBleHid() {
  buttonsScan();

  if (!hidStarted) {
    Serial.println("[BLE HID] Starting keyboard service...");
    Serial.printf("[BLE HID] Free heap before begin: %u\\n", ESP.getFreeHeap());
    if (WiFi.getMode() != WIFI_OFF) { WiFi.disconnect(true, true); WiFi.mode(WIFI_OFF); delay(250); }
    delay(150);
    Keyboard.begin();
    hidStarted = true;
    Serial.println("[BLE HID] Keyboard service started.");
    Serial.printf("[BLE HID] Free heap after begin: %u\\n", ESP.getFreeHeap());
    Serial.println("[BLE HID] Waiting for host connection...");
    delay(500);
  }

  if (btnLongPress(B_SEL, 2000)) {
    Keyboard.releaseAll();
    inMenu = true;
    delay(150);
    return;
  }

  if (Keyboard.isConnected()) {
    if (btnPressed(B_UP))    Keyboard.write(KEY_UP_ARROW);
    if (btnPressed(B_DOWN))  Keyboard.write(KEY_DOWN_ARROW);
    if (btnPressed(B_LEFT))  Keyboard.write(KEY_LEFT_ARROW);
    if (btnPressed(B_RIGHT)) Keyboard.write(KEY_RIGHT_ARROW);
    if (btnPrev[B_SEL] && !btnCur[B_SEL] && millis() - btnPressStart[B_SEL] < 1900) Keyboard.write(KEY_RETURN);
    if (btnLongPress(B_LEFT, 850)) Keyboard.write(KEY_ESC);
  }

  if (millis() - hidUiTick > 120) {
    hidUiTick = millis();
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0); display.print("BLE HID - NEXUS");
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
    display.setCursor(0, 16);
    display.print(Keyboard.isConnected() ? "PHONE: CONNECTED" : "PHONE: NOT PAIRED");
    display.setCursor(0, 29); display.print("UP/DN/L/R = arrows");
    display.setCursor(0, 40); display.print("TAP SEL = ENTER");
    display.setCursor(0, 51); display.print("HOLD L=ESC  SEL=EXIT");
    display.display();
  }
  delay(8);
}

// ================= BLE SCAN =================
BLEScan* bleScan = nullptr;
bool bleInited = false;
uint32_t bleLastUpdate = 0;
int bleDeviceCount = 0;
bool bleScanDone = false;

void drawBleScan() {
  buttonsScan();
  if (wantExit()) {
    if (bleScan) { bleScan->clearResults(); }
    if (!hidStarted) BLEDevice::deinit(true);
    bleInited = false; bleScanDone = false;
    inMenu = true; delay(150); return;
  }
  if (!bleInited) {
    sfxEnter();
    if (WiFi.getMode() != WIFI_OFF) { WiFi.disconnect(true, true); WiFi.mode(WIFI_OFF); delay(200); }
    if (hidStarted) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0); display.print("BLE SCAN UNAVAILABLE");
      display.setCursor(0, 18); display.print("HID already owns BLE");
      display.setCursor(0, 42); display.print("Restart to scan");
      display.display();
      delay(900);
      inMenu = true;
      return;
    }
    BLEDevice::init("Nexus");
    bleScan = BLEDevice::getScan(); bleScan->setActiveScan(true);
    bleInited = true; bleLastUpdate = 0; bleScanDone = false;
  }
  if (bleScanDone) {
    if (millis() - bleLastUpdate > 2000) { bleScanDone = false; bleLastUpdate = millis(); } else { delay(20); return; }
  }
  if (!bleScanDone) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("BLE SCAN..."); display.setCursor(0, 20); display.print("Wait 1s"); display.display();
    BLEScanResults* res = bleScan->start(1, false); bleDeviceCount = res->getCount();
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("BLE DEV: "); display.print(bleDeviceCount);
    int show = bleDeviceCount > 4 ? 4 : bleDeviceCount;
    for (int i = 0; i < show; i++) {
      BLEAdvertisedDevice d = res->getDevice(i);
      display.setCursor(0, 12 + i * 10);
      String name = d.haveName() ? d.getName().c_str() : "(no name)";
      if (name.length() > 12) name = name.substring(0, 12);
      display.print(name);
      display.setCursor(80, 12 + i * 10); display.print(d.getRSSI()); display.print("dB");
    }
    display.setCursor(0, 54); display.print("HOLD SEL=exit"); display.display();
    bleScan->clearResults(); bleScanDone = true; bleLastUpdate = millis();
  }
  delay(20);
}

// ================= ИГРЫ =================
float cubeAngleX = 0, cubeAngleY = 0, cubeAngleZ = 0;
float cubeVerts[8][3] = {{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}};
const int cubeEdges[12][2] = {{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
void rotatePoint(float &x, float &y, float &z, float ax, float ay, float az) {
  float y1 = y * cos(ax) - z * sin(ax); float z1 = y * sin(ax) + z * cos(ax); y = y1; z = z1;
  float x1 = x * cos(ay) + z * sin(ay); float z2 = -x * sin(ay) + z * cos(ay); x = x1; z = z2;
  float x2 = x * cos(az) - y * sin(az); float y2 = x * sin(az) + y * cos(az); x = x2; y = y2;
}
void draw3DCube() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("3D CUBE");
  const float s = 14.0, d = 4.0; int px[8], py[8];
  for (int i = 0; i < 8; i++) {
    float x = cubeVerts[i][0], y = cubeVerts[i][1], z = cubeVerts[i][2];
    rotatePoint(x, y, z, cubeAngleX, cubeAngleY, cubeAngleZ);
    float f = d / (d + z);
    px[i] = 64 + (int)(x * s * f); py[i] = 36 + (int)(y * s * f);
  }
  for (int i = 0; i < 12; i++) display.drawLine(px[cubeEdges[i][0]], py[cubeEdges[i][0]], px[cubeEdges[i][1]], py[cubeEdges[i][1]], SSD1306_WHITE);
  for (int i = 0; i < 8; i++) display.drawPixel(px[i], py[i], SSD1306_WHITE);
  display.display();
  cubeAngleX += 0.03; cubeAngleY += 0.05; cubeAngleZ += 0.02; delay(20);
}

#define SNK_W 16
#define SNK_H 6
int snkX[64], snkY[64], snkLen = 3, snkDir = 1, snkFoodX = 5, snkFoodY = 3, snkScore = 0;
bool snkDead = false; unsigned long snkLastMove = 0;
void snkReset() {
  snkLen = 3; snkDir = 1; snkScore = 0; snkDead = false;
  snkX[0] = 4; snkY[0] = 3; snkX[1] = 3; snkY[1] = 3; snkX[2] = 2; snkY[2] = 3;
  snkFoodX = 10; snkFoodY = 3; snkLastMove = millis();
}
void drawSnake() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { snkReset(); inited = true; }
  if (snkDead) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("SNAKE OVER");
    display.setCursor(0, 20); display.print("Score: "); display.print(snkScore);
    display.setCursor(0, 40); display.print("SEL=restart"); display.display();
    if (btnPressed(B_SEL)) snkReset();
    delay(20); return;
  }
  if (btnPressed(B_LEFT) && snkDir != 1) snkDir = 3;
  if (btnPressed(B_RIGHT) && snkDir != 3) snkDir = 1;
  if (btnPressed(B_UP) && snkDir != 2) snkDir = 0;
  if (btnPressed(B_DOWN) && snkDir != 0) snkDir = 2;
  if (millis() - snkLastMove > 150) {
    snkLastMove = millis();
    for (int i = snkLen - 1; i > 0; i--) { snkX[i] = snkX[i-1]; snkY[i] = snkY[i-1]; }
    if (snkDir == 0) snkY[0]--; if (snkDir == 1) snkX[0]++; if (snkDir == 2) snkY[0]++; if (snkDir == 3) snkX[0]--;
    if (snkX[0] < 0 || snkX[0] >= SNK_W || snkY[0] < 0 || snkY[0] >= SNK_H) { snkDead = true; sfxDeath(); return; }
    if (snkX[0] == snkFoodX && snkY[0] == snkFoodY) {
      snkScore += 10; if (snkLen < 64) snkLen++;
      snkFoodX = esp_random() % SNK_W; snkFoodY = esp_random() % SNK_H;
      sfxEat();
    }
    for (int i = 1; i < snkLen; i++) if (snkX[0] == snkX[i] && snkY[0] == snkY[i]) { snkDead = true; sfxDeath(); return; }
  }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("SNAKE: "); display.print(snkScore);
  int cell = 8, offY = 12;
  display.fillRect(snkFoodX * cell + 1, snkFoodY * cell + offY + 1, cell - 2, cell - 2, SSD1306_WHITE);
  for (int i = 0; i < snkLen; i++) display.drawRect(snkX[i] * cell, snkY[i] * cell + offY, cell, cell, SSD1306_WHITE);
  display.display(); delay(10);
}

#define TETRIS_W 10
#define TETRIS_H 16
uint16_t tetrisField[TETRIS_H]; int tetrisPiece = 0, tetrisRot = 0, tetrisX = 3, tetrisY = 0, tetrisScore = 0;
unsigned long lastTetrisDrop = 0; bool tetrisGameOver = false;
const int8_t tetrisShapes[7][4][4][2] = {
  {{{0,1},{1,1},{2,1},{3,1}}, {{2,0},{2,1},{2,2},{2,3}}, {{0,2},{1,2},{2,2},{3,2}}, {{1,0},{1,1},{1,2},{1,3}}},
  {{{1,0},{2,0},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{2,1}}},
  {{{1,0},{0,1},{1,1},{2,1}}, {{1,0},{1,1},{2,1},{1,2}}, {{0,1},{1,1},{2,1},{1,2}}, {{1,0},{0,1},{1,1},{1,2}}},
  {{{1,0},{2,0},{0,1},{1,1}}, {{1,0},{1,1},{2,1},{2,2}}, {{1,1},{2,1},{0,2},{1,2}}, {{0,0},{0,1},{1,1},{1,2}}},
  {{{0,0},{1,0},{1,1},{2,1}}, {{2,0},{1,1},{2,1},{1,2}}, {{0,1},{1,1},{1,2},{2,2}}, {{1,0},{0,1},{1,1},{0,2}}},
  {{{2,0},{0,1},{1,1},{2,1}}, {{1,0},{1,1},{1,2},{2,2}}, {{0,1},{1,1},{2,1},{0,2}}, {{0,0},{1,0},{1,1},{1,2}}},
  {{{0,0},{0,1},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{1,2}}, {{0,1},{1,1},{2,1},{2,2}}, {{1,0},{1,1},{0,2},{1,2}}}
};
bool tetrisCheckCollision(int p, int r, int x, int y) {
  for (int i = 0; i < 4; i++) {
    int cx = x + tetrisShapes[p][r][i][0], cy = y + tetrisShapes[p][r][i][1];
    if (cx < 0 || cx >= TETRIS_W || cy >= TETRIS_H) return true;
    if (cy >= 0 && (tetrisField[cy] & (1 << cx))) return true;
  }
  return false;
}
void tetrisSpawn() {
  tetrisPiece = random(0, 7); tetrisRot = 0; tetrisX = 3; tetrisY = 0;
  if (tetrisCheckCollision(tetrisPiece, tetrisRot, tetrisX, tetrisY)) { tetrisGameOver = true; sfxDeath(); }
}
void tetrisLockPiece() {
  for (int i = 0; i < 4; i++) {
    int x = tetrisX + tetrisShapes[tetrisPiece][tetrisRot][i][0], y = tetrisY + tetrisShapes[tetrisPiece][tetrisRot][i][1];
    if (y >= 0 && y < TETRIS_H && x >= 0 && x < TETRIS_W) tetrisField[y] |= (1 << x);
  }
  for (int y = TETRIS_H - 1; y >= 0; y--) {
    if ((tetrisField[y] & 0x3FF) == 0x3FF) {
      for (int yy = y; yy > 0; yy--) tetrisField[yy] = tetrisField[yy-1];
      tetrisField[0] = 0; y++; tetrisScore += 100;
      sfxLine();
    }
  }
  tetrisSpawn();
}
void tetrisReset() {
  for (int i = 0; i < TETRIS_H; i++) tetrisField[i] = 0;
  tetrisScore = 0; tetrisGameOver = false; tetrisSpawn(); lastTetrisDrop = millis();
}
void drawTetris() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { tetrisReset(); inited = true; }
  if (!tetrisGameOver) {
    if (btnPressed(B_DOWN) && !tetrisCheckCollision(tetrisPiece, tetrisRot, tetrisX, tetrisY + 1)) tetrisY++;
    if (btnPressed(B_UP)) { int nr = (tetrisRot + 1) % 4; if (!tetrisCheckCollision(tetrisPiece, nr, tetrisX, tetrisY)) tetrisRot = nr; sfxClick(); }
    if (btnPressed(B_LEFT) && !tetrisCheckCollision(tetrisPiece, tetrisRot, tetrisX - 1, tetrisY)) tetrisX--;
    if (btnPressed(B_RIGHT) && !tetrisCheckCollision(tetrisPiece, tetrisRot, tetrisX + 1, tetrisY)) tetrisX++;
    if (btnPressed(B_SEL)) { while (!tetrisCheckCollision(tetrisPiece, tetrisRot, tetrisX, tetrisY + 1)) tetrisY++; sfxJump(); }
    if (millis() - lastTetrisDrop > 700) {
      if (!tetrisCheckCollision(tetrisPiece, tetrisRot, tetrisX, tetrisY + 1)) tetrisY++; else tetrisLockPiece();
      lastTetrisDrop = millis();
    }
  } else { if (btnPressed(B_SEL)) { tetrisReset(); return; } }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("TETRIS S:"); display.print(tetrisScore);
  int ox = 44, oy = 12, cell = 4;
  display.drawRect(ox - 1, oy - 1, TETRIS_W * cell + 2, (TETRIS_H - 4) * cell + 2, SSD1306_WHITE);
  for (int y = 4; y < TETRIS_H; y++) for (int x = 0; x < TETRIS_W; x++)
    if (tetrisField[y] & (1 << x)) display.fillRect(ox + x * cell, oy + (y - 4) * cell, cell - 1, cell - 1, SSD1306_WHITE);
  if (!tetrisGameOver) {
    for (int i = 0; i < 4; i++) {
      int x = tetrisX + tetrisShapes[tetrisPiece][tetrisRot][i][0], y = tetrisY + tetrisShapes[tetrisPiece][tetrisRot][i][1];
      if (y >= 4 && y < TETRIS_H) display.fillRect(ox + x * cell, oy + (y - 4) * cell, cell - 1, cell - 1, SSD1306_WHITE);
    }
  } else {
    display.fillRect(14, 25, 100, 14, SSD1306_BLACK); display.drawRect(14, 25, 100, 14, SSD1306_WHITE);
    display.setCursor(18, 29); display.print("OVER SEL=restart");
  }
  display.display(); delay(20);
}

float pongBallX = 64, pongBallY = 32, pongBallVX = 1.8, pongBallVY = 1.2;
int pongPaddleY = 26, pongScore = 0, pongLives = 3; bool pongGameOver = false; unsigned long lastPongMove = 0;
void pongReset() { pongBallX = 64; pongBallY = 32; pongBallVX = 1.8; pongBallVY = 1.2; pongPaddleY = 26; pongScore = 0; pongLives = 3; pongGameOver = false; lastPongMove = millis(); }
void drawPong() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { pongReset(); inited = true; }
  if (!pongGameOver) {
    if (btnHeld(B_UP)) pongPaddleY -= 3;
    if (btnHeld(B_DOWN)) pongPaddleY += 3;
    if (pongPaddleY < 12) pongPaddleY = 12;
    if (pongPaddleY > 62 - 12) pongPaddleY = 62 - 12;
    if (millis() - lastPongMove > 25) {
      lastPongMove = millis(); pongBallX += pongBallVX; pongBallY += pongBallVY;
      if (pongBallY <= 12) { pongBallY = 12; pongBallVY = -pongBallVY; sfxClick(); }
      if (pongBallY >= 62) { pongBallY = 62; pongBallVY = -pongBallVY; sfxClick(); }
      if (pongBallX <= 6) {
        if (pongBallY >= pongPaddleY && pongBallY <= pongPaddleY + 12) { pongBallX = 6; pongBallVX = -pongBallVX; pongScore++; sfxHit(); }
        else { pongLives--; if (pongLives <= 0) { pongGameOver = true; sfxDeath(); } else { pongBallX = 64; pongBallY = 32; pongBallVX = 1.8; pongBallVY = 1.2; sfxDeath(); } }
      }
      if (pongBallX >= 126) { pongBallX = 126; pongBallVX = -pongBallVX; sfxClick(); }
    }
  } else { if (btnPressed(B_SEL)) { pongReset(); return; } }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("PONG S:"); display.print(pongScore); display.print(" L:"); display.print(pongLives);
  display.drawRect(0, 10, 128, 54, SSD1306_WHITE);
  display.fillRect(4, pongPaddleY, 2, 12, SSD1306_WHITE);
  display.fillRect((int)pongBallX, (int)pongBallY, 2, 2, SSD1306_WHITE);
  for (int y = 12; y < 62; y += 4) display.drawPixel(127, y, SSD1306_WHITE);
  if (pongGameOver) { display.fillRect(14, 25, 100, 14, SSD1306_BLACK); display.drawRect(14, 25, 100, 14, SSD1306_WHITE); display.setCursor(18, 29); display.print("OVER SEL=restart"); }
  display.display(); delay(15);
}

float flappyY = 32, flappyVY = 0; int flappyPipeX = 128, flappyGapY = 20, flappyScore = 0;
bool flappyDead = false; unsigned long lastFlappyUpdate = 0;
void flappyReset() { flappyY = 32; flappyVY = 0; flappyPipeX = 128; flappyGapY = random(10, 42); flappyScore = 0; flappyDead = false; lastFlappyUpdate = millis(); }
void drawFlappy() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { flappyReset(); inited = true; }
  if (flappyDead && btnPressed(B_SEL)) { flappyReset(); return; }
  if (!flappyDead && btnPressed(B_SEL)) { flappyVY = -2.8; sfxJump(); }
  if (!flappyDead && millis() - lastFlappyUpdate > 30) {
    lastFlappyUpdate = millis(); flappyVY += 0.35; flappyY += flappyVY; flappyPipeX -= 2;
    if (flappyPipeX < -10) { flappyPipeX = 128; flappyGapY = random(10, 42); flappyScore++; sfxEat(); }
    if (flappyY < 10 || flappyY > 60) { flappyDead = true; sfxDeath(); }
    if (flappyPipeX < 20 && flappyPipeX + 8 > 14) if (flappyY < flappyGapY || flappyY + 5 > flappyGapY + 22) { flappyDead = true; sfxDeath(); }
  }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("FLAPPY S:"); display.print(flappyScore);
  if (flappyDead) { display.setCursor(76, 0); display.print("SEL=res"); }
  display.drawRect(0, 9, 128, 55, SSD1306_WHITE);
  if (flappyPipeX > -10 && flappyPipeX < 128) {
    if (flappyGapY > 10) display.fillRect((int)flappyPipeX, 10, 8, flappyGapY - 10, SSD1306_WHITE);
    int bottomStart = flappyGapY + 22; if (bottomStart < 64) display.fillRect((int)flappyPipeX, bottomStart, 8, 64 - bottomStart, SSD1306_WHITE);
  }
  display.fillRect(14, (int)flappyY, 6, 5, SSD1306_WHITE);
  display.display(); delay(15);
}

#define GD_MAX_PLATFORMS 40
struct GDPlatform { int x; int y; int width; int height; bool isWall; bool isPit; };
GDPlatform gd_level[GD_MAX_PLATFORMS]; int gd_platform_count = 0;
void generateGeometryDashLevel() {
  gd_platform_count = 0; gd_level[0].x = 0; gd_level[0].y = 50; gd_level[0].width = 128; gd_level[0].height = 0; gd_level[0].isWall = false; gd_level[0].isPit = false; gd_platform_count++;
  int cx = 128, cy = 50;
  for (int i = 1; i < GD_MAX_PLATFORMS; i++) {
    int gap = random(25, 50), type = random(0, 3);
    if (type == 0) { gd_level[i].x = cx + gap; gd_level[i].y = cy; gd_level[i].width = random(10, 15); gd_level[i].height = random(15, 25); gd_level[i].isWall = true; gd_level[i].isPit = false; cx = gd_level[i].x + gd_level[i].width + random(20, 30); }
    else if (type == 1) { gd_level[i].x = cx + gap; gd_level[i].y = 50; gd_level[i].width = random(20, 35); gd_level[i].height = 0; gd_level[i].isWall = false; gd_level[i].isPit = true; cx = gd_level[i].x + gd_level[i].width + random(20, 30); }
    else { gd_level[i].x = cx + gap; gd_level[i].y = 50; gd_level[i].width = random(30, 50); gd_level[i].height = 0; gd_level[i].isWall = false; gd_level[i].isPit = false; cx = gd_level[i].x + gd_level[i].width; }
  }
}
float gd_player_y = 32, gd_player_vy = 0; bool gd_is_ground = true; int gd_score = 0, gd_game_speed = 3, gd_camera_x = 0;
void gd_reset_level() { gd_player_y = 50; gd_player_vy = 0; gd_is_ground = true; gd_score = 0; gd_game_speed = 3; gd_camera_x = 0; }
void drawGeometryDash() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { generateGeometryDashLevel(); gd_reset_level(); inited = true; }
  static bool game_over = false;
  if (game_over) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("GAME OVER");
    display.setCursor(0, 20); display.print("Score: "); display.print(gd_score);
    display.setCursor(0, 40); display.print("SEL=restart"); display.display();
    if (btnPressed(B_SEL)) { gd_reset_level(); game_over = false; }
    delay(20); return;
  }
  if (btnPressed(B_UP)) { if (gd_is_ground) { gd_player_vy = -4.5; gd_is_ground = false; sfxJump(); } }
  gd_player_vy += 0.7; gd_player_y += gd_player_vy;
  if (gd_player_y >= 50) { gd_player_y = 50; gd_player_vy = 0; gd_is_ground = true; }
  static uint32_t last_move = 0;
  if (millis() - last_move > 25) { last_move = millis(); gd_camera_x += gd_game_speed; gd_score++; }
  bool hit_wall = false;
  for (int i = 0; i < gd_platform_count; i++) {
    GDPlatform p = gd_level[i]; int screen_x = p.x - gd_camera_x;
    if (screen_x + p.width > 0 && screen_x < 128) {
      if (p.isWall) { if (10 + 5 > screen_x && 10 < screen_x + p.width) { if (gd_player_y + 5 > p.y - p.height) hit_wall = true; } }
      else if (!p.isPit) { if (10 + 2 >= screen_x && 10 + 2 <= screen_x + p.width) { if (gd_player_vy >= 0 && gd_player_y + 5 >= p.y && gd_player_y - gd_player_vy <= p.y + 2) { gd_player_y = p.y - 5; gd_player_vy = 0; gd_is_ground = true; } } }
    }
  }
  if (hit_wall) { game_over = true; sfxDeath(); return; }
  if (gd_player_y > 64) { game_over = true; sfxDeath(); return; }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("SCORE: "); display.print(gd_score);
  for (int i = 0; i < gd_platform_count; i++) {
    GDPlatform p = gd_level[i]; int screen_x = p.x - gd_camera_x;
    if (screen_x + p.width > 0 && screen_x < 128) {
      if (p.isPit) {}
      else if (p.isWall) { display.fillRect(screen_x, p.y - p.height, p.width, p.height, SSD1306_WHITE); display.fillRect(screen_x, p.y, p.width, 64 - p.y, SSD1306_WHITE); }
      else { display.fillRect(screen_x, p.y, p.width, 64 - p.y, SSD1306_WHITE); }
    }
  }
  display.fillRect(10, (int)gd_player_y, 5, 5, SSD1306_WHITE); display.display(); delay(20);
}

int siPlayerX = 64, siBulletX = -1, siBulletY = -1, siEnemyX = 64, siEnemyY = 20, siEnemyDir = 1, siScore = 0; bool siDead = false; unsigned long siLastMove = 0;
void siReset() { siPlayerX = 64; siBulletX = -1; siBulletY = -1; siEnemyX = 64; siEnemyY = 20; siEnemyDir = 1; siScore = 0; siDead = false; siLastMove = millis(); }
void drawSpaceInvaders() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { siReset(); inited = true; }
  if (siDead) { display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("GAME OVER"); display.setCursor(0, 20); display.print("Score: "); display.print(siScore); display.setCursor(0, 40); display.print("SEL=restart"); display.display(); if (btnPressed(B_SEL)) { siReset(); } delay(20); return; }
  if (btnHeld(B_LEFT) && siPlayerX > 4) siPlayerX -= 2; if (btnHeld(B_RIGHT) && siPlayerX < 124) siPlayerX += 2;
  if (btnPressed(B_SEL) && siBulletY < 0) { siBulletX = siPlayerX; siBulletY = 58; sfxShoot(); }
  if (siBulletY > 0) { siBulletY -= 4; if (abs(siBulletX - siEnemyX) < 6 && abs(siBulletY - siEnemyY) < 6) { siScore += 10; siBulletY = -1; siEnemyY = 20 + (esp_random() % 20); siEnemyX = 10 + (esp_random() % 100); sfxHit(); } }
  if (millis() - siLastMove > 50) { siLastMove = millis(); siEnemyX += siEnemyDir * 2; if (siEnemyX > 120) siEnemyDir = -1; if (siEnemyX < 8) siEnemyDir = 1; siEnemyY += 1; if (siEnemyY > 52) { siDead = true; sfxDeath(); return; } }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("SI S:"); display.print(siScore);
  display.fillRect(siPlayerX - 3, 60, 7, 3, SSD1306_WHITE); if (siBulletY > 0) display.fillRect(siBulletX, siBulletY, 1, 3, SSD1306_WHITE); display.fillRect(siEnemyX - 3, siEnemyY, 6, 4, SSD1306_WHITE); display.display(); delay(20);
}

float stBlockX = 64, stBlockW = 40, stBaseW = 40; int stScore = 0; bool stMovingRight = true, stDead = false; unsigned long stLastMove = 0;
void stReset() { stBaseW = 40; stBlockX = 64; stBlockW = stBaseW; stScore = 0; stMovingRight = true; stDead = false; stLastMove = millis(); }
void drawStackTower() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { stReset(); inited = true; }
  if (stDead) { display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("GAME OVER"); display.setCursor(0, 20); display.print("Blocks: "); display.print(stScore); display.setCursor(0, 40); display.print("SEL=restart"); display.display(); if (btnPressed(B_SEL)) { stReset(); } delay(20); return; }
  if (millis() - stLastMove > 15) { stLastMove = millis(); if (stMovingRight) { stBlockX += 2; if (stBlockX > 128) { stBlockX = 128; stMovingRight = false; } } else { stBlockX -= 2; if (stBlockX < 0) { stBlockX = 0; stMovingRight = true; } } }
  if (btnPressed(B_SEL)) {
    float towerLeft = 64 - stBaseW / 2, towerRight = 64 + stBaseW / 2; float blockLeft = stBlockX - stBlockW / 2, blockRight = stBlockX + stBlockW / 2;
    if (blockRight < towerLeft || blockLeft > towerRight) { stDead = true; sfxDeath(); return; }
    float newLeft = max(towerLeft, blockLeft), newRight = min(towerRight, blockRight); stBaseW = newRight - newLeft;
    if (stBaseW < 5) { stDead = true; sfxDeath(); return; }
    stScore++; stBlockX = 0; stMovingRight = true; stBlockW = stBaseW; sfxHit();
  }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("TOWER: "); display.print(stScore);
  int y = 60; float currentW = stBaseW;
  for (int i = 0; i < stScore && y > 10; i++) { display.fillRect(64 - currentW / 2, y, currentW, 4, SSD1306_WHITE); y -= 5; }
  display.fillRect(stBlockX - stBlockW / 2, y, stBlockW, 4, SSD1306_WHITE); display.display(); delay(20);
}

// ================= PULSE DODGE =================
float pulsePlayerX = 64.0f;
struct PulseDot { float x, y, speed; bool active; };
PulseDot pulseDots[6];
uint32_t pulseLastTick = 0;
uint32_t pulseLastSpawn = 0;
uint32_t pulseScore = 0;
bool pulseGameOver = false;
bool pulseGameReady = false;

void pulseReset() {
  pulsePlayerX = 64.0f;
  pulseScore = 0;
  pulseGameOver = false;
  pulseLastTick = millis();
  pulseLastSpawn = millis();
  for (int i = 0; i < 6; ++i) {
    pulseDots[i].x = random(5, 123);
    pulseDots[i].y = random(-64, -5);
    pulseDots[i].speed = 1.0f + (random(0, 15) / 10.0f);
    pulseDots[i].active = true;
  }
  pulseGameReady = true;
}

void drawPulseDodge() {
  buttonsScan();
  if (wantExit()) { inMenu = true; pulseGameReady = false; delay(150); return; }
  if (!pulseGameReady) pulseReset();

  if (pulseGameOver) {
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0, 0); display.print("PULSE DODGE");
    display.setCursor(0, 20); display.print("SCORE: "); display.print(pulseScore);
    display.setCursor(0, 40); display.print("SEL = RETRY");
    display.setCursor(0, 54); display.print("HOLD SEL = MENU");
    display.display();
    if (btnPressed(B_SEL)) pulseReset();
    delay(20); return;
  }

  if (btnHeld(B_LEFT)) pulsePlayerX -= 2.2f;
  if (btnHeld(B_RIGHT)) pulsePlayerX += 2.2f;
  pulsePlayerX = constrain(pulsePlayerX, 5.0f, 123.0f);

  uint32_t now = millis();
  if (now - pulseLastSpawn > (uint32_t)max(230, 720 - (int)(pulseScore * 12))) {
    pulseLastSpawn = now;
    for (int i = 0; i < 6; ++i) {
      if (!pulseDots[i].active || pulseDots[i].y > 64) {
        pulseDots[i].x = random(5, 123);
        pulseDots[i].y = -4;
        pulseDots[i].speed = 1.0f + min(2.5f, pulseScore * 0.035f) + random(0, 10) / 10.0f;
        pulseDots[i].active = true;
        break;
      }
    }
  }

  if (now - pulseLastTick > 24) {
    pulseLastTick = now;
    for (int i = 0; i < 6; ++i) {
      if (!pulseDots[i].active) continue;
      pulseDots[i].y += pulseDots[i].speed;
      if (pulseDots[i].y > 64) { pulseDots[i].active = false; pulseScore++; }
      if (pulseDots[i].y > 48 && pulseDots[i].y < 61 && fabsf(pulseDots[i].x - pulsePlayerX) < 7.0f) {
        pulseGameOver = true; break;
      }
    }
  }

  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(0, 0); display.print("PULSE DODGE  "); display.print(pulseScore);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  for (int i = 0; i < 6; ++i) {
    if (pulseDots[i].active) {
      int x = (int)pulseDots[i].x, y = (int)pulseDots[i].y;
      display.drawCircle(x, y, 2, SSD1306_WHITE);
      display.drawPixel(x, y, SSD1306_WHITE);
    }
  }
  display.fillTriangle((int)pulsePlayerX, 58, (int)pulsePlayerX - 5, 63, (int)pulsePlayerX + 5, 63, SSD1306_WHITE);
  display.display(); delay(8);
}

#define SIMON_MAX_LEN 8
int simonSeq[SIMON_MAX_LEN]; int simonLen = 1, simonShowIdx = 0, simonInputIdx = 0; bool simonShowing = false; uint32_t simonShowTime = 0; int simonScore = 0; bool simonDead = false;
void simonReset() { simonLen = 1; simonSeq[0] = esp_random() % 4; simonScore = 0; simonDead = false; simonShowing = false; simonShowIdx = 0; simonInputIdx = 0; }
void drawSimonSays() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { simonReset(); inited = true; }
  if (simonDead) { display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("GAME OVER"); display.setCursor(0, 20); display.print("Score: "); display.print(simonScore); display.setCursor(0, 40); display.print("SEL=restart"); display.display(); if (btnPressed(B_SEL)) { simonReset(); } delay(20); return; }
  if (simonShowing) {
    if (millis() - simonShowTime > 500) { simonShowTime = millis(); simonShowIdx++; if (simonShowIdx > simonLen) { simonShowing = false; simonInputIdx = 0; } }
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("WATCH:");
    display.setCursor(0, 12); display.print("Step: "); display.print(simonShowIdx); display.print("/"); display.print(simonLen);
    int px = 20, py = 20; display.drawCircle(px, py, 8, SSD1306_WHITE); display.setCursor(px - 2, py - 3); display.print("U");
    px = 20; py = 45; display.drawCircle(px, py, 8, SSD1306_WHITE); display.setCursor(px - 2, py - 3); display.print("D");
    px = 108; py = 20; display.drawCircle(px, py, 8, SSD1306_WHITE); display.setCursor(px - 2, py - 3); display.print("L");
    px = 108; py = 45; display.drawCircle(px, py, 8, SSD1306_WHITE); display.setCursor(px - 2, py - 3); display.print("R");
    if (simonShowIdx < simonLen) { int b = simonSeq[simonShowIdx]; if (b == 0) display.fillCircle(20, 20, 6, SSD1306_WHITE); if (b == 1) display.fillCircle(20, 45, 6, SSD1306_WHITE); if (b == 2) display.fillCircle(108, 20, 6, SSD1306_WHITE); if (b == 3) display.fillCircle(108, 45, 6, SSD1306_WHITE); }
    display.display(); delay(30); return;
  }
  int pressed = -1; if (btnPressed(B_UP)) pressed = 0; else if (btnPressed(B_DOWN)) pressed = 1; else if (btnPressed(B_LEFT)) pressed = 2; else if (btnPressed(B_RIGHT)) pressed = 3;
  if (pressed >= 0) { if (pressed == simonSeq[simonInputIdx]) { simonInputIdx++; if (simonInputIdx >= simonLen) { simonScore += 10; simonLen++; if (simonLen > SIMON_MAX_LEN) simonLen = SIMON_MAX_LEN; for (int i = 0; i < simonLen; i++) simonSeq[i] = esp_random() % 4; simonShowing = true; simonShowIdx = 0; simonShowTime = millis(); sfxEat(); } } else { simonDead = true; sfxDeath(); } }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("SIMON S:"); display.print(simonScore);
  display.setCursor(0, 12); display.print("Your turn! Len: "); display.print(simonLen);
  int px = 20, py = 20; display.drawCircle(px, py, 8, SSD1306_WHITE); display.setCursor(px - 2, py - 3); display.print("U");
  px = 20; py = 45; display.drawCircle(px, py, 8, SSD1306_WHITE); display.setCursor(px - 2, py - 3); display.print("D");
  px = 108; py = 20; display.drawCircle(px, py, 8, SSD1306_WHITE); display.setCursor(px - 2, py - 3); display.print("L");
  px = 108; py = 45; display.drawCircle(px, py, 8, SSD1306_WHITE); display.setCursor(px - 2, py - 3); display.print("R");
  display.display(); delay(20);
}

float trexY = 50, trexVY = 0; bool trexGround = true; int trexObsX = 128, trexScore = 0, trexSpeed = 4; bool trexDead = false; unsigned long trexLastMove = 0;
void trexReset() { trexY = 50; trexVY = 0; trexGround = true; trexObsX = 128; trexScore = 0; trexSpeed = 4; trexDead = false; trexLastMove = millis(); }
void drawTRex() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { trexReset(); inited = true; }
  if (trexDead) { display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.print("GAME OVER"); display.setCursor(0, 20); display.print("Score: "); display.print(trexScore); display.setCursor(0, 40); display.print("SEL=restart"); display.display(); if (btnPressed(B_SEL)) { trexReset(); } delay(20); return; }
  if ((btnPressed(B_SEL) || btnPressed(B_UP)) && trexGround) { trexVY = -5.5; trexGround = false; sfxJump(); }
  trexVY += 0.6; trexY += trexVY;
  if (trexY >= 50) { trexY = 50; trexVY = 0; trexGround = true; }
  if (millis() - trexLastMove > 20) { trexLastMove = millis(); trexObsX -= trexSpeed; if (trexObsX < -10) { trexObsX = 128; trexScore += 10; if (trexScore % 50 == 0) trexSpeed++; sfxEat(); } }
  if (trexObsX < 20 && trexObsX > 8) { if (trexY > 40) { trexDead = true; sfxDeath(); return; } }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("TREX S:"); display.print(trexScore);
  display.drawLine(0, 55, 128, 55, SSD1306_WHITE); display.fillRect(10, (int)trexY, 6, 6, SSD1306_WHITE); display.fillRect(trexObsX, 45, 4, 10, SSD1306_WHITE);
  display.display(); delay(20);
}

// ================= BREAKOUT =================
#define BRK_ROWS 3
#define BRK_COLS 8
bool brkBlocks[BRK_ROWS][BRK_COLS];
float brkPaddleX = 54, brkBallX = 64, brkBallY = 50, brkBallVX = 2.0, brkBallVY = -2.0;
int brkScore = 0, brkLives = 3;
bool brkDead = false, brkWin = false;
unsigned long brkLastMove = 0;

void brkReset() {
  for (int r = 0; r < BRK_ROWS; r++) for (int c = 0; c < BRK_COLS; c++) brkBlocks[r][c] = true;
  brkPaddleX = 54; brkBallX = 64; brkBallY = 50; brkBallVX = 2.0; brkBallVY = -2.0;
  brkScore = 0; brkLives = 3; brkDead = false; brkWin = false;
  brkLastMove = millis();
}

void drawBreakout() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { brkReset(); inited = true; }
  if (brkDead || brkWin) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
    display.print(brkWin ? "YOU WIN!" : "GAME OVER");
    display.setCursor(0, 20); display.print("Score: "); display.print(brkScore);
    display.setCursor(0, 40); display.print("SEL=restart"); display.display();
    if (btnPressed(B_SEL)) brkReset();
    delay(20); return;
  }
  if (btnHeld(B_LEFT)) brkPaddleX -= 4;
  if (btnHeld(B_RIGHT)) brkPaddleX += 4;
  if (brkPaddleX < 0) brkPaddleX = 0;
  if (brkPaddleX > 128 - 20) brkPaddleX = 128 - 20;

  if (millis() - brkLastMove > 20) {
    brkLastMove = millis();
    brkBallX += brkBallVX; brkBallY += brkBallVY;
    if (brkBallX < 2) { brkBallX = 2; brkBallVX = -brkBallVX; sfxClick(); }
    if (brkBallX > 125) { brkBallX = 125; brkBallVX = -brkBallVX; sfxClick(); }
    if (brkBallY < 12) { brkBallY = 12; brkBallVY = -brkBallVY; sfxClick(); }
    if (brkBallY > 58 && brkBallY < 62 && brkBallX > brkPaddleX && brkBallX < brkPaddleX + 20) { brkBallY = 58; brkBallVY = -brkBallVY; sfxHit(); }
    int brkOffY = 14;
    for (int r = 0; r < BRK_ROWS; r++) for (int c = 0; c < BRK_COLS; c++) {
      if (!brkBlocks[r][c]) continue;
      int bx = c * 16 + 2, by = r * 6 + brkOffY;
      if (brkBallX > bx && brkBallX < bx + 14 && brkBallY > by && brkBallY < by + 5) { brkBlocks[r][c] = false; brkScore += 10; brkBallVY = -brkBallVY; sfxHit(); }
    }
    bool anyBlock = false;
    for (int r = 0; r < BRK_ROWS; r++) for (int c = 0; c < BRK_COLS; c++) if (brkBlocks[r][c]) anyBlock = true;
    if (!anyBlock) { brkWin = true; sfxWin(); }
    if (brkBallY > 64) { brkLives--; if (brkLives <= 0) { brkDead = true; sfxDeath(); } else { brkBallX = 64; brkBallY = 50; brkBallVX = 2.0; brkBallVY = -2.0; sfxDeath(); } }
  }

  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("BRK S:"); display.print(brkScore); display.print(" L:"); display.print(brkLives);
  int brkOffY = 14;
  for (int r = 0; r < BRK_ROWS; r++) for (int c = 0; c < BRK_COLS; c++) if (brkBlocks[r][c]) display.fillRect(c * 16 + 2, r * 6 + brkOffY, 14, 5, SSD1306_WHITE);
  display.fillRect(brkPaddleX, 60, 20, 3, SSD1306_WHITE);
  display.fillRect((int)brkBallX, (int)brkBallY, 2, 2, SSD1306_WHITE);
  display.display(); delay(15);
}

// ================= 2048 =================
#define G2048_SIZE 4
int g2048Field[G2048_SIZE][G2048_SIZE];
int g2048Score = 0;
bool g2048Dead = false, g2048Win = false;

void g2048Spawn() {
  int empty[16][2], n = 0;
  for (int r = 0; r < G2048_SIZE; r++) for (int c = 0; c < G2048_SIZE; c++) if (g2048Field[r][c] == 0) { empty[n][0] = r; empty[n][1] = c; n++; }
  if (n == 0) return;
  int i = esp_random() % n;
  g2048Field[empty[i][0]][empty[i][1]] = (esp_random() % 10 == 0) ? 4 : 2;
}

void g2048Reset() {
  for (int r = 0; r < G2048_SIZE; r++) for (int c = 0; c < G2048_SIZE; c++) g2048Field[r][c] = 0;
  g2048Score = 0; g2048Dead = false; g2048Win = false;
  g2048Spawn(); g2048Spawn();
}

void g2048MoveRow(int row[G2048_SIZE]) {
  int out[G2048_SIZE] = {0,0,0,0};
  int idx = 0;
  for (int i = 0; i < G2048_SIZE; i++) if (row[i] != 0) out[idx++] = row[i];
  for (int i = 0; i < G2048_SIZE - 1; i++) {
    if (out[i] != 0 && out[i] == out[i+1]) { out[i] *= 2; out[i+1] = 0; g2048Score += out[i]; if (out[i] == 2048) { g2048Win = true; sfxWin(); } }
  }
  int out2[G2048_SIZE] = {0,0,0,0};
  idx = 0;
  for (int i = 0; i < G2048_SIZE; i++) if (out[i] != 0) out2[idx++] = out[i];
  for (int i = 0; i < G2048_SIZE; i++) row[i] = out2[i];
}

bool g2048MoveLeft() {
  bool moved = false;
  for (int r = 0; r < G2048_SIZE; r++) {
    int old[G2048_SIZE]; for (int c = 0; c < G2048_SIZE; c++) old[c] = g2048Field[r][c];
    g2048MoveRow(g2048Field[r]);
    for (int c = 0; c < G2048_SIZE; c++) if (old[c] != g2048Field[r][c]) moved = true;
  }
  return moved;
}
bool g2048MoveRight() {
  bool moved = false;
  for (int r = 0; r < G2048_SIZE; r++) {
    int old[G2048_SIZE]; for (int c = 0; c < G2048_SIZE; c++) old[c] = g2048Field[r][c];
    int rev[G2048_SIZE]; for (int c = 0; c < G2048_SIZE; c++) rev[c] = g2048Field[r][G2048_SIZE-1-c];
    g2048MoveRow(rev);
    for (int c = 0; c < G2048_SIZE; c++) g2048Field[r][G2048_SIZE-1-c] = rev[c];
    for (int c = 0; c < G2048_SIZE; c++) if (old[c] != g2048Field[r][c]) moved = true;
  }
  return moved;
}
bool g2048MoveUp() {
  bool moved = false;
  for (int c = 0; c < G2048_SIZE; c++) {
    int old[G2048_SIZE]; for (int r = 0; r < G2048_SIZE; r++) old[r] = g2048Field[r][c];
    int col[G2048_SIZE]; for (int r = 0; r < G2048_SIZE; r++) col[r] = g2048Field[r][c];
    g2048MoveRow(col);
    for (int r = 0; r < G2048_SIZE; r++) g2048Field[r][c] = col[r];
    for (int r = 0; r < G2048_SIZE; r++) if (old[r] != g2048Field[r][c]) moved = true;
  }
  return moved;
}
bool g2048MoveDown() {
  bool moved = false;
  for (int c = 0; c < G2048_SIZE; c++) {
    int old[G2048_SIZE]; for (int r = 0; r < G2048_SIZE; r++) old[r] = g2048Field[r][c];
    int rev[G2048_SIZE]; for (int r = 0; r < G2048_SIZE; r++) rev[r] = g2048Field[G2048_SIZE-1-r][c];
    g2048MoveRow(rev);
    for (int r = 0; r < G2048_SIZE; r++) g2048Field[G2048_SIZE-1-r][c] = rev[r];
    for (int r = 0; r < G2048_SIZE; r++) if (old[r] != g2048Field[r][c]) moved = true;
  }
  return moved;
}

bool g2048HasMove() {
  for (int r = 0; r < G2048_SIZE; r++) for (int c = 0; c < G2048_SIZE; c++) {
    if (g2048Field[r][c] == 0) return true;
    if (c < G2048_SIZE-1 && g2048Field[r][c] == g2048Field[r][c+1]) return true;
    if (r < G2048_SIZE-1 && g2048Field[r][c] == g2048Field[r+1][c]) return true;
  }
  return false;
}

void draw2048() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { g2048Reset(); inited = true; }
  if (g2048Dead || g2048Win) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
    display.print(g2048Win ? "YOU WIN 2048!" : "GAME OVER");
    display.setCursor(0, 20); display.print("Score: "); display.print(g2048Score);
    display.setCursor(0, 40); display.print("SEL=restart"); display.display();
    if (btnPressed(B_SEL)) g2048Reset();
    delay(20); return;
  }
  bool moved = false;
  if (btnPressed(B_LEFT))  moved = g2048MoveLeft();
  if (btnPressed(B_RIGHT)) moved = g2048MoveRight();
  if (btnPressed(B_UP))    moved = g2048MoveUp();
  if (btnPressed(B_DOWN))  moved = g2048MoveDown();
  if (moved) { sfxClick(); g2048Spawn(); if (!g2048HasMove()) { g2048Dead = true; sfxDeath(); } }
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("2048 S:"); display.print(g2048Score);
  int ox = 4, oy = 14, cell = 28, pad = 2;
  for (int r = 0; r < G2048_SIZE; r++) for (int c = 0; c < G2048_SIZE; c++) {
    int x = ox + c * cell, y = oy + r * cell;
    display.drawRect(x, y, cell - pad, cell - pad, SSD1306_WHITE);
    if (g2048Field[r][c] != 0) {
      char txt[6]; snprintf(txt, 6, "%d", g2048Field[r][c]);
      int16_t tx1, ty1; uint16_t tw, th;
      display.getTextBounds(txt, 0, 0, &tx1, &ty1, &tw, &th);
      int fs = (g2048Field[r][c] >= 1000) ? 1 : (g2048Field[r][c] >= 100 ? 1 : 2);
      display.setTextSize(fs);
      display.getTextBounds(txt, 0, 0, &tx1, &ty1, &tw, &th);
      display.setCursor(x + (cell - pad - tw) / 2, y + (cell - pad - th) / 2);
      display.print(txt);
      display.setTextSize(1);
    }
  }
  display.display(); delay(20);
}

// ================= MAZE 2D =================
#define MAZE_W 10
#define MAZE_H 5
#define MAZE_CELL 8
uint8_t mazeCells[MAZE_H][MAZE_W];
int mazePx = 0, mazePy = 0;
bool mazeWin = false;

void mazeGenerate() {
  for (int r = 0; r < MAZE_H; r++) for (int c = 0; c < MAZE_W; c++) mazeCells[r][c] = 0x0F;
  bool visited[MAZE_H][MAZE_W] = {};
  int stack[MAZE_W * MAZE_H][2]; int sp = 0;
  stack[sp][0] = 0; stack[sp][1] = 0; sp++;
  visited[0][0] = true;
  const int dr[4] = {-1, 0, 1, 0}, dc[4] = {0, 1, 0, -1};
  while (sp > 0) {
    int r = stack[sp-1][0], c = stack[sp-1][1];
    int dirs[4] = {0,1,2,3};
    for (int i = 0; i < 4; i++) { int j = esp_random() % 4; int t = dirs[i]; dirs[i] = dirs[j]; dirs[j] = t; }
    bool moved = false;
    for (int i = 0; i < 4; i++) {
      int d = dirs[i];
      int nr = r + dr[d], nc = c + dc[d];
      if (nr >= 0 && nr < MAZE_H && nc >= 0 && nc < MAZE_W && !visited[nr][nc]) {
        mazeCells[r][c] &= ~(1 << d);
        mazeCells[nr][nc] &= ~(1 << ((d + 2) % 4));
        visited[nr][nc] = true;
        stack[sp][0] = nr; stack[sp][1] = nc; sp++;
        moved = true;
        break;
      }
    }
    if (!moved) sp--;
  }
  mazePx = 0; mazePy = 0; mazeWin = false;
}

void drawMaze() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { mazeGenerate(); inited = true; }
  if (mazeWin) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
    display.print("MAZE SOLVED!");
    display.setCursor(0, 20); display.print("SEL=new maze");
    display.display();
    if (btnPressed(B_SEL)) { mazeGenerate(); sfxEnter(); }
    delay(20); return;
  }
  if (btnPressed(B_UP) && !(mazeCells[mazePy][mazePx] & 1) && mazePy > 0) { mazePy--; sfxClick(); }
  if (btnPressed(B_RIGHT) && !(mazeCells[mazePy][mazePx] & 2) && mazePx < MAZE_W-1) { mazePx++; sfxClick(); }
  if (btnPressed(B_DOWN) && !(mazeCells[mazePy][mazePx] & 4) && mazePy < MAZE_H-1) { mazePy++; sfxClick(); }
  if (btnPressed(B_LEFT) && !(mazeCells[mazePy][mazePx] & 8) && mazePx > 0) { mazePx--; sfxClick(); }
  if (mazePx == MAZE_W-1 && mazePy == MAZE_H-1) { mazeWin = true; sfxWin(); }

  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("MAZE");
  int ox = 4, oy = 14;
  for (int r = 0; r < MAZE_H; r++) for (int c = 0; c < MAZE_W; c++) {
    int x = ox + c * MAZE_CELL, y = oy + r * MAZE_CELL;
    if (mazeCells[r][c] & 1) display.drawLine(x, y, x + MAZE_CELL, y, SSD1306_WHITE);
    if (mazeCells[r][c] & 2) display.drawLine(x + MAZE_CELL, y, x + MAZE_CELL, y + MAZE_CELL, SSD1306_WHITE);
    if (mazeCells[r][c] & 4) display.drawLine(x, y + MAZE_CELL, x + MAZE_CELL, y + MAZE_CELL, SSD1306_WHITE);
    if (mazeCells[r][c] & 8) display.drawLine(x, y, x, y + MAZE_CELL, SSD1306_WHITE);
  }
  display.fillRect(ox + mazePx * MAZE_CELL + 2, oy + mazePy * MAZE_CELL + 2, MAZE_CELL - 4, MAZE_CELL - 4, SSD1306_WHITE);
  display.drawRect(ox + (MAZE_W-1) * MAZE_CELL + 2, oy + (MAZE_H-1) * MAZE_CELL + 2, MAZE_CELL - 4, MAZE_CELL - 4, SSD1306_WHITE);
  display.display(); delay(20);
}

// ================= WHACK-A-MOLE =================
#define WAM_COLS 3
#define WAM_ROWS 2
int wamMole = -1;
int wamScore = 0, wamMisses = 0;
bool wamDead = false;
uint32_t wamLastMove = 0;

void wamReset() { wamMole = -1; wamScore = 0; wamMisses = 0; wamDead = false; wamLastMove = millis(); }

void drawWhackAMole() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { wamReset(); inited = true; }
  if (wamDead) {
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
    display.print("GAME OVER"); display.setCursor(0, 20); display.print("Score: "); display.print(wamScore);
    display.setCursor(0, 40); display.print("SEL=restart"); display.display();
    if (btnPressed(B_SEL)) wamReset();
    delay(20); return;
  }
  if (millis() - wamLastMove > 800) {
    wamLastMove = millis();
    if (wamMole < 0) wamMole = esp_random() % (WAM_COLS * WAM_ROWS);
    else { wamMole = -1; wamMisses++; if (wamMisses >= 5) { wamDead = true; sfxDeath(); } else sfxDeath(); }
  }
  static int wamSel = 0;
  if (btnPressed(B_LEFT))  { wamSel = (wamSel + WAM_COLS * WAM_ROWS - 1) % (WAM_COLS * WAM_ROWS); sfxClick(); }
  if (btnPressed(B_RIGHT)) { wamSel = (wamSel + 1) % (WAM_COLS * WAM_ROWS); sfxClick(); }
  if (btnPressed(B_UP))    { wamSel = (wamSel + WAM_COLS * WAM_ROWS - WAM_COLS) % (WAM_COLS * WAM_ROWS); sfxClick(); }
  if (btnPressed(B_DOWN))  { wamSel = (wamSel + WAM_COLS) % (WAM_COLS * WAM_ROWS); sfxClick(); }
  if (btnPressed(B_SEL)) { if (wamSel == wamMole) { wamScore += 10; wamMole = -1; wamLastMove = millis(); sfxHit(); } else sfxClick(); }

  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.print("WHACK S:"); display.print(wamScore); display.print(" M:"); display.print(wamMisses);
  int ox = 16, oy = 16, cw = 32, ch = 22;
  for (int r = 0; r < WAM_ROWS; r++) for (int c = 0; c < WAM_COLS; c++) {
    int idx = r * WAM_COLS + c;
    int x = ox + c * cw, y = oy + r * ch;
    display.drawRect(x, y, cw - 2, ch - 2, SSD1306_WHITE);
    if (idx == wamMole) display.fillCircle(x + cw/2 - 1, y + ch/2 - 1, 6, SSD1306_WHITE);
    if (idx == wamSel) display.drawRect(x + 1, y + 1, cw - 4, ch - 4, SSD1306_WHITE);
  }
  display.display(); delay(20);
}

// ================= MAZE 3D (RAYCASTER) =================
#define M3D_W 10
#define M3D_H 10
uint8_t m3d_map[M3D_H][M3D_W];
float m3d_px = 1.5, m3d_py = 1.5, m3d_angle = 0.0;
int m3d_level = 1;
int m3d_goal_x = M3D_W - 2, m3d_goal_y = M3D_H - 2;
bool m3d_win = false;

void m3d_generate() {
  for (int r = 0; r < M3D_H; r++) for (int c = 0; c < M3D_W; c++) m3d_map[r][c] = 1;
  int cx = 1, cy = 1; m3d_map[cy][cx] = 0;
  m3d_px = cx + 0.5; m3d_py = cy + 0.5; m3d_angle = 0.0;
  int stack[M3D_W * M3D_H][2]; int sp = 0;
  stack[sp][0] = cx; stack[sp][1] = cy; sp++;
  const int dx[] = {0, 1, 0, -1}; const int dy[] = {-1, 0, 1, 0};
  while (sp > 0) {
    int x = stack[sp - 1][0], y = stack[sp - 1][1];
    int dirs[] = {0, 1, 2, 3};
    for (int i = 0; i < 4; i++) { int r = random(0, 4); int t = dirs[i]; dirs[i] = dirs[r]; dirs[r] = t; }
    bool moved = false;
    for (int i = 0; i < 4; i++) {
      int d = dirs[i]; int nx = x + dx[d] * 2; int ny = y + dy[d] * 2;
      if (nx > 0 && nx < M3D_W - 1 && ny > 0 && ny < M3D_H - 1 && m3d_map[ny][nx] == 1) {
        m3d_map[y + dy[d]][x + dx[d]] = 0; m3d_map[ny][nx] = 0;
        stack[sp][0] = nx; stack[sp][1] = ny; sp++;
        moved = true; break;
      }
    }
    if (!moved) sp--;
  }
  m3d_goal_x = M3D_W - 2; m3d_goal_y = M3D_H - 2;
  m3d_map[m3d_goal_y][m3d_goal_x] = 0; m3d_win = false;
}

void drawMaze3D() {
  buttonsScan(); if (wantExit()) { inMenu = true; delay(150); return; }
  static bool inited = false; if (!inited) { m3d_generate(); inited = true; }
  if (m3d_win) {
    display.clearDisplay(); display.setTextSize(2); display.setCursor(10, 20); display.print("LEVEL CLEAR!");
    display.setTextSize(1); display.setCursor(10, 45); display.print("SEL=Next Level");
    display.display();
    if (btnPressed(B_SEL)) { m3d_level++; m3d_generate(); }
    return;
  }
  float moveSpeed = 0.08; float rotSpeed = 0.08;
  if (btnHeld(B_LEFT))  m3d_angle -= rotSpeed;
  if (btnHeld(B_RIGHT)) m3d_angle += rotSpeed;
  if (btnHeld(B_UP)) { float nx = m3d_px + cos(m3d_angle) * moveSpeed; float ny = m3d_py + sin(m3d_angle) * moveSpeed; if (m3d_map[(int)ny][(int)nx] == 0) { m3d_px = nx; m3d_py = ny; } }
  if (btnHeld(B_DOWN)) { float nx = m3d_px - cos(m3d_angle) * moveSpeed; float ny = m3d_py - sin(m3d_angle) * moveSpeed; if (m3d_map[(int)ny][(int)nx] == 0) { m3d_px = nx; m3d_py = ny; } }
  if (abs(m3d_px - (m3d_goal_x + 0.5)) < 0.5 && abs(m3d_py - (m3d_goal_y + 0.5)) < 0.5) { m3d_win = true; sfxWin(); }
  display.clearDisplay();
  for (int x = 0; x < 128; x++) {
    float rayAngle = (m3d_angle - 0.6) + (x * 1.2 / 128.0);
    float rayX = m3d_px, rayY = m3d_py;
    float stepX = cos(rayAngle) * 0.05; float stepY = sin(rayAngle) * 0.05;
    float dist = 0; bool hitGoal = false;
    while (true) {
      rayX += stepX; rayY += stepY; dist += 0.05;
      int rx = (int)rayX, ry = (int)rayY;
      if (rx < 0 || rx >= M3D_W || ry < 0 || ry >= M3D_H) break;
      if (rx == m3d_goal_x && ry == m3d_goal_y) { if (dist < 5.0) hitGoal = true; break; }
      if (m3d_map[ry][rx] == 1) break;
      if (dist > 10.0) break;
    }
    int wallHeight = (int)(64.0 / dist); if (wallHeight > 64) wallHeight = 64;
    int wallStart = 32 - wallHeight / 2; int wallEnd = 32 + wallHeight / 2;
    if (hitGoal) { if (millis() % 400 < 200) { for (int y = wallStart; y < wallEnd; y += 2) display.drawPixel(x, y, SSD1306_WHITE); } }
    else { display.drawLine(x, wallStart, x, wallEnd, SSD1306_WHITE); }
  }
  int mapSize = 3; int ox = 128 - (M3D_W * mapSize) - 2; int oy = 2;
  for (int r = 0; r < M3D_H; r++) for (int c = 0; c < M3D_W; c++) {
    if (m3d_map[r][c] == 1) display.fillRect(ox + c * mapSize, oy + r * mapSize, mapSize - 1, mapSize - 1, SSD1306_WHITE);
    else if (c == m3d_goal_x && r == m3d_goal_y) display.drawRect(ox + c * mapSize, oy + r * mapSize, mapSize - 1, mapSize - 1, SSD1306_WHITE);
  }
  display.fillRect(ox + (int)m3d_px * mapSize, oy + (int)m3d_py * mapSize, 2, 2, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("LVL: "); display.print(m3d_level);
  display.setCursor(0, 56); display.print("DIST: "); display.print((int)(abs(m3d_px - m3d_goal_x) + abs(m3d_py - m3d_goal_y)));
  display.display(); delay(10);
}

// ================= IR RANDOM (ОДИН ТЕСТ ПО UP) =================
uint32_t irRandSentCount = 0;
uint32_t irRandLastCode = 0;
bool irRandHasSent = false;

void drawIRRandom() {
  buttonsScan();
  if (wantExit()) { inMenu = true; delay(150); return; }

  // Одно нажатие UP = одна случайная NEC-команда.
  // Повторная отправка возможна только после нового нажатия.
  if (btnPressed(B_UP)) {
    irRandLastCode = esp_random();
    irrecv.disableIRIn();
    irsend.sendNEC(irRandLastCode, 32);
    delay(40);
    irrecv.enableIRIn();
    irRandSentCount++;
    irRandHasSent = true;
    sfxShoot();
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("IR RANDOM TEST");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 19); display.print("Sent: "); display.print(irRandSentCount);
  display.setCursor(0, 31); display.print("UP = one test");
  display.setCursor(0, 42); display.print("SEL = free");
  display.setCursor(0, 54);
  if (irRandHasSent) {
    display.print("Last: 0x");
    char codeText[9];
    snprintf(codeText, sizeof(codeText), "%08lX", (unsigned long)irRandLastCode);
    display.print(codeText);
  } else {
    display.print("Press UP to send");
  }
  display.display();
  delay(10);
}

// ================= WIFI SCANNER (SAFE MONITOR) =================
int wifiScanCount = 0;
int wifiScanIndex = 0;
bool wifiScanReady = false;
uint32_t wifiScanLast = 0;

void drawWifiDeauth() {
  buttonsScan();
  if (wantExit()) { WiFi.scanDelete(); WiFi.mode(WIFI_OFF); inMenu = true; delay(150); return; }

  if (btnPressed(B_SEL) || (!wifiScanReady && millis() - wifiScanLast > 500)) {
    wifiScanLast = millis();
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);
    wifiScanCount = WiFi.scanNetworks(false, true);
    wifiScanReady = true;
    wifiScanIndex = 0;
  }

  if (wifiScanCount > 0) {
    if (btnPressed(B_UP)) wifiScanIndex = (wifiScanIndex + wifiScanCount - 1) % wifiScanCount;
    if (btnPressed(B_DOWN)) wifiScanIndex = (wifiScanIndex + 1) % wifiScanCount;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("WIFI SCANNER");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  if (!wifiScanReady) {
    display.setCursor(0, 25); display.print("Scanning...");
  } else if (wifiScanCount <= 0) {
    display.setCursor(0, 23); display.print("No networks found");
    display.setCursor(0, 43); display.print("SEL = rescan");
  } else {
    display.setCursor(0, 14); display.print(wifiScanIndex + 1); display.print("/"); display.print(wifiScanCount);
    display.setCursor(0, 25);
    String ssid = WiFi.SSID(wifiScanIndex);
    if (ssid.length() > 20) ssid = ssid.substring(0, 20);
    display.print(ssid.length() ? ssid : "<hidden>");
    display.setCursor(0, 37); display.print(WiFi.RSSI(wifiScanIndex)); display.print(" dBm CH "); display.print(WiFi.channel(wifiScanIndex));
    display.setCursor(0, 49);
    wifi_auth_mode_t enc = WiFi.encryptionType(wifiScanIndex);
    display.print(enc == WIFI_AUTH_OPEN ? "OPEN" : "SECURED");
  }
  display.setCursor(0, 58); display.print("UP/DN nav SEL scan");
  display.display();
  delay(20);
}

// ================= ИКОНКИ ДЛЯ МЕНЮ =================
void drawModeIcon(int mode, int cx, int cy) {
  switch (mode) {
    case 0: // IR RECORDER
      display.drawCircle(cx, cy, 3, SSD1306_WHITE);
      display.drawCircle(cx, cy, 6, SSD1306_WHITE);
      display.drawCircle(cx, cy, 9, SSD1306_WHITE);
      break;
    case 2: // IR SAVED
      display.drawRect(cx-7, cy-7, 14, 14, SSD1306_WHITE);
      display.fillRect(cx-3, cy-7, 6, 4, SSD1306_WHITE);
      display.drawRect(cx-4, cy, 8, 6, SSD1306_WHITE);
      break;
    case 3: // SYSTEM INFO
      display.drawRect(cx-6, cy-6, 12, 12, SSD1306_WHITE);
      display.drawRect(cx-3, cy-3, 6, 6, SSD1306_WHITE);
      for (int i=-6;i<=6;i+=3){display.drawPixel(cx+i, cy-8, SSD1306_WHITE); display.drawPixel(cx+i, cy+8, SSD1306_WHITE); display.drawPixel(cx-8, cy+i, SSD1306_WHITE); display.drawPixel(cx+8, cy+i, SSD1306_WHITE);}
      break;
    case 4: // WIFI SERVER
      display.drawCircle(cx, cy+3, 2, SSD1306_WHITE);
      display.drawCircle(cx, cy+3, 5, SSD1306_WHITE);
      display.drawCircle(cx, cy+3, 8, SSD1306_WHITE);
      display.drawLine(cx-8, cy+3, cx+8, cy+3, SSD1306_WHITE);
      break;
    case 5: // BLE SCAN
      display.drawLine(cx, cy-8, cx, cy+8, SSD1306_WHITE);
      display.drawLine(cx, cy-8, cx+5, cy-3, SSD1306_WHITE);
      display.drawLine(cx+5, cy-3, cx-5, cy+3, SSD1306_WHITE);
      display.drawLine(cx-5, cy-3, cx+5, cy+3, SSD1306_WHITE);
      display.drawLine(cx+5, cy+3, cx, cy+8, SSD1306_WHITE);
      break;
    case 6: // 3D CUBE
      display.drawRect(cx-6, cy-6, 12, 12, SSD1306_WHITE);
      display.drawRect(cx-3, cy-3, 12, 12, SSD1306_WHITE);
      display.drawLine(cx-6, cy-6, cx-3, cy-3, SSD1306_WHITE);
      display.drawLine(cx+6, cy-6, cx+9, cy-3, SSD1306_WHITE);
      display.drawLine(cx-6, cy+6, cx-3, cy+9, SSD1306_WHITE);
      display.drawLine(cx+6, cy+6, cx+9, cy+9, SSD1306_WHITE);
      break;
    case 7: // SNAKE
      display.drawRect(cx-7, cy-3, 4, 4, SSD1306_WHITE);
      display.drawRect(cx-3, cy-3, 4, 4, SSD1306_WHITE);
      display.drawRect(cx+1, cy-3, 4, 4, SSD1306_WHITE);
      display.fillCircle(cx+6, cy+3, 2, SSD1306_WHITE);
      break;
    case 8: // TETRIS
      display.fillRect(cx-7, cy-6, 4, 4, SSD1306_WHITE);
      display.fillRect(cx-3, cy-6, 4, 4, SSD1306_WHITE);
      display.fillRect(cx-3, cy-2, 4, 4, SSD1306_WHITE);
      display.fillRect(cx+1, cy+2, 4, 4, SSD1306_WHITE);
      break;
    case 9: // PONG
      display.drawRect(cx-8, cy-4, 2, 8, SSD1306_WHITE);
      display.drawRect(cx+6, cy-2, 2, 8, SSD1306_WHITE);
      display.fillCircle(cx, cy, 2, SSD1306_WHITE);
      break;
    case 10: // FLAPPY
      display.fillCircle(cx-2, cy, 3, SSD1306_WHITE);
      display.drawRect(cx+4, cy-8, 3, 6, SSD1306_WHITE);
      display.drawRect(cx+4, cy+2, 3, 6, SSD1306_WHITE);
      break;
    case 11: // GEO DASH
      display.fillRect(cx-6, cy-6, 12, 12, SSD1306_WHITE);
      display.drawLine(cx-6, cy+6, cx+6, cy+6, SSD1306_WHITE);
      display.drawPixel(cx-3, cy-3, SSD1306_BLACK);
      display.drawPixel(cx+3, cy-3, SSD1306_BLACK);
      break;
    case 12: // INVADERS
      display.drawRect(cx-6, cy-4, 4, 4, SSD1306_WHITE);
      display.drawRect(cx+2, cy-4, 4, 4, SSD1306_WHITE);
      display.drawLine(cx, cy+2, cx, cy+8, SSD1306_WHITE);
      display.drawLine(cx-4, cy+8, cx+4, cy+8, SSD1306_WHITE);
      break;
    case 13: // STACK TOWER
      display.fillRect(cx-6, cy+4, 12, 3, SSD1306_WHITE);
      display.fillRect(cx-5, cy, 10, 3, SSD1306_WHITE);
      display.fillRect(cx-4, cy-4, 8, 3, SSD1306_WHITE);
      display.fillRect(cx-3, cy-8, 6, 3, SSD1306_WHITE);
      break;
    case 14: // PULSE DODGE
      display.drawCircle(cx, cy, 3, SSD1306_WHITE);
      display.drawCircle(cx, cy, 6, SSD1306_WHITE);
      display.drawLine(cx-6, cy-6, cx+6, cy+6, SSD1306_WHITE);
      display.drawLine(cx+6, cy-6, cx-6, cy+6, SSD1306_WHITE);
      break;
    case 15: // SIMON
      display.drawCircle(cx-4, cy-4, 3, SSD1306_WHITE);
      display.drawCircle(cx+4, cy-4, 3, SSD1306_WHITE);
      display.drawCircle(cx-4, cy+4, 3, SSD1306_WHITE);
      display.drawCircle(cx+4, cy+4, 3, SSD1306_WHITE);
      break;
    case 16: // T-REX
      display.drawRect(cx-6, cy-4, 6, 8, SSD1306_WHITE);
      display.drawRect(cx-6, cy-8, 8, 4, SSD1306_WHITE);
      display.drawPixel(cx-2, cy-7, SSD1306_BLACK);
      break;
    case 20: // BREAKOUT
      display.drawRect(cx-7, cy+5, 14, 2, SSD1306_WHITE);
      display.fillCircle(cx, cy-2, 2, SSD1306_WHITE);
      display.drawRect(cx-6, cy-8, 3, 3, SSD1306_WHITE);
      display.drawRect(cx-1, cy-8, 3, 3, SSD1306_WHITE);
      display.drawRect(cx+4, cy-8, 3, 3, SSD1306_WHITE);
      break;
    case 21: // 2048
      display.drawRect(cx-8, cy-8, 16, 16, SSD1306_WHITE);
      display.drawLine(cx, cy-8, cx, cy+8, SSD1306_WHITE);
      display.drawLine(cx-8, cy, cx+8, cy, SSD1306_WHITE);
      display.fillRect(cx+1, cy-7, 6, 6, SSD1306_WHITE);
      display.fillRect(cx-7, cy+1, 6, 6, SSD1306_WHITE);
      break;
    case 22: // MAZE
      display.drawRect(cx-8, cy-8, 16, 16, SSD1306_WHITE);
      display.drawLine(cx-8, cy-2, cx-1, cy-2, SSD1306_WHITE);
      display.drawLine(cx-1, cy-2, cx-1, cy+4, SSD1306_WHITE);
      display.drawLine(cx-1, cy+4, cx+5, cy+4, SSD1306_WHITE);
      display.drawLine(cx+5, cy+4, cx+5, cy-4, SSD1306_WHITE);
      break;
    case 23: // WHACK-A-MOLE
      display.drawRect(cx-8, cy-8, 6, 6, SSD1306_WHITE);
      display.drawRect(cx+2, cy-8, 6, 6, SSD1306_WHITE);
      display.drawRect(cx-8, cy+2, 6, 6, SSD1306_WHITE);
      display.drawRect(cx+2, cy+2, 6, 6, SSD1306_WHITE);
      display.fillCircle(cx-5, cy-5, 2, SSD1306_WHITE);
      break;
    case 24: // 3D MAZE
      display.drawCircle(cx, cy, 7, SSD1306_WHITE);
      display.drawLine(cx-9, cy, cx-4, cy, SSD1306_WHITE);
      display.drawLine(cx+4, cy, cx+9, cy, SSD1306_WHITE);
      display.drawLine(cx, cy-9, cx, cy-4, SSD1306_WHITE);
      display.drawLine(cx, cy+4, cx, cy+9, SSD1306_WHITE);
      display.drawPixel(cx, cy, SSD1306_WHITE);
      break;
    case 25: // TV REMOTE
      display.drawRect(cx-7, cy-8, 14, 16, SSD1306_WHITE);
      display.fillCircle(cx-3, cy-4, 1, SSD1306_WHITE);
      display.fillCircle(cx+3, cy-4, 1, SSD1306_WHITE);
      display.fillCircle(cx-3, cy+2, 1, SSD1306_WHITE);
      display.fillCircle(cx+3, cy+2, 1, SSD1306_WHITE);
      display.drawLine(cx-4, cy-6, cx+4, cy-6, SSD1306_WHITE);
      break;
    case 26: // BADUSB LAB
      display.drawRect(cx-6, cy-6, 12, 12, SSD1306_WHITE);
      display.drawRect(cx-3, cy-3, 6, 6, SSD1306_WHITE);
      for (int i=-6;i<=6;i+=3){display.drawPixel(cx+i, cy-8, SSD1306_WHITE); display.drawPixel(cx+i, cy+8, SSD1306_WHITE); display.drawPixel(cx-8, cy+i, SSD1306_WHITE); display.drawPixel(cx+8, cy+i, SSD1306_WHITE);}
      break;
    case 27: // NEXUS PET
      display.drawRoundRect(cx-7, cy-5, 7, 8, 3, SSD1306_WHITE);
      display.drawRoundRect(cx+2, cy-5, 7, 8, 3, SSD1306_WHITE);
      display.fillCircle(cx-4, cy-1, 2, SSD1306_WHITE);
      display.fillCircle(cx+5, cy-1, 2, SSD1306_WHITE);
      break;
    case 28: // BLE HID
      display.drawLine(cx, cy-8, cx, cy+8, SSD1306_WHITE);
      display.drawLine(cx, cy-8, cx+5, cy-3, SSD1306_WHITE);
      display.drawLine(cx+5, cy-3, cx-5, cy+3, SSD1306_WHITE);
      display.drawLine(cx-5, cy-3, cx+5, cy+3, SSD1306_WHITE);
      display.drawLine(cx+5, cy+3, cx, cy+8, SSD1306_WHITE);
      break;
    case 29: // IR RANDOM — знак вопроса
      display.setTextSize(3);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(cx - 7, cy - 15);
      display.print("?");
      display.setTextSize(1);
      break;
    case 30: // WIFI SCANNER
      display.drawCircle(cx, cy - 4, 7, SSD1306_WHITE);
      display.fillRect(cx - 5, cy + 4, 10, 3, SSD1306_WHITE);
      display.fillRect(cx - 4, cy + 7, 3, 2, SSD1306_WHITE);
      display.fillRect(cx + 1, cy + 7, 3, 2, SSD1306_WHITE);
      display.fillCircle(cx - 3, cy - 4, 2, SSD1306_BLACK);
      display.fillCircle(cx + 3, cy - 4, 2, SSD1306_BLACK);
      display.drawPixel(cx, cy - 1, SSD1306_BLACK);
      display.drawPixel(cx - 1, cy + 1, SSD1306_BLACK);
      display.drawPixel(cx + 1, cy + 1, SSD1306_BLACK);
      break;
  }
}

// ================= МЕНЮ =================
const uint32_t MENU_ANIM_MS = 200;

struct MenuStar { int x, y, speed, depth; };
MenuStar menuStars[22];
bool starsInit = false;
void initStars() {
  for (int i = 0; i < 22; i++) {
    menuStars[i].x = random(0, 128);
    menuStars[i].y = random(13, 50);
    menuStars[i].depth = random(0, 2);
    menuStars[i].speed = 1 + menuStars[i].depth * 2;
  }
  starsInit = true;
}
void drawStars() {
  if (!starsInit) initStars();
  for (int i = 0; i < 22; i++) {
    display.drawPixel(menuStars[i].x, menuStars[i].y, SSD1306_WHITE);
    if (menuStars[i].depth) display.drawPixel(menuStars[i].x + 1, menuStars[i].y, SSD1306_WHITE);
    menuStars[i].x -= menuStars[i].speed;
    if (menuStars[i].x < 0) { menuStars[i].x = 128; menuStars[i].y = random(13, 50); }
  }
}

void drawModeNameText(const char* t, int xOff, int yBase) {
  int16_t x1, y1; uint16_t w, h;
  display.setTextSize(2);
  display.getTextBounds(t, 0, 0, &x1, &y1, &w, &h);
  if (w > 90) {
    display.setTextSize(1);
    display.getTextBounds(t, 0, 0, &x1, &y1, &w, &h);
  }
  int x = 84 - (w / 2) + xOff;
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setCursor(x, yBase);
  display.print(t);
}

void drawCarouselFrame(int prevIdx, int curIdx, int progress, int dir, bool animate) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  drawStars();
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("NEXUS OS");
  char cnt[10]; snprintf(cnt, 10, "%d/%d", curIdx + 1, totalModes);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(cnt, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(128 - w, 0); display.print(cnt);
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  int prevOff = 0, curOff = 0;
  if (animate) {
    if (dir > 0) { prevOff = -(140 * progress / 100); curOff = 140 - (140 * progress / 100); }
    else          { prevOff = (140 * progress / 100);  curOff = -140 + (140 * progress / 100); }
  }

  int curModeId = menuModeIds[curIdx];
  int prevModeId = (prevIdx >= 0 && prevIdx < totalModes) ? menuModeIds[prevIdx] : -1;

  if (curOff > -128 && curOff < 128) {
    display.drawRect(2, 14, 32, 32, SSD1306_WHITE);
    drawModeIcon(curModeId, 18 + curOff / 3, 30);
  }
  if (animate && prevIdx != curIdx && prevOff > -128 && prevOff < 128) {
    display.drawRect(2, 14, 32, 32, SSD1306_WHITE);
    drawModeIcon(prevModeId, 18 + prevOff / 3, 30);
  }

  if (curOff > -128 && curOff < 128) {
    drawModeNameText(modeNames[curIdx], curOff, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    int descW = strlen(modeDesc[curIdx]) * 6;
    int dx = 84 - (descW / 2) + curOff;
    display.setCursor(dx, 38); display.print(modeDesc[curIdx]);
  }

  display.setTextSize(2); display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setCursor(0, 24); display.print("<");
  display.setCursor(112, 24); display.print(">");

  display.drawLine(0, 48, 128, 48, SSD1306_WHITE);
  char numTxt[4]; snprintf(numTxt, 4, "%02d", curIdx + 1);
  display.setTextSize(1); display.setCursor(2, 54); display.print(numTxt);
  int barX = 20, barW = 84;
  display.drawRect(barX, 54, barW, 8, SSD1306_WHITE);
  int fill = (barW - 2) * (curIdx + 1) / totalModes;
  display.fillRect(barX + 1, 55, fill, 6, SSD1306_WHITE);

  char st[16]; snprintf(st, 16, "IR:%d", savedCount);
  int16_t sx1, sy1; uint16_t sw, sh;
  display.getTextBounds(st, 0, 0, &sx1, &sy1, &sw, &sh);
  display.setCursor(126 - sw, 54); display.print(st);

  display.display();
}

void drawCarouselMenu() {
  buttonsScan();
  beepUpdate();
  static int shownIdx = -1, targetIdx = 0, animFrom = 0, animDir = 1;
  static bool animating = false;
  static uint32_t animStart = 0;

  if (shownIdx < 0) {
    shownIdx = currentMode; targetIdx = currentMode;
    if (shownIdx >= totalModes) shownIdx = 0;
    drawCarouselFrame(shownIdx, shownIdx, 100, 1, false);
    delay(20); return;
  }

  if (!animating) {
    if (btnPressed(B_LEFT)) {
      targetIdx = (shownIdx - 1 + totalModes) % totalModes;
      animFrom = shownIdx; animDir = -1; animating = true; animStart = millis();
      currentMode = targetIdx;
      sfxClick();
    } else if (btnPressed(B_RIGHT)) {
      targetIdx = (shownIdx + 1) % totalModes;
      animFrom = shownIdx; animDir = +1; animating = true; animStart = millis();
      currentMode = targetIdx;
      sfxClick();
    } else if (btnPressed(B_SEL) && !btnLongFired[B_SEL]) {
      inMenu = false;
      currentMode = menuModeIds[shownIdx];
      shownIdx = -1; delay(150); sfxEnter(); return;
    }
  }

  if (animating) {
    uint32_t elapsed = millis() - animStart;
    int progress = (elapsed * 100) / MENU_ANIM_MS;
    if (progress >= 100) { progress = 100; shownIdx = targetIdx; animating = false; }
    drawCarouselFrame(animFrom, targetIdx, progress, animDir, progress < 100);
  } else {
    drawCarouselFrame(shownIdx, shownIdx, 100, 1, false);
  }
  delay(15);
}

// ================= BOOT SCREEN =================
void bootScreen() {
  display.clearDisplay();
  display.drawBitmap(0, 0, myBitmap, 128, 64, SSD1306_WHITE);
  display.display();
  delay(2000);
  int notes[] = {523, 659, 784, 1046};
  for (int i = 0; i < 4; i++) {
    ledcWriteTone(BUZZER_PIN, notes[i]); delay(120); ledcWriteTone(BUZZER_PIN, 0); delay(40);
  }
  for (int y = 64; y >= 0; y -= 4) {
    display.clearDisplay();
    drawCarouselFrame(currentMode, currentMode, 100, 1, false);
    display.fillRect(0, y, 128, 64 - y, SSD1306_BLACK);
    display.display();
    delay(15);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  for (int i = 0; i < NUM_BTNS; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);
  pinMode(IR_RECV_PIN, INPUT);
  pinMode(IR_SEND_PIN, OUTPUT);
  digitalWrite(IR_SEND_PIN, LOW);

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(0); display.clearDisplay(); display.display();
  loadCodesFromFlash();
  petLoad();
  petInit();
  irrecv.enableIRIn();
  irsend.begin();
  digitalWrite(IR_SEND_PIN, LOW);
  WiFi.mode(WIFI_OFF);
  bootScreen();
}

// ================= LOOP =================
void loop() {
  beepUpdate();
  petUpdateBackground();

  if (inMenu) { drawCarouselMenu(); return; }
  switch (currentMode) {
    case 0: drawIRRecorder(); break;
    case 2: drawIRSaved();    break;
    case 3: drawSystemInfo(); break;
    case 4: drawWifiServer(); break;
    case 5: drawBleScan();    break;
    case 6: draw3DCube();     break;
    case 7: drawSnake();      break;
    case 8: drawTetris();     break;
    case 9: drawPong();       break;
    case 10: drawFlappy();    break;
    case 11: drawGeometryDash(); break;
    case 12: drawSpaceInvaders(); break;
    case 13: drawStackTower(); break;
    case 14: drawPulseDodge(); break;
    case 15: drawSimonSays();  break;
    case 16: drawTRex();       break;
    case 20: drawBreakout();   break;
    case 21: draw2048();       break;
    case 22: drawMaze();       break;
    case 23: drawWhackAMole(); break;
    case 24: drawMaze3D();     break;
    case 25: drawTvCodes();    break;
    case 26: drawUsbLab();     break;
    case 27: drawNexusPet();   break;
    case 28: drawBleHid();     break;
    case 29: drawIRRandom();   break;
    case 30: drawWifiDeauth(); break;
  }
}
