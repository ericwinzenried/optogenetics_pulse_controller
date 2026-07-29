/*
  RTPP_Pulser_OLED.ino
  --------------------
  Adafruit Feather M0 + 128x64 monochrome OLED FeatherWing (SH1107)

  Behavior:
    - A0 receives a digital (TTL) gate signal from Bonsai.
    - While A0 is HIGH, pin 13 emits a pulse train at the user-set
      frequency and pulse width. While A0 is LOW, pin 13 is held LOW.
    - The OLED FeatherWing's three buttons adjust frequency and pulse
      width on the fly, with the values shown on screen.
    - Settings can be written to flash so they survive a power cycle.

  Buttons (with the wing rotated so the buttons are on the left edge):
    A (top,    D9) = increase selected value
    B (middle, D6) = tap to move to the next parameter
                     hold 2 s to save the current settings to flash
    C (bottom, D5) = decrease selected value
    Hold A or C to repeat; keep holding to speed up.

  The top-left corner reads "saved" or "unsaved": "unsaved" means the
  on-screen settings differ from what is stored in flash, i.e. they will be
  lost at the next power cycle.

  Libraries required (Library Manager):
    Adafruit GFX Library
    Adafruit SH110X          <-- the 128x64 wing uses an SH1107 controller.
                                 (The older 128x32 wing is SSD1306 instead.)
    FlashStorage             <-- by Cristian Maglie / Arduino, for SAMD boards.

  Notes / gotchas:
    - On the Feather M0, LED_BUILTIN *is* pin 13, so the original sketch's
      two digitalWrite() calls were driving the same pin. Here pin 13 is
      used once, as the TTL output; the red LED follows it for free.
    - The wing's button A shares D9 with the Feather M0's A7 battery-voltage
      divider. Button A still works; you just can't read battery voltage
      while using it.
    - Timing is done with micros() and a non-blocking state machine (no
      delay()), so button presses and screen redraws never stretch a pulse.
    - The SAMD21 has no EEPROM, so "saving" means erasing and rewriting a
      page of program flash. That blocks for a few ms with interrupts off,
      so a save is never performed while a train is running (it is deferred
      until the gate drops), and only when a value has actually changed.
      Flash is rated for a limited number of erase cycles, which is why this
      is a deliberate gesture rather than an automatic write on every press.
    - Re-uploading the sketch wipes the saved settings, since they live in
      the same flash that the uploader erases.

  Original 20 Hz / 10 ms version: Lex Kravitz, Aug 2018.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <FlashStorage.h>

// ---------------------------------------------------------------------------
// Hardware map
// ---------------------------------------------------------------------------

const uint8_t PIN_TRIGGER   = A0;  // gate input from Bonsai
const uint8_t PIN_TTL_OUT   = 13;  // TTL pulse train out (also the red LED)
const uint8_t PIN_BTN_UP    = 9;   // FeatherWing button A
const uint8_t PIN_BTN_SEL   = 6;   // FeatherWing button B
const uint8_t PIN_BTN_DOWN  = 5;   // FeatherWing button C

// The 128x64 wing is a 64x128 panel that we rotate into landscape, so the
// constructor gets the *native* dimensions and setRotation() does the rest.
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
const uint8_t OLED_I2C_ADDR = 0x3C;

// ---------------------------------------------------------------------------
// Adjustable settings and their limits
// ---------------------------------------------------------------------------

uint16_t freqHz  = 20;   // pulse train frequency
uint16_t widthMs = 10;   // pulse on-time

const uint16_t FREQ_MIN_HZ  = 1;
const uint16_t FREQ_MAX_HZ  = 100;
const uint16_t WIDTH_MIN_MS = 1;

// The parameter the buttons currently act on.
enum Param { PARAM_FREQ = 0, PARAM_WIDTH = 1, PARAM_COUNT = 2 };
uint8_t selectedParam = PARAM_FREQ;

// Set true if you want the screen to keep updating during a pulse train.
// Left false by default: an I2C screen write takes a few ms, and skipping it
// while pulsing guarantees the pulse edges stay clean. With it false, the
// display refreshes the moment the train ends.
const bool ALLOW_REDRAW_DURING_TRAIN = false;

// Time the sketch reserves for a screen write when the option above is true.
const uint32_t REDRAW_BUDGET_US = 8000;

// ---------------------------------------------------------------------------
// Persistent storage
// ---------------------------------------------------------------------------

// The magic number distinguishes a real saved record from blank flash, so a
// board that has never been saved to starts from the defaults above.
const uint32_t SETTINGS_MAGIC = 0x52545030UL;  // "RTP0"

struct StoredSettings {
  uint32_t magic;
  uint16_t freqHz;
  uint16_t widthMs;
};

// Reserves a flash slot named settingsStore that holds one StoredSettings.
FlashStorage(settingsStore, StoredSettings);

// Mirror of what is currently in flash, used to detect unsaved edits and to
// avoid burning an erase cycle on a save that wouldn't change anything.
uint16_t savedFreqHz  = 0;
uint16_t savedWidthMs = 0;

// Set when a save is requested mid-train; performed once the gate drops.
bool savePending = false;

// ---------------------------------------------------------------------------
// Pulse engine state
// ---------------------------------------------------------------------------

bool     trainRunning  = false;  // is the Bonsai gate currently HIGH?
bool     outputHigh    = false;  // current state of PIN_TTL_OUT
uint32_t pulseStartUs  = 0;      // micros() timestamp of the current pulse's rising edge
uint32_t pulseCount    = 0;      // pulses delivered in the most recent train

// Settings are copied into these when a train starts, so that turning a knob
// mid-train can't shift an edge that has already been scheduled.
uint32_t activePeriodUs = 0;
uint32_t activeWidthUs  = 0;

bool displayDirty = true;  // set whenever something on screen has changed

// Transient message shown over the normal screen (e.g. "SETTINGS SAVED").
const char *toastText   = NULL;
uint32_t    toastUntilMs = 0;
const uint32_t TOAST_MS = 1200;

// ---------------------------------------------------------------------------
// Button handling: debounce, press-and-hold auto-repeat, and long-press
// ---------------------------------------------------------------------------

const uint32_t DEBOUNCE_MS     = 25;
const uint32_t REPEAT_DELAY_MS = 400;   // wait before the first auto-repeat
const uint32_t REPEAT_SLOW_MS  = 120;   // repeat interval at first
const uint32_t REPEAT_FAST_MS  = 30;    // repeat interval once held a while
const uint32_t REPEAT_RAMP_MS  = 1200;  // how long until repeats speed up
const uint32_t LONG_HOLD_MS    = 2000;  // hold time that triggers a save

struct Button {
  uint8_t  pin;
  bool     pressed;       // debounced state
  uint32_t lastChangeMs;  // when the debounced state last flipped
  uint32_t heldSinceMs;   // when the current press began
  uint32_t nextRepeatMs;  // when the next auto-repeat is due
  bool     longFired;     // has this press already triggered the long action?
  // Event flags below are valid only for the loop pass that follows
  // updateButton(); they are cleared at the top of every call.
  bool justPressed;
  bool justReleased;
  bool repeated;
  bool heldLong;          // fires once, as the hold threshold is crossed
};

Button btnUp   = { PIN_BTN_UP,   false, 0, 0, 0, false, false, false, false, false };
Button btnSel  = { PIN_BTN_SEL,  false, 0, 0, 0, false, false, false, false, false };
Button btnDown = { PIN_BTN_DOWN, false, 0, 0, 0, false, false, false, false, false };

void updateButton(Button &b) {
  uint32_t t = millis();

  b.justPressed  = false;
  b.justReleased = false;
  b.repeated     = false;
  b.heldLong     = false;

  // Buttons pull the pin to ground, so LOW means pressed.
  bool raw = (digitalRead(b.pin) == LOW);

  if (raw != b.pressed) {
    // Ignore transitions that arrive inside the debounce window.
    if ((t - b.lastChangeMs) >= DEBOUNCE_MS) {
      b.pressed = raw;
      b.lastChangeMs = t;
      if (raw) {
        b.heldSinceMs  = t;
        b.nextRepeatMs = t + REPEAT_DELAY_MS;
        b.longFired    = false;
        b.justPressed  = true;
      } else {
        b.justReleased = true;
      }
    }
  } else if (b.pressed) {
    // Signed subtraction keeps this correct across the millis() rollover.
    if ((int32_t)(t - b.nextRepeatMs) >= 0) {
      b.repeated = true;
      uint32_t heldMs = t - b.heldSinceMs;
      b.nextRepeatMs = t + (heldMs > REPEAT_RAMP_MS ? REPEAT_FAST_MS : REPEAT_SLOW_MS);
    }
    // Long-press fires once, while the button is still down, so the gesture
    // gives feedback at the 2 s mark rather than on release.
    if (!b.longFired && (t - b.heldSinceMs) >= LONG_HOLD_MS) {
      b.longFired = true;
      b.heldLong  = true;
    }
  }
}

// ---------------------------------------------------------------------------
// Settings helpers
// ---------------------------------------------------------------------------

uint32_t periodUs() {
  return 1000000UL / freqHz;
}

// Largest legal pulse width for the current frequency. At least 1 ms of
// off-time is kept so the train can never collapse into a constant HIGH.
uint16_t maxWidthMs() {
  uint16_t periodMs = (uint16_t)(periodUs() / 1000UL);
  return (periodMs > WIDTH_MIN_MS) ? (periodMs - 1) : WIDTH_MIN_MS;
}

// Called after any change so an edit to one parameter can't leave the other
// in an impossible state (e.g. 10 ms pulses requested at 100 Hz).
void clampSettings() {
  if (freqHz < FREQ_MIN_HZ) freqHz = FREQ_MIN_HZ;
  if (freqHz > FREQ_MAX_HZ) freqHz = FREQ_MAX_HZ;
  if (widthMs < WIDTH_MIN_MS) widthMs = WIDTH_MIN_MS;
  if (widthMs > maxWidthMs()) widthMs = maxWidthMs();
}

void adjustSelected(int16_t delta) {
  if (selectedParam == PARAM_FREQ) {
    // Cast to int32 first so decrementing 1 Hz can't wrap around to 65535.
    freqHz = (uint16_t)constrain((int32_t)freqHz + delta, 0, 65535);
  } else {
    widthMs = (uint16_t)constrain((int32_t)widthMs + delta, 0, 65535);
  }
  clampSettings();
  displayDirty = true;
}

bool settingsUnsaved() {
  return (freqHz != savedFreqHz) || (widthMs != savedWidthMs);
}

void loadSettings() {
  StoredSettings s = settingsStore.read();
  if (s.magic == SETTINGS_MAGIC) {
    freqHz  = s.freqHz;
    widthMs = s.widthMs;
    clampSettings();
    savedFreqHz  = freqHz;
    savedWidthMs = widthMs;
  } else {
    // Nothing valid in flash: keep the compiled-in defaults, and leave the
    // mirror at 0 so the screen shows these values as unsaved.
    clampSettings();
    savedFreqHz  = 0;
    savedWidthMs = 0;
  }
}

// Erases and rewrites the flash page. Blocks for a few ms with interrupts
// disabled, so callers must only invoke this while the gate is low.
void saveSettings() {
  StoredSettings s;
  s.magic   = SETTINGS_MAGIC;
  s.freqHz  = freqHz;
  s.widthMs = widthMs;
  settingsStore.write(s);
  savedFreqHz  = freqHz;
  savedWidthMs = widthMs;
}

void showToast(const char *text) {
  toastText    = text;
  toastUntilMs = millis() + TOAST_MS;
  displayDirty = true;
}

// ---------------------------------------------------------------------------
// Pulse engine
// ---------------------------------------------------------------------------

void setOutput(bool high) {
  digitalWrite(PIN_TTL_OUT, high ? HIGH : LOW);
  outputHigh = high;
}

// Gate went HIGH: latch the settings and fire the first pulse immediately so
// stimulation onset stays aligned with Bonsai's trigger.
void startTrain() {
  activePeriodUs = periodUs();
  activeWidthUs  = (uint32_t)widthMs * 1000UL;
  pulseStartUs   = micros();
  pulseCount     = 1;
  trainRunning   = true;
  setOutput(true);
  displayDirty = true;
}

// Gate went LOW: stop mid-train, leaving the output safely low.
void stopTrain() {
  trainRunning = false;
  setOutput(false);
  displayDirty = true;
}

// Advances the pulse train. Called every pass through loop() while gated.
void serviceTrain() {
  uint32_t now = micros();

  if (outputHigh) {
    // Falling edge: the on-time has elapsed.
    if ((uint32_t)(now - pulseStartUs) >= activeWidthUs) {
      setOutput(false);
    }
  } else {
    uint32_t elapsed = (uint32_t)(now - pulseStartUs);
    if (elapsed >= activePeriodUs) {
      // Advance the reference time by exactly one period rather than resetting
      // it to now(), so rounding error can't accumulate over a long train.
      if (elapsed > 2 * activePeriodUs) {
        // More than a period was missed (shouldn't happen, but don't try to
        // "catch up" by firing a burst of back-to-back pulses).
        pulseStartUs = now;
      } else {
        pulseStartUs += activePeriodUs;
      }
      setOutput(true);
      pulseCount++;
    }
  }
}

// True when there is enough dead time to push a frame to the OLED without
// risking a late pulse edge.
bool safeToRedraw() {
  if (!trainRunning) return true;
  if (!ALLOW_REDRAW_DURING_TRAIN) return false;
  if (outputHigh) return false;
  // How long until the next rising edge is due?
  uint32_t elapsed = (uint32_t)(micros() - pulseStartUs);
  if (elapsed >= activePeriodUs) return false;
  return (activePeriodUs - elapsed) > REDRAW_BUDGET_US;
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

// Draws text ending at xRight, which keeps numbers of different digit counts
// lined up on their last character instead of jittering left and right.
void printRightAligned(const char *text, int16_t xRight, int16_t y, uint8_t size) {
  int16_t width = (int16_t)strlen(text) * 6 * size;
  display.setCursor(xRight - width, y);
  display.print(text);
}

// One parameter row: label on the left, big value on the right, and an outline
// around the row that is currently selected. Both text sizes are centered
// within the given height, so the row can be resized without redoing offsets.
void drawParamRow(int16_t y, int16_t height, const char *label,
                  const char *value, bool selected) {
  display.setTextSize(1);
  display.setCursor(6, y + (height - 8) / 2);
  display.print(label);
  display.setTextSize(2);
  printRightAligned(value, 124, y + (height - 16) / 2, 2);
  if (selected) {
    display.drawRect(0, y, 128, height, SH110X_WHITE);
  }
}

// Centered message box drawn on top of everything else.
void drawToast(const char *text) {
  int16_t textWidth = (int16_t)strlen(text) * 6;
  int16_t boxWidth  = textWidth + 12;
  int16_t x = (128 - boxWidth) / 2;
  int16_t y = 22;
  // Clear the area first so the text underneath doesn't show through.
  display.fillRect(x, y, boxWidth, 16, SH110X_BLACK);
  display.drawRect(x, y, boxWidth, 16, SH110X_WHITE);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(x + 6, y + 4);
  display.print(text);
}

void drawDisplay() {
  // Sized for the longest string built below.
  char buf[32];

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  // --- Status line: flash state on the left, gate state on the right ---
  display.setCursor(0, 0);
  display.print(settingsUnsaved() ? "unsaved" : "saved");

  if (trainRunning) {
    snprintf(buf, sizeof(buf), "RUN %lu", (unsigned long)pulseCount);
  } else if (pulseCount > 0) {
    // After a train, show how many pulses the last one delivered.
    snprintf(buf, sizeof(buf), "last %lu", (unsigned long)pulseCount);
  } else {
    snprintf(buf, sizeof(buf), "ready");
  }
  printRightAligned(buf, 128, 0, 1);

  // A thin rule rather than the filled bar used before: this panel already
  // shows image retention, and a large permanently-lit area is what causes it.
  display.drawFastHLine(0, 9, 128, SH110X_WHITE);

  // --- Parameter rows ---
  snprintf(buf, sizeof(buf), "%u Hz", freqHz);
  drawParamRow(11, 18, "FREQ", buf, selectedParam == PARAM_FREQ);

  snprintf(buf, sizeof(buf), "%u ms", widthMs);
  drawParamRow(29, 18, "ON", buf, selectedParam == PARAM_WIDTH);

  // --- Button legend, laid out to match the buttons' top-to-bottom order ---
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("A +    B next    C -");
  display.setCursor(0, 56);
  display.print("hold B 2s = save");

  if (toastText != NULL) {
    drawToast(toastText);
  }

  display.display();
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
  pinMode(PIN_TTL_OUT, OUTPUT);
  digitalWrite(PIN_TTL_OUT, LOW);

  // Pulldown so a disconnected or floating BNC reads as "no stimulation".
  pinMode(PIN_TRIGGER, INPUT_PULLDOWN);

  // The wing's buttons switch to ground, so they need pull-ups.
  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_SEL,  INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

  display.begin(OLED_I2C_ADDR, true);
  // 400 kHz I2C: roughly four times faster than the default, which keeps
  // screen writes short enough to slot between pulses.
  Wire.setClock(400000);
  display.setRotation(1);       // landscape, buttons on the left edge
  display.setTextWrap(false);   // long strings clip instead of wrapping
  display.clearDisplay();       // drop the library's splash screen

  loadSettings();
  drawDisplay();
  displayDirty = false;
}

void loop() {
  // 1. Follow the gate input from Bonsai.
  bool gateHigh = (digitalRead(PIN_TRIGGER) == HIGH);
  if (gateHigh && !trainRunning) {
    startTrain();
  } else if (!gateHigh && trainRunning) {
    stopTrain();
  }

  // 2. Keep the pulse train on schedule. This runs first among the
  //    housekeeping tasks so edges take priority over UI work.
  if (trainRunning) {
    serviceTrain();
  }

  // 3. Read the buttons.
  updateButton(btnUp);
  updateButton(btnSel);
  updateButton(btnDown);

  // Tap B to change parameter; a tap is a release that didn't turn into a
  // hold, so the two gestures never fire together.
  if (btnSel.justReleased && !btnSel.longFired) {
    selectedParam = (selectedParam + 1) % PARAM_COUNT;
    displayDirty = true;
  }
  // Hold B for 2 s to commit the current settings to flash.
  if (btnSel.heldLong) {
    if (!settingsUnsaved()) {
      showToast("ALREADY SAVED");
    } else if (trainRunning) {
      // Never erase flash mid-train; queue it for the end of the trial.
      savePending = true;
      showToast("SAVE QUEUED");
    } else {
      saveSettings();
      showToast("SETTINGS SAVED");
    }
  }

  // Both edit buttons act on the initial press and on each auto-repeat.
  if (btnUp.justPressed || btnUp.repeated) {
    adjustSelected(+1);
  }
  if (btnDown.justPressed || btnDown.repeated) {
    adjustSelected(-1);
  }

  // 4. Carry out a save that was requested while the gate was high.
  if (savePending && !trainRunning) {
    savePending = false;
    saveSettings();
    showToast("SETTINGS SAVED");
  }

  // 5. Expire the toast message, which forces one more redraw to remove it.
  if (toastText != NULL && (int32_t)(millis() - toastUntilMs) >= 0) {
    toastText = NULL;
    displayDirty = true;
  }

  // 6. Redraw only when something changed and the timing allows it.
  if (displayDirty && safeToRedraw()) {
    drawDisplay();
    displayDirty = false;
  }
}
