/*
  RTPP_Pulser_OLED.ino
  --------------------
  Adafruit Feather M0 + 128x64 monochrome OLED FeatherWing (SH1107)

  Behavior:
    - A0 receives a digital (TTL) gate signal from Bonsai.
    - While A0 is HIGH, pin 13 emits a pulse train at the user-set
      frequency and pulse width. While A0 is LOW, pin 13 is held LOW.
    - Frequency and pulse width can be adjusted at any time, including
      while a train is running. Changes take effect on the next pulse.
    - Settings can be written to flash so they survive a power cycle.

  Buttons (with the wing rotated so the buttons are on the left edge):
    A (top,    D9) = increase selected value
    B (middle, D6) = tap to move to the next parameter
                     hold 2 s to save the current settings to flash
    C (bottom, D5) = decrease selected value
    Hold A or C to repeat; keep holding to speed up.

  The top-left corner reads:
    "saved"   - on-screen settings match what is in flash
    "unsaved" - they differ, and will be lost at the next power cycle
    "queued"  - a save was requested during a train and will happen when
                the gate drops (flash cannot be erased mid-train)

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
      delay()), so button presses never stretch a pulse.
    - Mid-train screen updates are the one thing that can perturb timing,
      because an I2C write blocks for milliseconds. They are therefore split
      into small regions, only sent while the output is LOW, and only when
      enough off-time remains before the next rising edge. See the
      "Screen refresh" section.
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

// The interval and on-time governing the pulse currently in flight. These are
// refreshed from the live settings at every rising edge, which is what lets a
// mid-train edit take effect without ever moving an edge already scheduled.
uint32_t activePeriodUs = 0;
uint32_t activeWidthUs  = 0;

// ---------------------------------------------------------------------------
// Screen refresh bookkeeping
// ---------------------------------------------------------------------------

// While idle the whole frame is redrawn, which is simple and safe. While a
// train is running only these small regions are pushed, one per loop pass, so
// that each I2C transfer is short enough to fit in the gap between pulses.
bool needFullRedraw   = true;
bool needValueRedraw[PARAM_COUNT] = { false, false };  // one row's value text
bool needMarkerRedraw = false;   // the selection arrow moved
bool needFlagRedraw   = false;   // saved / unsaved / queued changed
bool needGateRedraw   = false;   // ready / RUN / last N changed

// Conservative estimates of how long each partial transfer takes at 400 kHz,
// used to decide whether it fits before the next rising edge. Raise these if
// you ever see a late edge on the scope; lower them to allow refreshes at
// higher pulse rates.
const uint32_t BUDGET_VALUE_US = 7000;  // value text spans three memory pages
const uint32_t BUDGET_SMALL_US = 2500;  // narrow bands: marker, flag, gate

// Transient message shown over the normal screen. Only drawn on full frames,
// so toasts appear while idle; mid-train feedback uses the flag field instead.
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
  uint16_t beforeFreq  = freqHz;
  uint16_t beforeWidth = widthMs;

  if (selectedParam == PARAM_FREQ) {
    // Cast to int32 first so decrementing 1 Hz can't wrap around to 65535.
    freqHz = (uint16_t)constrain((int32_t)freqHz + delta, 0, 65535);
  } else {
    widthMs = (uint16_t)constrain((int32_t)widthMs + delta, 0, 65535);
  }
  clampSettings();

  // Flag only the rows whose text actually changed. Raising the frequency can
  // pull the width down with it, so both rows may need refreshing from a
  // single button press.
  if (freqHz != beforeFreq)   needValueRedraw[PARAM_FREQ]  = true;
  if (widthMs != beforeWidth) needValueRedraw[PARAM_WIDTH] = true;
  needFlagRedraw  = true;   // the saved/unsaved state may have flipped
  needFullRedraw  = true;   // covers the idle case in one go
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
  needFullRedraw = true;
}

// ---------------------------------------------------------------------------
// Pulse engine
// ---------------------------------------------------------------------------

void setOutput(bool high) {
  digitalWrite(PIN_TTL_OUT, high ? HIGH : LOW);
  outputHigh = high;
}

// Copies the live settings into the active pair. Called at each rising edge,
// never mid-pulse, so the edge that is already scheduled is left alone.
void latchSettings() {
  activePeriodUs = periodUs();
  activeWidthUs  = (uint32_t)widthMs * 1000UL;
}

// Gate went HIGH: fire the first pulse immediately so stimulation onset stays
// aligned with Bonsai's trigger.
void startTrain() {
  latchSettings();
  pulseStartUs = micros();
  pulseCount   = 1;
  trainRunning = true;
  setOutput(true);
  needGateRedraw = true;
}

// Gate went LOW: stop mid-train, leaving the output safely low.
void stopTrain() {
  trainRunning = false;
  setOutput(false);
  needFullRedraw = true;
}

// Advances the pulse train. Called every pass through loop() while gated.
void serviceTrain() {
  uint32_t now = micros();

  if (outputHigh) {
    // Falling edge: the on-time has elapsed. If the width was just shortened
    // below the time already served, this fires on the very next pass.
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
      // Pick up any edit made since the last rising edge.
      latchSettings();
    }
  }
}

// True when at least budgetUs of off-time remains before the next rising edge,
// i.e. an I2C transfer of that length can finish without delaying the edge.
bool roomBeforeNextPulse(uint32_t budgetUs) {
  if (!trainRunning) return true;
  if (outputHigh) return false;  // never write to the screen mid-pulse
  uint32_t elapsed = (uint32_t)(micros() - pulseStartUs);
  if (elapsed >= activePeriodUs) return false;
  return (activePeriodUs - elapsed) > budgetUs;
}

// ---------------------------------------------------------------------------
// Screen geometry
//
// Shared by the full redraw and the partial ones so the two always agree.
// ---------------------------------------------------------------------------

const int16_t ROW_Y[PARAM_COUNT]     = { 11, 29 };  // top of each parameter row
const int16_t ROW_H                  = 18;
const int16_t VALUE_TEXT_Y[PARAM_COUNT] = { 12, 30 };  // top of the size-2 value
const int16_t VALUE_BAND_X           = 40;   // left edge of the value erase band
const int16_t VALUE_BAND_W           = 88;   // out to the right edge of the screen
const int16_t VALUE_RIGHT_EDGE       = 124;  // values are right-aligned here
const int16_t MARKER_BAND_W          = 6;    // column reserved for the arrow
const int16_t FLAG_BAND_W            = 48;   // fits "unsaved" (42 px)
const int16_t GATE_BAND_X            = 60;   // gate text is right-aligned to 128
const int16_t RULE_Y                 = 9;
const int16_t LEGEND_Y1              = 48;
const int16_t LEGEND_Y2              = 56;

// ---------------------------------------------------------------------------
// Display: text helpers
// ---------------------------------------------------------------------------

// Draws text ending at xRight, which keeps numbers of different digit counts
// lined up on their last character instead of jittering left and right.
void printRightAligned(const char *text, int16_t xRight, int16_t y, uint8_t size) {
  int16_t width = (int16_t)strlen(text) * 6 * size;
  display.setCursor(xRight - width, y);
  display.print(text);
}

const char *paramLabel(uint8_t param) {
  return (param == PARAM_FREQ) ? "FREQ" : "ON";
}

void formatParamValue(uint8_t param, char *out, size_t n) {
  if (param == PARAM_FREQ) {
    snprintf(out, n, "%u Hz", freqHz);
  } else {
    snprintf(out, n, "%u ms", widthMs);
  }
}

// Left slot: relationship between the on-screen settings and flash.
const char *flagText() {
  if (savePending) return "queued";
  return settingsUnsaved() ? "unsaved" : "saved";
}

// Right slot: what the gate is doing. The pulse count is deliberately left out
// while running, since a number that changed every pulse would demand a screen
// write every pulse.
void formatGateState(char *out, size_t n) {
  if (trainRunning) {
    snprintf(out, n, "RUN");
  } else if (pulseCount > 0) {
    snprintf(out, n, "last %lu", (unsigned long)pulseCount);
  } else {
    snprintf(out, n, "ready");
  }
}

// ---------------------------------------------------------------------------
// Display: partial refreshes
//
// Each of these erases one narrow band and redraws it, then pushes only that
// band over I2C. The SH110X library tracks a dirty rectangle, so the transfer
// is proportional to the area touched -- which is what makes these safe to do
// between pulses. Each ends with its own display() call so the dirty window
// stays small instead of growing to the union of several regions.
// ---------------------------------------------------------------------------

void drawValueField(uint8_t param) {
  char buf[16];
  display.fillRect(VALUE_BAND_X, VALUE_TEXT_Y[param], VALUE_BAND_W, 16, SH110X_BLACK);
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(2);
  formatParamValue(param, buf, sizeof(buf));
  printRightAligned(buf, VALUE_RIGHT_EDGE, VALUE_TEXT_Y[param], 2);
  display.display();
}

void drawSelectionMarker() {
  // Clear the whole arrow column, then draw the one arrow that belongs.
  int16_t bandTop    = ROW_Y[0];
  int16_t bandHeight = ROW_Y[PARAM_COUNT - 1] + ROW_H - bandTop;
  display.fillRect(0, bandTop, MARKER_BAND_W, bandHeight, SH110X_BLACK);
  int16_t cy = ROW_Y[selectedParam] + ROW_H / 2;
  display.fillTriangle(0, cy - 4, 0, cy + 4, 4, cy, SH110X_WHITE);
  display.display();
}

void drawFlagField() {
  display.fillRect(0, 0, FLAG_BAND_W, 8, SH110X_BLACK);
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(flagText());
  display.display();
}

void drawGateField() {
  char buf[16];
  display.fillRect(GATE_BAND_X, 0, 128 - GATE_BAND_X, 8, SH110X_BLACK);
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  formatGateState(buf, sizeof(buf));
  printRightAligned(buf, 128, 0, 1);
  display.display();
}

// ---------------------------------------------------------------------------
// Display: full frame
// ---------------------------------------------------------------------------

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

void drawFullScreen() {
  char buf[16];

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  // --- Status line: flash state on the left, gate state on the right ---
  display.setCursor(0, 0);
  display.print(flagText());
  formatGateState(buf, sizeof(buf));
  printRightAligned(buf, 128, 0, 1);

  // A thin rule rather than a filled bar: this panel already shows image
  // retention, and a large permanently-lit area is what causes it.
  display.drawFastHLine(0, RULE_Y, 128, SH110X_WHITE);

  // --- Parameter rows: small label left, large value right ---
  for (uint8_t p = 0; p < PARAM_COUNT; p++) {
    display.setTextSize(1);
    display.setCursor(MARKER_BAND_W, ROW_Y[p] + (ROW_H - 8) / 2);
    display.print(paramLabel(p));
    display.setTextSize(2);
    formatParamValue(p, buf, sizeof(buf));
    printRightAligned(buf, VALUE_RIGHT_EDGE, VALUE_TEXT_Y[p], 2);
  }
  int16_t cy = ROW_Y[selectedParam] + ROW_H / 2;
  display.fillTriangle(0, cy - 4, 0, cy + 4, 4, cy, SH110X_WHITE);

  // --- Button legend, in the buttons' own top-to-bottom order ---
  display.setTextSize(1);
  display.setCursor(0, LEGEND_Y1);
  display.print("A +    B next    C -");
  display.setCursor(0, LEGEND_Y2);
  display.print("hold B 2s = save");

  if (toastText != NULL) {
    drawToast(toastText);
  }

  display.display();
}

void clearRedrawFlags() {
  needFullRedraw   = false;
  needMarkerRedraw = false;
  needFlagRedraw   = false;
  needGateRedraw   = false;
  for (uint8_t p = 0; p < PARAM_COUNT; p++) {
    needValueRedraw[p] = false;
  }
}

// Pushes at most one small region per call, so a single loop pass never blocks
// for more than one transfer. Returns after the first region it manages to send.
void serviceLiveRefresh() {
  if (needGateRedraw && roomBeforeNextPulse(BUDGET_SMALL_US)) {
    drawGateField();
    needGateRedraw = false;
    return;
  }
  for (uint8_t p = 0; p < PARAM_COUNT; p++) {
    if (needValueRedraw[p] && roomBeforeNextPulse(BUDGET_VALUE_US)) {
      drawValueField(p);
      needValueRedraw[p] = false;
      return;
    }
  }
  if (needMarkerRedraw && roomBeforeNextPulse(BUDGET_SMALL_US)) {
    drawSelectionMarker();
    needMarkerRedraw = false;
    return;
  }
  if (needFlagRedraw && roomBeforeNextPulse(BUDGET_SMALL_US)) {
    drawFlagField();
    needFlagRedraw = false;
  }
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
  // 400 kHz I2C: roughly four times faster than the default, and the reason
  // the partial refreshes are short enough to slot between pulses.
  Wire.setClock(400000);
  display.setRotation(1);       // landscape, buttons on the left edge
  display.setTextWrap(false);   // long strings clip instead of wrapping
  display.clearDisplay();       // drop the library's splash screen

  loadSettings();
  drawFullScreen();
  clearRedrawFlags();
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
    needMarkerRedraw = true;
    needFullRedraw   = true;
  }
  // Hold B for 2 s to commit the current settings to flash.
  if (btnSel.heldLong) {
    if (!settingsUnsaved()) {
      showToast("ALREADY SAVED");
    } else if (trainRunning) {
      // Never erase flash mid-train; queue it for the end of the trial. The
      // top-left field switches to "queued" as immediate feedback, which is
      // far cheaper to draw than a toast box.
      savePending = true;
      needFlagRedraw = true;
    } else {
      saveSettings();
      showToast("SETTINGS SAVED");
    }
  }

  // Both edit buttons act on the initial press and on each auto-repeat, and
  // work whether or not a train is running.
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
    needFullRedraw = true;
  }

  // 6. Refresh the screen. Idle means a full frame is free; mid-train the
  //    partial path keeps each transfer inside the gap between pulses.
  if (!trainRunning) {
    if (needFullRedraw) {
      drawFullScreen();
      clearRedrawFlags();
    }
  } else {
    serviceLiveRefresh();
  }
}
