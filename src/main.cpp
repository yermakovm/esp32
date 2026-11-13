#include <Arduino.h>

// Homework 2.2 — measure the mechanical delay of a relay.
//
// Wiring (board = esp32-s3-devkitc-1):
//   GPIO26 -> relay module IN / transistor base (via resistor) : drives the coil
//   GPIO27 -> relay "dry" NO (normally-open) contact; other contact leg -> GND
//             INPUT_PULLUP provides the pull-up: HIGH = contact open, LOW = closed
//   5V + GND from the board to the relay module.
//
// Method: attachInterrupt(CHANGE) on the contact pin, timestamp the edges with
// micros(), drive the coil, and take the FIRST edge after the drive command as
// the real make/break. Everything that follows within the settle window is
// contact bounce: it is counted and reported, but never used as the delay.

static constexpr uint8_t kRelayPin = 26;   // coil drive
static constexpr uint8_t kContactPin = 27; // dry contact, INPUT_PULLUP

static constexpr uint8_t kSamples = 12;    // >= 10 as required
static constexpr uint32_t kHoldUs = 60000;  // how long the coil stays in each state
static constexpr uint32_t kSettleUs = 15000;  // window used to observe bounce
static constexpr uint32_t kTimeoutUs = 200000;  // no edge at all -> wiring problem
// Coil switching emits an EMI spike on the contact line; a real relay needs
// milliseconds to move, so anything faster than this is noise, not the contact.
static constexpr uint32_t kMinDelayUs = 200;

static volatile uint32_t s_driveUs = 0;  // micros() at the moment the coil was driven

static volatile uint32_t s_firstEdgeUs = 0;  // 0 = no edge captured yet
static volatile uint32_t s_lastEdgeUs = 0;
static volatile uint32_t s_edgeCount = 0;
static volatile uint32_t s_noiseEdges = 0;

IRAM_ATTR void onContactEdge() {
  const uint32_t t = micros();  // shortcut: micros() wraps after ~35 min, a trial takes ~60 ms
  if (t - s_driveUs <= kMinDelayUs) {
    s_noiseEdges = s_noiseEdges + 1;
    return;
  }
  if (s_firstEdgeUs == 0) s_firstEdgeUs = t;
  s_lastEdgeUs = t;
  s_edgeCount = s_edgeCount + 1;
}

struct Stats {
  uint32_t n = 0;
  uint32_t sum = 0;
  uint32_t min = UINT32_MAX;
  uint32_t max = 0;

  void add(uint32_t v) {
    n++;
    sum += v;
    if (v < min) min = v;
    if (v > max) max = v;
  }

  void print(const char *label) const {
    if (!n) {
      Serial.printf("%s: no samples\n", label);
      return;
    }
    Serial.printf("%s: avg=%.0f us  min=%lu us  max=%lu us  (n=%lu)\n", label,
                  static_cast<double>(sum) / n, (unsigned long)min, (unsigned long)max,
                  (unsigned long)n);
  }
};

// Drive the coil to `on`, wait for the first contact edge, then keep watching
// kSettleUs longer to catch bounce. Returns the delay in us, -1 on timeout.
static int32_t measure(bool on, uint32_t &chatterUs, uint32_t &edges) {
  s_firstEdgeUs = 0;
  s_lastEdgeUs = 0;
  s_edgeCount = 0;
  s_noiseEdges = 0;

  const uint32_t t0 = micros();
  s_driveUs = t0;
  attachInterrupt(digitalPinToInterrupt(kContactPin), onContactEdge, CHANGE);
  digitalWrite(kRelayPin, on ? HIGH : LOW);

  int32_t delayUs = -1;
  while (micros() - t0 < kTimeoutUs) {
    const uint32_t first = s_firstEdgeUs;
    if (first != 0 && micros() - first >= kSettleUs) {
      delayUs = static_cast<int32_t>(first - t0);
      break;
    }
  }
  detachInterrupt(digitalPinToInterrupt(kContactPin));

  edges = s_edgeCount;
  chatterUs = (edges > 1) ? s_lastEdgeUs - s_firstEdgeUs : 0;

  if (s_noiseEdges) {
    Serial.printf("  (%lu edge(s) ignored as drive noise)\n", (unsigned long)s_noiseEdges);
  }

  // Keep the coil in this state for the rest of the dwell time.
  const uint32_t elapsed = micros() - t0;
  if (elapsed < kHoldUs) delayMicroseconds(kHoldUs - elapsed);

  return delayUs;
}

void setup() {
  Serial.begin(115200);
  pinMode(kRelayPin, OUTPUT);
  pinMode(kContactPin, INPUT_PULLUP);
  digitalWrite(kRelayPin, LOW);
  delay(1000);

  Serial.printf("Relay delay test: pin %u -> coil, pin %u <- contact (pull-up: LOW = closed)\n",
                kRelayPin, kContactPin);
  Serial.println("#  on_delay_us  off_delay_us  on_edges  off_edges  on_bounce_us  off_bounce_us");

  Stats onStats, offStats;

  for (uint8_t i = 1; i <= kSamples; i++) {
    uint32_t onBounceUs = 0, onEdges = 0, offBounceUs = 0, offEdges = 0;

    const int32_t onDelay = measure(true, onBounceUs, onEdges);
    const int32_t offDelay = measure(false, offBounceUs, offEdges);

    Serial.printf("%2u  %10ld  %12ld  %8lu  %9lu  %12lu  %13lu\n", i, (long)onDelay,
                  (long)offDelay, (unsigned long)onEdges, (unsigned long)offEdges,
                  (unsigned long)onBounceUs, (unsigned long)offBounceUs);

    if (onDelay < 0) Serial.println("  ON timeout: check coil drive / contact wiring");
    if (offDelay < 0) Serial.println("  OFF timeout: check contact wiring");
    if (onDelay >= 0) onStats.add(onDelay);
    if (offDelay >= 0) offStats.add(offDelay);
  }

  onStats.print("turn-ON delay");
  offStats.print("turn-OFF delay");
}

void loop() {}
