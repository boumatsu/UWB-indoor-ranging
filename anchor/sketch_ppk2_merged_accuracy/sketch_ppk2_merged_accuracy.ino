/*
 * ESP32 UWB Power Profiling + Solo Ranging Anchor
 * 
 * Combines power profiling capabilities with sophisticated ranging test control.
 * 
 * Power Tests: IDLE_TEST, RX_TEST, TX_TEST, SLEEP_CYCLE, AUTO_SEQUENCE
 * Ranging Test: RANGING_TEST with precise GPIO timing markers
 * 
 * Hardware: ESP32 + DWM1000 UWB module + SH1106 OLED display
 * 
 * GPIO Functionality:
 * - Pin 25: PPK2 TX marker / POLL/RANGE received (50μs pulses)
 * - Pin 26: PPK2 RX marker / Range calculation complete (2ms pulses)
 */

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <Wire.h>
#include <U8g2lib.h>

// ===== HARDWARE PIN DEFINITIONS =====
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
const uint8_t PIN_RST = 27, PIN_IRQ = 34, PIN_SS = 17;
#define I2C_SDA 22
#define I2C_SCL 21
const uint8_t BUTTON_PIN = 13;

// ===== GPIO PINS FOR POWER ANALYSIS =====
#define PPK2_TX_PIN 25  // GPIO for marking TX events / POLL/RANGE received
#define PPK2_RX_PIN 26  // GPIO for marking RX events / Range calculation complete

// ===== LOOKUP TABLES =====
const uint8_t channelMap[] = {0, 1, 2, 3, 4, 5, 7}; // index 0 unused, maps configOption to channel
const uint16_t antennaDelays[] = {16534, 16525, 16517, 16531, 16383, 16534, 16534, 16401}; // indexed by channel

// ===== DEVICE CONFIGURATION =====
char anchor_addr[] = "84:00:5B:D5:A9:9A:E2:9C";
U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
uint8_t configOption = 1;
uint8_t channel = 1;
uint8_t pulseFrequency = DW1000.TX_PULSE_FREQ_16MHZ;
uint8_t dataRate = DW1000.TRX_RATE_6800KBPS;
uint8_t preambleLength = DW1000.TX_PREAMBLE_LEN_128;

// ===== TEST STATE MANAGEMENT =====
enum TestMode {
  MODE_NONE,
  MODE_IDLE_TEST,
  MODE_RX_TEST,
  MODE_TX_TEST,
  MODE_RANGING_TEST,
  MODE_SLEEP_CYCLE,
  MODE_AUTO_SEQUENCE
};

volatile TestMode currentTestMode = MODE_NONE;
bool usingRangingLayer = false;

// ===== RANGING TEST STATE (from solo) =====
enum AnchorState { STATE_IDLE, STATE_ACTIVE };
volatile AnchorState currentState = STATE_IDLE;

// ===== POWER TEST VARIABLES =====
// Sleep Test Variables
unsigned long sleepStartTime = 0, sleepDuration = 0;
bool inDeepSleep = false;

// TX Test Variables
unsigned long lastTxTime = 0, txInterval = 50;  // Default 50ms
volatile bool txInProgress = false;

// ===== AUTO SEQUENCE TEST VARIABLES =====
enum SequenceStep {
  SEQ_RX_1,      // RX_TEST for 2sec
  SEQ_IDLE_1,    // IDLE_TEST for 4sec
  SEQ_TX,        // TX_TEST for 3sec
  SEQ_IDLE_2,    // IDLE_TEST for 2sec
  SEQ_RX_2,      // RX_TEST for 4sec
  SEQ_IDLE_3,    // IDLE_TEST for 2sec
  SEQ_SLEEP,     // SLEEP_CYCLE
  SEQ_COMPLETE   // Sequence finished
};

volatile SequenceStep currentSequenceStep = SEQ_RX_1;
unsigned long sequenceStepStartTime = 0;
bool autoSequenceActive = false;

// ===== RANGING TEST VARIABLES (from solo) =====
int rangingTestTargetCount = 0, rangingTestCurrentCount = 0;
bool rangingTestActive = false;
int pin25PulseCount = 0, pin26PulseCount = 0;

// Ranging test sequence timing
unsigned long rangingSequenceStartTime = 0;
bool rangingIdlePeriodActive = false;
const unsigned long RANGING_IDLE_DURATION = 1000; // 1 second idle before ranging

// ===== BUTTON AND DISPLAY =====
uint8_t lastButtonState = HIGH;
unsigned long lastDebounceTime = 0, lastOledUpdate = 0;
const unsigned long debounceDelay = 300, oledInterval = 200;

// ===== POWER PROFILING HANDLERS =====
void handleSentPowerProfile() {
  digitalWrite(PPK2_TX_PIN, LOW);
  txInProgress = false;
}

void handleReceivedPowerProfile() {
  if (currentTestMode == MODE_RX_TEST) {
    DW1000.newReceive();
    DW1000.startReceive();
  }
}

void handleReceiveTimeoutPowerProfile() {
  if (currentTestMode == MODE_RX_TEST) {
    DW1000.newReceive();
    DW1000.startReceive();
  }
}

void handleReceiveFailedPowerProfile() {
  if (currentTestMode == MODE_RX_TEST) {
    DW1000.newReceive();
    DW1000.startReceive();
  }
}

// ===== SOLO RANGING HANDLERS =====
void handleTimestampAvailable() {
  if (currentTestMode == MODE_RANGING_TEST && currentState == STATE_ACTIVE && rangingTestActive) {
    digitalWrite(PPK2_TX_PIN, HIGH);
    delayMicroseconds(50);  // 50μs pulse duration
    digitalWrite(PPK2_TX_PIN, LOW);
    pin25PulseCount++;
  }
}

void newRange() {
  DW1000Device* device = DW1000Ranging.getDistantDevice();
  if (!device) return;

  float range = device->getRange();
  if (currentTestMode == MODE_RANGING_TEST && currentState == STATE_ACTIVE && rangingTestActive) {
    pin26PulseCount++;
    rangingTestCurrentCount++;

    // Generate Pin 26 pulse for range completion
    digitalWrite(PPK2_RX_PIN, HIGH);
    delayMicroseconds(2000);  // 2ms duration
    digitalWrite(PPK2_RX_PIN, LOW);

    // Output comprehensive ranging data
    Serial.printf("#%d:%.2fm|P25:%d|P26:%d|T:%lu|TS:%lld,%lld,%lld,%lld,%lld,%lld\n", 
      rangingTestCurrentCount, range, pin25PulseCount, pin26PulseCount, millis(),
      device->timePollSent.getTimestamp(), device->timePollReceived.getTimestamp(),
      device->timePollAckSent.getTimestamp(), device->timePollAckReceived.getTimestamp(),
      device->timeRangeSent.getTimestamp(), device->timeRangeReceived.getTimestamp());

    if (rangingTestCurrentCount >= rangingTestTargetCount) {
      Serial.printf("TEST_COMPLETE|Total:P25=%d,P26=%d,Time=%lums\n",
        pin25PulseCount, pin26PulseCount, millis());
      stopRangingTest(); // This will enter indefinite idle mode
    }
  }
}

void newDevice(DW1000Device* device) {
  if (device) Serial.printf("DEV_ADD:%X|T:%lums\n", device->getShortAddress(), millis());
}

void inactiveDevice(DW1000Device* device) {
  if (device) Serial.printf("DEV_LOST:%X|T:%lums\n", device->getShortAddress(), millis());
}

// ===== SYSTEM INITIALIZATION =====
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize GPIOs
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PPK2_TX_PIN, OUTPUT);
  pinMode(PPK2_RX_PIN, OUTPUT);
  digitalWrite(PPK2_TX_PIN, LOW);
  digitalWrite(PPK2_RX_PIN, LOW);

  // Control unused pins to avoid interference
  pinMode(32, OUTPUT);
  digitalWrite(32, LOW);
  pinMode(33, OUTPUT);
  digitalWrite(33, LOW);

  // Initialize I2C and OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin();
  display.setFont(u8g2_font_6x12_tf);
  updateOLED("Power + Ranging", "Initializing...");

  // Initialize SPI
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Apply initial configuration
  applyChannelConfiguration();

  Serial.println("\n=== UWB Power Profiling + Solo Ranging ===");
  Serial.println("Power Tests:");
  Serial.println("  IDLE_TEST           - Measure idle state power");
  Serial.println("  RX_TEST             - Measure receiver power");
  Serial.println("  TX_TEST [delay_ms]  - Measure periodic TX power");
  Serial.println("  SLEEP_CYCLE <ms>    - Measure deep sleep power");
  Serial.println("  auto                - Automated power sequence");
  Serial.println("Solo Ranging:");
  Serial.println("  RANGING_TEST <count> - Controlled ranging with GPIO markers");
  Serial.println("  STOP_TEST           - Stop current test");
  Serial.println("  Button: Change channel (1,2,3,4,5,7)");

  updateOLED("Ready", ("CH: " + String(channel)).c_str());
}

// ===== MAIN PROGRAM LOOP =====
void loop() {
  handleButtonPress();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    parseCommand(cmd);
  }

  // Main test state machine
  switch (currentTestMode) {
    case MODE_NONE: if (usingRangingLayer) DW1000Ranging.loop(); break;
    case MODE_TX_TEST: handleTxTest(); break;
    case MODE_RANGING_TEST: handleRangingTest(); break;
    case MODE_SLEEP_CYCLE: handleSleepTest(); break;
    case MODE_AUTO_SEQUENCE: handleAutoSequence(); break;
  }

  // Update OLED periodically
  if (millis() - lastOledUpdate > oledInterval) {
    updateOledDisplay();
  }
}

// ===== USER INTERFACE =====
void handleButtonPress() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading == LOW && lastButtonState == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
    if (currentTestMode == MODE_NONE) {
      configOption++;
      if (configOption > 6) configOption = 1;
      applyChannelConfiguration();
    }
    lastDebounceTime = millis();
  }
  lastButtonState = reading;
}

void parseCommand(String cmd) {
  String command;
  int arg1 = 0;

  // Parse command and arguments
  int firstSpace = cmd.indexOf(' ');
  if (firstSpace != -1) {
    command = cmd.substring(0, firstSpace);
    arg1 = cmd.substring(firstSpace + 1).toInt();
  } else {
    command = cmd;
  }

  // Stop any current test first
  stopAllTests();

  // Execute new command
  if (command == "IDLE_TEST") startIdleTest();
  else if (command == "RX_TEST") startRxTest();
  else if (command == "TX_TEST") { txInterval = (arg1 > 0) ? arg1 : 50; startTxTest(); }
  else if (command == "RANGING_TEST") { if (arg1 > 0) startRangingTest(arg1); else Serial.println("Usage: RANGING_TEST <count>"); }
  else if (command == "SLEEP_CYCLE") { sleepDuration = (arg1 > 0) ? arg1 : 5000; startSleepTest(); }
  else if (command == "auto") startAutoSequence();
  else if (command == "STOP_TEST") Serial.println("All tests stopped.");
  else Serial.println("Unknown command.");
}

// ===== POWER TEST IMPLEMENTATIONS =====
void startIdleTest() {
  Serial.println("IDLE_TEST: Starting - measuring idle power");
  initializeDirectMode();
  DW1000.idle();
  DW1000.clearInterrupts();
  DW1000.receivePermanently(false);
  currentTestMode = MODE_IDLE_TEST;
  Serial.println("IDLE_TEST: Active - minimal power mode");
}

void startRxTest() {
  Serial.println("RX_TEST: Starting - measuring RX power");
  initializeDirectMode();
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(false);
  DW1000.startReceive();
  digitalWrite(PPK2_RX_PIN, HIGH);
  currentTestMode = MODE_RX_TEST;
  Serial.println("RX_TEST: Active - listening continuously");
}

void startTxTest() {
  Serial.printf("TX_TEST: Active - TX every %dms\n", txInterval);
  initializeDirectMode();
  DW1000.idle();
  lastTxTime = millis();
  currentTestMode = MODE_TX_TEST;
}

void startSleepTest() {
  Serial.printf("SLEEP_CYCLE: Starting for %lums - entering deep sleep\n", sleepDuration);
  stopAllTests();
  DW1000.clearInterrupts();
  DW1000.idle();
  sleepStartTime = millis();
  Serial.flush();
  updateOLED("SLEEP_CYCLE", "Sleeping...");
  delay(100);
  DW1000.deepSleep();
  inDeepSleep = true;
  currentTestMode = MODE_SLEEP_CYCLE;
  Serial.println("SLEEP_CYCLE: Active - measuring ultra-low power");
}

void startAutoSequence() {
  Serial.printf("AUTO: Starting power sequence on CH%d\n", channel);
  currentSequenceStep = SEQ_RX_1;
  sequenceStepStartTime = millis();
  autoSequenceActive = true;
  currentTestMode = MODE_AUTO_SEQUENCE;
  
  // Start first step: RX_TEST for 2sec
  Serial.println("AUTO: Step 1/7 - RX_TEST (2sec)");
  initializeDirectMode();
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(false);
  DW1000.startReceive();
  digitalWrite(PPK2_RX_PIN, HIGH);
}

// ===== AUTO SEQUENCE HANDLER =====
void handleAutoSequence() {
  if (!autoSequenceActive) return;
  
  unsigned long elapsed = millis() - sequenceStepStartTime;
  
  switch (currentSequenceStep) {
    case SEQ_RX_1:
      if (elapsed >= 2000) { // 2 seconds
        digitalWrite(PPK2_RX_PIN, LOW);
        Serial.println("AUTO: Step 2/7 - IDLE_TEST (4sec)");
        DW1000.idle();
        DW1000.clearInterrupts();
        DW1000.receivePermanently(false);
        currentSequenceStep = SEQ_IDLE_1;
        sequenceStepStartTime = millis();
      }
      break;
      
    case SEQ_IDLE_1:
      if (elapsed >= 4000) { // 4 seconds
        Serial.println("AUTO: Step 3/7 - TX_TEST (3sec, 500ms interval)");
        DW1000.idle();
        txInterval = 500;
        lastTxTime = millis();
        currentSequenceStep = SEQ_TX;
        sequenceStepStartTime = millis();
      }
      break;
      
    case SEQ_TX:
      // Handle TX test logic
      if (millis() - lastTxTime >= txInterval && !txInProgress) {
        digitalWrite(PPK2_TX_PIN, HIGH);
        byte testData[100] = "PWR_PROFILE_TX_EXTENDED_MESSAGE_FOR_LONGER_TRANSMISSION_TIME_AND_BETTER_POWER_MEASUREMENT_100B";
        DW1000.newTransmit();
        DW1000.setDefaults();
        DW1000.setData(testData, sizeof(testData));
        txInProgress = true;
        DW1000.startTransmit();
        lastTxTime = millis();
      }
      if (!txInProgress && millis() - lastTxTime > 10) {
        DW1000.idle();
      }
      
      if (elapsed >= 3000) { // 3 seconds
        Serial.println("AUTO: Step 4/7 - IDLE_TEST (2sec)");
        DW1000.idle();
        DW1000.clearInterrupts();
        DW1000.receivePermanently(false);
        currentSequenceStep = SEQ_IDLE_2;
        sequenceStepStartTime = millis();
      }
      break;
      
    case SEQ_IDLE_2:
      if (elapsed >= 2000) { // 2 seconds
        Serial.println("AUTO: Step 5/7 - RX_TEST (4sec)");
        DW1000.newReceive();
        DW1000.setDefaults();
        DW1000.receivePermanently(false);
        DW1000.startReceive();
        digitalWrite(PPK2_RX_PIN, HIGH);
        currentSequenceStep = SEQ_RX_2;
        sequenceStepStartTime = millis();
      }
      break;
      
    case SEQ_RX_2:
      if (elapsed >= 4000) { // 4 seconds
        digitalWrite(PPK2_RX_PIN, LOW);
        Serial.println("AUTO: Step 6/7 - IDLE_TEST (2sec)");
        DW1000.idle();
        DW1000.clearInterrupts();
        DW1000.receivePermanently(false);
        currentSequenceStep = SEQ_IDLE_3;
        sequenceStepStartTime = millis();
      }
      break;
      
    case SEQ_IDLE_3:
      if (elapsed >= 2000) { // 2 seconds
        Serial.println("AUTO: Step 7/7 - SLEEP_CYCLE (3000ms)");
        DW1000.clearInterrupts();
        DW1000.idle();
        sleepStartTime = millis();
        sleepDuration = 3000;
        Serial.flush();
        delay(100);
        DW1000.deepSleep();
        inDeepSleep = true;
        currentSequenceStep = SEQ_SLEEP;
        sequenceStepStartTime = millis();
      }
      break;
      
    case SEQ_SLEEP:
      if (inDeepSleep) {
        if (millis() - sleepStartTime >= sleepDuration) {
          Serial.println("AUTO: Sleep complete - reinitializing");
          DW1000.spiWakeup();
          delay(10);
          applyChannelConfiguration();
          inDeepSleep = false;
          currentSequenceStep = SEQ_COMPLETE;
        }
      }
      break;
      
    case SEQ_COMPLETE:
      Serial.println("AUTO: Sequence complete");
      autoSequenceActive = false;
      currentTestMode = MODE_NONE;
      break;
  }
}

// ===== SOLO RANGING TEST IMPLEMENTATION =====
void startRangingTest(int count) {
  Serial.printf("RANGING_TEST: Starting %d cycles - idle phase\n", count);
  
  // Initialize direct mode for idle period
  initializeDirectMode();
  
  // Enter true idle state for 1 second
  DW1000.idle();
  DW1000.clearInterrupts();
  DW1000.receivePermanently(false);

  // Initialize ranging test parameters
  rangingTestTargetCount = count;
  rangingTestCurrentCount = 0;
  pin25PulseCount = 0;
  pin26PulseCount = 0;

  // Start the sequence with idle period
  rangingSequenceStartTime = millis();
  rangingIdlePeriodActive = true;
  rangingTestActive = false; // Not yet in active ranging
  currentTestMode = MODE_RANGING_TEST;
  currentState = STATE_IDLE;
}

void stopRangingTest() {
  // Complete the ranging test and enter indefinite idle mode
  rangingTestActive = false;
  rangingIdlePeriodActive = false;
  currentState = STATE_IDLE;
  currentTestMode = MODE_IDLE_TEST; // Enter indefinite idle mode
  
  Serial.printf("STOP_TEST|T:%lums\n", millis());

  // Generate end-of-test sync pulses
  digitalWrite(PPK2_TX_PIN, HIGH);
  digitalWrite(PPK2_RX_PIN, HIGH);
  delay(20);
  digitalWrite(PPK2_TX_PIN, LOW);
  digitalWrite(PPK2_RX_PIN, LOW);
  delay(20);
  
  // Enter true idle state
  initializeDirectMode();
  DW1000.idle();
  DW1000.clearInterrupts();
  DW1000.receivePermanently(false);
  
  Serial.println("RANGING_TEST: Complete - entered idle mode");
}

// ===== TEST HANDLERS =====
void handleTxTest() {
  if (millis() - lastTxTime >= txInterval && !txInProgress) {
    digitalWrite(PPK2_TX_PIN, HIGH);
    byte testData[100] = "PWR_PROFILE_TX_EXTENDED_MESSAGE_FOR_LONGER_TRANSMISSION_TIME_AND_BETTER_POWER_MEASUREMENT_100B";
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData(testData, sizeof(testData));
    txInProgress = true;
    DW1000.startTransmit();
    lastTxTime = millis();
  }
  if (!txInProgress && millis() - lastTxTime > 10) {
    DW1000.idle();
  }
}

void handleRangingTest() {
  // Check if we're in the initial idle period
  if (rangingIdlePeriodActive) {
    if (millis() - rangingSequenceStartTime >= RANGING_IDLE_DURATION) {
      // Idle period complete - transition to active ranging
      Serial.println("RANGING: Active start");
      
      // Initialize ranging mode
      initializeRangingMode();
      
      // Generate start-of-ranging sync pulses (20ms duration)
      digitalWrite(PPK2_TX_PIN, HIGH);
      digitalWrite(PPK2_RX_PIN, HIGH);
      delay(20);
      digitalWrite(PPK2_TX_PIN, LOW);
      digitalWrite(PPK2_RX_PIN, LOW);
      delay(20);

      Serial.printf("START_TEST|Target:%d|T:%lums\n", rangingTestTargetCount, millis());

      // Activate ranging
      rangingIdlePeriodActive = false;
      rangingTestActive = true;
      currentState = STATE_ACTIVE;
      
      Serial.println("RANGING_TEST: Active - waiting for tag");
    }
    // During idle period, do nothing (stay in idle)
    return;
  }
  
  // Active ranging period - let ranging protocol handle everything
  if (currentState == STATE_ACTIVE && rangingTestActive) {
    DW1000Ranging.loop();
  }
}

void handleSleepTest() {
  if (inDeepSleep) {
    if (millis() - sleepStartTime >= sleepDuration) {
      Serial.println("SLEEP_CYCLE: Complete - chip reinitialized");
      DW1000.spiWakeup();
      delay(10);
      applyChannelConfiguration();
      inDeepSleep = false;
      currentTestMode = MODE_NONE;
    }
  } else {
    if (millis() - sleepStartTime >= sleepDuration) {
      currentTestMode = MODE_NONE;
      Serial.println("SLEEP_CYCLE: Complete");
    }
  }
}

void stopAllTests() {
  currentTestMode = MODE_NONE;
  currentState = STATE_IDLE;
  inDeepSleep = false;
  txInProgress = false;
  rangingTestActive = false;
  rangingIdlePeriodActive = false; // Clear ranging sequence state
  autoSequenceActive = false; // Clear auto sequence state
  digitalWrite(PPK2_TX_PIN, LOW);
  digitalWrite(PPK2_RX_PIN, LOW);

  if (usingRangingLayer) {
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
  } else {
    initializeRangingMode();
  }
}

// ===== MODE INITIALIZATION =====
void initializeDirectMode() {
  if (usingRangingLayer) {
    Serial.println("MODE: Direct control initialized");
    DW1000.reset();
    delay(5);
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    
    // Attach power profiling handlers
    DW1000.attachSentHandler(handleSentPowerProfile);
    DW1000.attachReceivedHandler(handleReceivedPowerProfile);
    DW1000.attachReceiveTimeoutHandler(handleReceiveTimeoutPowerProfile);
    DW1000.attachReceiveFailedHandler(handleReceiveFailedPowerProfile);

    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setChannel(channel);
    uint16_t antennaDelay = getAntennaDelayForChannel(channel);
    DW1000.setAntennaDelay(antennaDelay);
    byte customMode[3] = { DW1000.TRX_RATE_6800KBPS, DW1000.TX_PULSE_FREQ_16MHZ, DW1000.TX_PREAMBLE_LEN_128 };
    DW1000.enableMode(customMode);
    DW1000.commitConfiguration();
    usingRangingLayer = false;
  }
}

void initializeRangingMode() {
  if (!usingRangingLayer) {
    Serial.println("MODE: Ranging protocol initialized");
    applyChannelConfiguration();
    usingRangingLayer = true;
  }
}

void applyChannelConfiguration() {
  // OPTIMIZED: Use lookup table instead of switch statement
  channel = (configOption <= 6) ? channelMap[configOption] : 5;

  // Full reinitialization
  SPI.end();
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000.reset();
  delay(5);

  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  uint16_t antennaDelay = getAntennaDelayForChannel(channel);
  DW1000.setAntennaDelay(antennaDelay);

  // Configure for ranging mode
  pulseFrequency = DW1000.TX_PULSE_FREQ_16MHZ;
  dataRate = DW1000.TRX_RATE_6800KBPS;
  preambleLength = DW1000.TX_PREAMBLE_LEN_128;
  byte customMode[3] = { dataRate, pulseFrequency, preambleLength };

  // Attach ranging handlers
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  DW1000Ranging.startAsAnchor(anchor_addr, customMode, false);
  DW1000.setChannel(channel);
  DW1000.commitConfiguration();
  
  // Attach solo ranging timestamp handler
  DW1000.attachReceiveTimestampAvailableHandler(handleTimestampAvailable);

  usingRangingLayer = true;
  Serial.printf("Channel %d configured with antenna delay %d\n", channel, antennaDelay);
}

// OPTIMIZED: Use lookup table instead of switch statement
uint16_t getAntennaDelayForChannel(uint8_t ch) {
  return (ch <= 7) ? antennaDelays[ch] : 16534;
}

// ===== DISPLAY MANAGEMENT =====
void updateOledDisplay() {
  lastOledUpdate = millis();
  char line2[20];
  
  switch (currentTestMode) {
    case MODE_NONE: snprintf(line2, 20, "CH: %d", channel); updateOLED("Ready", line2); break;
    case MODE_IDLE_TEST: updateOLED("IDLE_TEST", "Min Power Mode"); break;
    case MODE_RX_TEST: updateOLED("RX_TEST", "RX Active"); break;
    case MODE_TX_TEST: snprintf(line2, 20, "TX Every %lums", txInterval); updateOLED("TX_TEST", line2); break;
    case MODE_RANGING_TEST: snprintf(line2, 20, "%d/%d ranges", rangingTestCurrentCount, rangingTestTargetCount); updateOLED("RANGING_TEST", line2); break;
    case MODE_SLEEP_CYCLE: updateOLED("SLEEP_CYCLE", inDeepSleep ? "Deep Sleep" : "Complete"); break;
    case MODE_AUTO_SEQUENCE: snprintf(line2, 20, "CH: %d AUTO", channel); updateOLED("AUTO_SEQ", line2); break;
  }
}

void updateOLED(const char* line1, const char* line2) {
  display.firstPage();
  do {
    display.setCursor(0, 12);
    display.print(line1);
    display.setCursor(0, 24);
    display.print(line2);
  } while (display.nextPage());
}