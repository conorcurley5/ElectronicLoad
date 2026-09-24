#include <Arduino.h>
#include <SPI.h>

#include <cmath>
#include <cstring>

namespace Pins {
constexpr uint8_t kFaultN = 8;
constexpr uint8_t kDacCs = 9;
constexpr uint8_t kAdcCs = 10;
constexpr uint8_t kMosi = 11;
constexpr uint8_t kSclk = 12;
constexpr uint8_t kMiso = 13;
}  // namespace Pins

namespace Limits {
// Deliberately low for initial board bring-up. Raise only after validating the
// control-loop waveform, heatsink, current calibration, and fault behaviour.
constexpr float kMaximumTestCurrentA = 0.250F;
constexpr float kRampStepA = 0.010F;
constexpr uint32_t kRampStepDelayMs = 20;
constexpr uint16_t kThermistorOpenCode = 32600;
constexpr float kSoftwareTemperatureLimitC = 75.0F;
}  // namespace Limits

namespace Scaling {
constexpr float kDacFullScaleV = 2.5F;
constexpr float kAdcInternalReferenceV = 2.048F;
constexpr float kDutDividerRatio = 13.7F;
constexpr float kCurrentSenseVoltsPerAmp = 1.0F;
constexpr float kThermistorFixedOhms = 10000.0F;
constexpr float kThermistorNominalOhms = 10000.0F;
constexpr float kThermistorNominalK = 298.15F;
constexpr float kThermistorBetaK = 3600.0F;
}  // namespace Scaling

namespace Ads1120 {
constexpr uint8_t kCommandReset = 0x06;
constexpr uint8_t kCommandStart = 0x08;
constexpr uint8_t kCommandReadData = 0x10;
constexpr uint8_t kCommandReadRegisters = 0x20;
constexpr uint8_t kCommandWriteRegisters = 0x40;

constexpr uint8_t kMuxAin0Avss = 0x08;
constexpr uint8_t kMuxAin1Avss = 0x09;
constexpr uint8_t kMuxAin2Avss = 0x0A;
constexpr uint8_t kPgaBypass = 0x01;
constexpr uint8_t kInternalReference = 0x00;
constexpr uint8_t kSupplyReference = 0xC0;

// Normal mode, single shot, 20 SPS, temperature sensor and burn-out sources off.
constexpr uint8_t kConfig1 = 0x00;
constexpr uint8_t kConfig3 = 0x00;
constexpr uint32_t kConversionTimeMs = 60;
}  // namespace Ads1120

namespace Dac80501 {
constexpr uint8_t kRegisterSync = 0x02;
constexpr uint8_t kRegisterConfig = 0x03;
constexpr uint8_t kRegisterGain = 0x04;
constexpr uint8_t kRegisterTrigger = 0x05;
constexpr uint8_t kRegisterDac = 0x08;
constexpr uint16_t kSoftReset = 0x000A;
constexpr uint16_t kReferenceAndDacPowerDown = 0x0101;
constexpr uint16_t kRefDivideBy2GainBy2 = 0x0101;
}  // namespace Dac80501

// Both parts clock data on SCLK's falling edge. Start slowly for jumper-wire
// bring-up; speed can be increased after signal-integrity checks.
SPISettings adcSpiSettings(1000000, MSBFIRST, SPI_MODE1);
SPISettings dacSpiSettings(1000000, MSBFIRST, SPI_MODE1);

struct Measurements {
  int16_t rawVoltage = 0;
  int16_t rawCurrent = 0;
  int16_t rawThermistor = 0;
  float dutVoltageV = NAN;
  float currentA = NAN;
  float temperatureC = NAN;
  bool thermistorValid = false;
};

bool adcHealthy = false;
bool faultLatched = true;
float commandedCurrentA = 0.0F;
const char* faultReason = "startup";
uint32_t lastTelemetryMs = 0;
char commandBuffer[64] = {};
size_t commandLength = 0;

bool hardwareFaultActive() {
  return digitalRead(Pins::kFaultN) == LOW;
}

void dacWriteRegister(uint8_t address, uint16_t value) {
  SPI.beginTransaction(dacSpiSettings);
  digitalWrite(Pins::kDacCs, LOW);
  SPI.transfer(address & 0x0F);
  SPI.transfer(static_cast<uint8_t>(value >> 8));
  SPI.transfer(static_cast<uint8_t>(value));
  digitalWrite(Pins::kDacCs, HIGH);
  SPI.endTransaction();
  // Explicit margin for SYNC high time and sequential DAC update timing.
  delayMicroseconds(2);
}

void dacWriteCode(uint16_t code) {
  dacWriteRegister(Dac80501::kRegisterDac, code);
}

uint16_t currentToDacCode(float currentA) {
  // R7/R8 divide VDAC by ten, while the 0.1-ohm shunt converts current
  // to 0.1 V/A. Consequently, ideal load current in amps equals VDAC.
  const float normalized = currentA / Scaling::kDacFullScaleV;
  const float code = normalized * 65535.0F;
  if (code <= 0.0F) {
    return 0;
  }
  if (code >= 65535.0F) {
    return 65535;
  }
  return static_cast<uint16_t>(lroundf(code));
}

void forceLoadOff() {
  dacWriteCode(0);
  commandedCurrentA = 0.0F;
}

void latchFault(const char* reason) {
  forceLoadOff();
  if (!faultLatched || std::strcmp(faultReason, reason) != 0) {
    Serial.print(F("FAULT LATCHED: "));
    Serial.println(reason);
  }
  faultLatched = true;
  faultReason = reason;
}

void adcCommand(uint8_t command) {
  SPI.beginTransaction(adcSpiSettings);
  digitalWrite(Pins::kAdcCs, LOW);
  SPI.transfer(command);
  digitalWrite(Pins::kAdcCs, HIGH);
  SPI.endTransaction();
}

void adcWriteRegisters(uint8_t startAddress, const uint8_t* values,
                       size_t count) {
  if (count == 0 || count > 4 || startAddress > 3) {
    return;
  }

  const uint8_t command = Ads1120::kCommandWriteRegisters |
                          ((startAddress & 0x03) << 2) |
                          ((count - 1) & 0x03);
  SPI.beginTransaction(adcSpiSettings);
  digitalWrite(Pins::kAdcCs, LOW);
  SPI.transfer(command);
  for (size_t index = 0; index < count; ++index) {
    SPI.transfer(values[index]);
  }
  digitalWrite(Pins::kAdcCs, HIGH);
  SPI.endTransaction();
}

void adcReadRegisters(uint8_t startAddress, uint8_t* values, size_t count) {
  if (count == 0 || count > 4 || startAddress > 3) {
    return;
  }

  const uint8_t command = Ads1120::kCommandReadRegisters |
                          ((startAddress & 0x03) << 2) |
                          ((count - 1) & 0x03);
  SPI.beginTransaction(adcSpiSettings);
  digitalWrite(Pins::kAdcCs, LOW);
  SPI.transfer(command);
  for (size_t index = 0; index < count; ++index) {
    values[index] = SPI.transfer(0x00);
  }
  digitalWrite(Pins::kAdcCs, HIGH);
  SPI.endTransaction();
}

int16_t adcReadLatestData() {
  SPI.beginTransaction(adcSpiSettings);
  digitalWrite(Pins::kAdcCs, LOW);
  SPI.transfer(Ads1120::kCommandReadData);
  const uint16_t result = (static_cast<uint16_t>(SPI.transfer(0x00)) << 8) |
                          SPI.transfer(0x00);
  digitalWrite(Pins::kAdcCs, HIGH);
  SPI.endTransaction();
  return static_cast<int16_t>(result);
}

bool waitForAdcConversion() {
  const uint32_t startMs = millis();
  while (millis() - startMs < Ads1120::kConversionTimeMs) {
    if (hardwareFaultActive()) {
      latchFault("temperature comparator asserted");
    }
    delay(1);
  }
  return true;
}

int16_t adcReadSingleEnded(uint8_t mux, uint8_t referenceSelection) {
  const uint8_t registers[4] = {
      static_cast<uint8_t>((mux << 4) | Ads1120::kPgaBypass),
      Ads1120::kConfig1,
      referenceSelection,
      Ads1120::kConfig3,
  };
  adcWriteRegisters(0, registers, 4);
  adcCommand(Ads1120::kCommandStart);
  waitForAdcConversion();
  return adcReadLatestData();
}

float positiveCodeToVoltage(int16_t code, float referenceV) {
  if (code <= 0) {
    return 0.0F;
  }
  return static_cast<float>(code) * referenceV / 32768.0F;
}

bool thermistorCodeToCelsius(int16_t code, float& temperatureC) {
  if (code <= 0 || code >= Limits::kThermistorOpenCode) {
    return false;
  }

  // With AVDD used as the ADC reference, the code directly provides the
  // divider ratio and rejects 3.3-V supply tolerance.
  const float ratio = static_cast<float>(code) / 32768.0F;
  if (ratio <= 0.0F || ratio >= 1.0F) {
    return false;
  }

  const float resistance =
      Scaling::kThermistorFixedOhms * ratio / (1.0F - ratio);
  const float inverseKelvin =
      (1.0F / Scaling::kThermistorNominalK) +
      logf(resistance / Scaling::kThermistorNominalOhms) /
          Scaling::kThermistorBetaK;
  temperatureC = (1.0F / inverseKelvin) - 273.15F;
  return std::isfinite(temperatureC);
}

Measurements readMeasurements() {
  Measurements result;
  result.rawVoltage = adcReadSingleEnded(Ads1120::kMuxAin0Avss,
                                         Ads1120::kInternalReference);
  result.rawCurrent = adcReadSingleEnded(Ads1120::kMuxAin1Avss,
                                         Ads1120::kInternalReference);
  result.rawThermistor = adcReadSingleEnded(Ads1120::kMuxAin2Avss,
                                            Ads1120::kSupplyReference);

  const float voltageSenseV = positiveCodeToVoltage(
      result.rawVoltage, Scaling::kAdcInternalReferenceV);
  const float currentSenseV = positiveCodeToVoltage(
      result.rawCurrent, Scaling::kAdcInternalReferenceV);
  result.dutVoltageV = voltageSenseV * Scaling::kDutDividerRatio;
  result.currentA = currentSenseV / Scaling::kCurrentSenseVoltsPerAmp;
  result.thermistorValid =
      thermistorCodeToCelsius(result.rawThermistor, result.temperatureC);
  return result;
}

bool initializeAdc() {
  adcCommand(Ads1120::kCommandReset);
  delay(2);

  const uint8_t expected[4] = {
      static_cast<uint8_t>((Ads1120::kMuxAin0Avss << 4) |
                           Ads1120::kPgaBypass),
      Ads1120::kConfig1,
      Ads1120::kInternalReference,
      Ads1120::kConfig3,
  };
  adcWriteRegisters(0, expected, 4);

  uint8_t actual[4] = {};
  adcReadRegisters(0, actual, 4);
  return std::memcmp(expected, actual, sizeof(expected)) == 0;
}

void initializeDac() {
  dacWriteRegister(Dac80501::kRegisterTrigger, Dac80501::kSoftReset);
  delay(2);  // POR requires 250 us; allow extra settling margin.
  // Load zero before enabling the output or changing its range. The fitted
  // DAC60501Z resets to zero; this sequence also clears stale buffer contents.
  dacWriteCode(0);
  dacWriteRegister(Dac80501::kRegisterSync, 0x0000);
  dacWriteRegister(Dac80501::kRegisterGain,
                   Dac80501::kRefDivideBy2GainBy2);
  dacWriteRegister(Dac80501::kRegisterConfig, 0x0000);
  delay(2);
  dacWriteCode(0);
}

void printMeasurements(const Measurements& measurements) {
  Serial.print(F("Vdut="));
  Serial.print(measurements.dutVoltageV, 4);
  Serial.print(F(" V, I="));
  Serial.print(measurements.currentA, 4);
  Serial.print(F(" A, T="));
  if (measurements.thermistorValid) {
    Serial.print(measurements.temperatureC, 1);
    Serial.print(F(" C"));
  } else {
    Serial.print(F("INVALID/OPEN"));
  }
  Serial.print(F(", FAULT_N="));
  Serial.print(hardwareFaultActive() ? F("LOW") : F("HIGH"));
  Serial.print(F(", command="));
  Serial.print(commandedCurrentA, 3);
  Serial.print(F(" A, state="));
  if (faultLatched) {
    Serial.print(F("LATCHED (") );
    Serial.print(faultReason);
    Serial.println(')');
  } else {
    Serial.println(F("ready"));
  }
}

void printStatus() {
  if (!adcHealthy) {
    Serial.println(F("ADC communication failed; load is locked off."));
    return;
  }

  const Measurements measurements = readMeasurements();
  if (!measurements.thermistorValid) {
    latchFault("thermistor open or invalid");
  } else if (measurements.temperatureC >=
             Limits::kSoftwareTemperatureLimitC) {
    latchFault("software temperature limit reached");
  }
  printMeasurements(measurements);
}

void printHelp() {
  Serial.println();
  Serial.println(F("Electronic load bring-up firmware"));
  Serial.println(F("Startup recovery v3; DAC SPI writes have no readback."));
  Serial.println(F("  status       read voltage, current, temperature and fault"));
  Serial.println(F("  i <amps>     set 0.000 to 0.250 A (slow ramp upward)"));
  Serial.println(F("  off          immediately write zero to the DAC"));
  Serial.println(F("  clear        reinitialize DAC/ADC and check safety; stays off"));
  Serial.println(F("  dacref       DUT disconnected: reference off 5 s, then restore"));
  Serial.println(F("  help         show this list"));
  Serial.println(F("Use a current-limited DUT supply and an oscilloscope."));
  Serial.println();
}

void clearFaultIfSafe();

void applyCurrentCommand(float targetA) {
  if (!std::isfinite(targetA) || targetA < 0.0F ||
      targetA > Limits::kMaximumTestCurrentA) {
    Serial.print(F("Rejected: range is 0 to "));
    Serial.print(Limits::kMaximumTestCurrentA, 3);
    Serial.println(F(" A."));
    return;
  }

  if (targetA == 0.0F) {
    forceLoadOff();
    Serial.println(F("Load command is zero."));
    return;
  }

  if (faultLatched || !adcHealthy || hardwareFaultActive()) {
    forceLoadOff();
    Serial.println(F("Rejected: clear the fault after correcting its cause."));
    return;
  }

  // Recover a DAC that powered up late or lost its configuration. Always
  // restart from zero and recheck the ADC/temperature before ramping. Existing
  // faults above still require an explicit clear command from the operator.
  clearFaultIfSafe();
  if (faultLatched || !adcHealthy || hardwareFaultActive()) {
    forceLoadOff();
    Serial.println(F("Rejected: peripheral recovery/safety check failed."));
    return;
  }

  if (targetA <= commandedCurrentA) {
    dacWriteCode(currentToDacCode(targetA));
    commandedCurrentA = targetA;
  } else {
    while (commandedCurrentA < targetA) {
      if (hardwareFaultActive()) {
        latchFault("temperature comparator asserted during ramp");
        return;
      }

      float nextA = commandedCurrentA + Limits::kRampStepA;
      if (nextA > targetA) {
        nextA = targetA;
      }
      dacWriteCode(currentToDacCode(nextA));
      commandedCurrentA = nextA;
      delay(Limits::kRampStepDelayMs);
    }
  }

  Serial.print(F("Commanded "));
  Serial.print(commandedCurrentA, 3);
  Serial.println(F(" A."));
}

void clearFaultIfSafe() {
  forceLoadOff();
  faultLatched = true;
  faultReason = "peripheral recovery pending";
  initializeDac();
  // Retry even if the ADC was absent at ESP32 startup. The old implementation
  // cached a failed startup check forever, so clear could never recover it.
  adcHealthy = initializeAdc();
  if (!adcHealthy) {
    latchFault("ADS1120 register readback failed");
    Serial.println(F("Cannot clear: ADC communication is unhealthy."));
    return;
  }
  if (hardwareFaultActive()) {
    latchFault("temperature comparator asserted");
    Serial.println(F("Cannot clear: FAULT_N is still low."));
    return;
  }

  // Allow the conversion wait to latch even a short FAULT_N pulse, and do
  // not discard that event just because the pin is high again afterward.
  faultLatched = false;
  const Measurements measurements = readMeasurements();
  if (faultLatched) {
    Serial.println(F("Cannot clear: fault occurred during measurements."));
    return;
  }
  if (hardwareFaultActive()) {
    latchFault("temperature comparator asserted while clearing");
    Serial.println(F("Cannot clear: FAULT_N asserted during checks."));
    return;
  }
  if (!measurements.thermistorValid) {
    latchFault("thermistor open or invalid");
    Serial.println(F("Cannot clear: thermistor is open or invalid."));
    return;
  }
  if (measurements.temperatureC >= Limits::kSoftwareTemperatureLimitC) {
    latchFault("software temperature limit reached");
    Serial.println(F("Cannot clear: measured temperature is too high."));
    return;
  }

  faultLatched = false;
  faultReason = "none";
  Serial.println(F("DAC configuration sent; ADC/safety checks passed. Output commanded zero."));
}

void testDacReference() {
  // No ADC/DAC readback can verify this test: observe VREFIO with a meter.
  // Always leave the load latched off; never resume the previous current.
  latchFault("DAC reference test; run clear before loading");
  initializeDac();
  forceLoadOff();
  Serial.println(F("DACREF: DUT must be disconnected. Measure VREFIO to GND."));
  Serial.println(F("DACREF: reference ON requested; expect about 2.5 V (2 s)."));
  delay(2000);

  // CONFIG bit 8 disables the reference; bit 0 grounds VOUT via 1 kohm.
  dacWriteRegister(Dac80501::kRegisterConfig,
                   Dac80501::kReferenceAndDacPowerDown);
  Serial.println(F("DACREF: reference OFF requested for 5 s (03 01 01)."));
  Serial.println(F("Watch for a fall; the reference capacitor may discharge slowly."));
  const uint32_t startedMs = millis();
  while (millis() - startedMs < 5000) {
    if (hardwareFaultActive()) {
      latchFault("temperature comparator asserted during DAC reference test");
    }
    delay(1);
  }

  // Restore even if FAULT_N asserted; only the reference/configuration is
  // restored, with a zero DAC code and the software fault still latched.
  initializeDac();
  forceLoadOff();
  Serial.println(F("DACREF: reference ON requested; expect return to about 2.5 V."));
  Serial.println(F("Test finished. Load remains latched off. No automatic pass/fail."));
}

void processCommand(char* line) {
  char command[16] = {};
  float value = 0.0F;
  const int fields = sscanf(line, "%15s %f", command, &value);
  if (fields < 1) {
    return;
  }

  if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printHelp();
  } else if (strcmp(command, "status") == 0 || strcmp(command, "s") == 0) {
    printStatus();
  } else if (strcmp(command, "off") == 0) {
    forceLoadOff();
    Serial.println(F("Load command is zero."));
  } else if (strcmp(command, "clear") == 0) {
    clearFaultIfSafe();
  } else if (strcmp(command, "dacref") == 0) {
    testDacReference();
  } else if ((strcmp(command, "i") == 0 ||
              strcmp(command, "current") == 0) &&
             fields == 2) {
    applyCurrentCommand(value);
  } else {
    Serial.println(F("Unknown command. Type 'help'."));
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\r') {
      continue;
    }
    if (character == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(commandBuffer);
      commandLength = 0;
      continue;
    }
    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = character;
    }
  }
}

void setup() {
  // Put both devices in their inactive state before starting the SPI peripheral.
  pinMode(Pins::kAdcCs, OUTPUT);
  pinMode(Pins::kDacCs, OUTPUT);
  digitalWrite(Pins::kAdcCs, HIGH);
  digitalWrite(Pins::kDacCs, HIGH);
  pinMode(Pins::kFaultN, INPUT);

  SPI.begin(Pins::kSclk, Pins::kMiso, Pins::kMosi);
  delay(10);  // Initial settling margin, not a guarantee of board power presence.
  initializeDac();
  forceLoadOff();

  Serial.begin(115200);
  // Native USB CDC may enumerate after setup starts. Do not wait forever,
  // because safety initialization must proceed even without a host attached.
  const uint32_t serialWaitStartedMs = millis();
  while (!Serial && millis() - serialWaitStartedMs < 1500) {
    delay(10);
  }
  // Repeat after USB enumeration in case the board rail came up meanwhile.
  initializeDac();
  forceLoadOff();
  adcHealthy = initializeAdc();

  if (!adcHealthy) {
    latchFault("ADS1120 register readback failed");
  } else if (hardwareFaultActive()) {
    latchFault("temperature comparator asserted at startup");
  } else {
    const Measurements initial = readMeasurements();
    if (!initial.thermistorValid) {
      latchFault("thermistor open or invalid at startup");
    } else if (initial.temperatureC >=
               Limits::kSoftwareTemperatureLimitC) {
      latchFault("software temperature limit reached at startup");
    } else {
      faultLatched = false;
      faultReason = "none";
    }
  }

  printHelp();
  printStatus();
}

void loop() {
  if (hardwareFaultActive()) {
    latchFault("temperature comparator asserted");
  }

  readSerialCommands();

  if (millis() - lastTelemetryMs >= 1000) {
    lastTelemetryMs = millis();
    printStatus();
  }

  delay(1);
}
