/* PicoFox - 2M FM Fox Transmitter

Copyright 2025 Giorgi Enterprises LLC dba AI6YM.radio.
License: Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (see LICENSE file).

COMMERCIAL USE OF THIS CODE AND ANY DERIVATIVE IS FORBIDDEN WITHOUT PERMISSION FROM THE COPYRIGHT HOLDER!

You may configure this software without compiling and flashing. Simply connect the device to your PC with a USB cable and edit the
settings.txt file. You may also change the audio which is played between morse code IDs by replacing the audio.wav file. The file you
provide MUST be 5kHz sampling, unsigned 16bit PCM audio.

The device may be reset to factory settings by deleting the settings.txt and audio.wav files. In factory settings the device WILL NOT
transmit. Configuring a callsign is required to enable the transmitter.
*/

#include <Arduino.h>
#include <Wire.h>
#include <si5351.h>
#include "SPI.h"
#include "SdFat_Adafruit_Fork.h"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"

#include "audio.h"

// #define DEBUG 1

// Board revision ID pins.
// Pulldown required. 0000 == r2, 1000 == r3.
#define ID0 22u
#define ID1 23u
#define ID2 24u
#define ID3 25u

#define SDA 0u                                  // I2C data pin.
#define SCL 1u                                  // I2C clock pin.
#define I2C_CLOCK_HZ 1000000                    // >= 800kHz needed for sufficiently fast I2C writes.
#define AMP_EN 2u                               // Pin which enables the amplifier. Set LOW to enable, HIGH to disable.
#define ATTN_LE 4u                              // Pin which latches the attenuator.
#define AUDIO_SAMPLE_RATE_HZ 5000               // Audio sample rate (also rate of updates to SI5351).
#define FM_DEVIATION_HZ 10000                   // Maximum RF deviation in Hz.
#define SI5351_PLL SI5351_PLLA                  // PLL which will be used on the SI5351, consider this PLL unavailable for other purposes.
#define SI5351_CLOCK_OUTPUT SI5351_CLK0         // Clock output which drives RF circuitry.
#define SI5351_PLL_HZ 900000000                 // PLL frequency in Hz.
#define SI5351_MS_MAX_DENOM 1048575             // 2^20 - 1, max denominator for fractional MS divider.
#define SI5351_MS_RATIONAL_CALC_ITERS 16        // Maximum iterations searching for MS parameters, keeps calculation within 200us.

#define SI5351_CLOCK_PARAM_START 42             // Register address on which multisynth parameters start.
#define SI5351_CLOCK_DIV 6                      // Fixed divider between PLL clock and MS.
#define SI5351_DRIVE_LEVEL_R2 SI5351_DRIVE_2MA  // Power output from SI5351, higher levels don't add much output but do create spurious emmissions.
#define SI5351_DRIVE_LEVEL_R3 SI5351_DRIVE_2MA  // Power output from SI5351.
#define SAMPLE_WRITE_CORR_US -3                 // Time correction between sample writes to account for code which can't be measured.
#define MAX_UINT32 4294967295                   // Maximum value for a 32 bit unsigned integer. Used to prevent glitches when micros() overflows.

#define TEST_MODE_PIN 14u                       // Pin which is monitored to enter test mode. If you use this pin feel free to remove this logic.

// Frequency limitations.
#define MIN_FREQ_MHZ 144
#define MAX_FREQ_MHZ_ITU1 146  // ITU zone 1.
#define MAX_FREQ_MHZ 148       // ITU zones 2 and 3.

// Test mode detected. Indicates program / test before shipping.
bool test_mode;

// Board revision.
uint16_t revision;
bool wireRunning = false;

// RF Clock Gen
Si5351 si5351;

// Uses same external flash chip that the RP2040 is executing from. Size depends on Tools -> Flash Size setting.
Adafruit_FlashTransport_RP2040 flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

// Filesystem and file descriptors.
FatFormatter formatter;
FatVolume fatfs;
FatFile root;
FatFile file;

// Controller for USB mass storage device.
Adafruit_USBD_MSC usb_msc;
bool hostMounted = false;

// Flash file names.
const char SETTINGS_TXT[] = "settings.txt";
const char AUDIO_WAV[] = "audio.wav";
const char CALLSIGN_WAV[] = ".callsign.wav";
const char SETTINGS_CRC[] = ".settings_crc.bin";

// Default settings.
const char DEFAULT_CALLSIGN[12] = "";
const uint8_t DEFAULT_ITU_ZONE = 2;
const double DEFAULT_FREQ_MHZ = 146.565;
const uint8_t DEFAULT_DUTY_CYCLE = 100;
const uint8_t DEFAULT_WPM = 15;
const uint8_t DEFAULT_FARNSWORTH_WPM = 10;
const uint16_t DEFAULT_MORSE_TONE_HZ = 600;
const uint8_t DEFAULT_TONE_AMPLITUDE_PERCENT = 70;
const uint8_t DEFAULT_ATTENUATION = 0;
const uint8_t ATTENUATION_CYCLE_OFFSETS[] = {0, 40, 20};
const int32_t CARRIER_OFFSET_SEQUENCE_HZ[] = {0, 1000, 3000, 5000, 3000, 1000, 0, -1000, -3000, -5000, -3000, -1000};

// Config struct, start by loading defaults.
struct Settings {
  char callsign[12];
  uint8_t ituZone;
  double transmitFreqMHz;
  uint8_t dutyCyclePercent;
  uint8_t morseWPM;
  uint8_t farnsworthWPM;
  uint16_t morseToneHz;
  uint8_t toneAmplitudePercent;
  bool isConfigured;
  uint8_t attenuation;
};

Settings settings = {
  .callsign = { *DEFAULT_CALLSIGN },
  .ituZone = DEFAULT_ITU_ZONE,
  .transmitFreqMHz = DEFAULT_FREQ_MHZ,
  .dutyCyclePercent = DEFAULT_DUTY_CYCLE,
  .morseWPM = DEFAULT_WPM,
  .farnsworthWPM = DEFAULT_FARNSWORTH_WPM,
  .morseToneHz = DEFAULT_MORSE_TONE_HZ,
  .isConfigured = (strlen(DEFAULT_CALLSIGN) > 0),
  .attenuation = DEFAULT_ATTENUATION
};

int settingsCrc = 0;

struct MsABC {
  uint32_t A;
  uint32_t B;
  uint32_t C;
  Si5351RegSet reg;
  uint8_t int_mode;
};

// Creates a fat16 filesystem in the flash space.
void formatFat16(void) {
  uint8_t workbuf[4096];
  formatter.format(&flash, workbuf);
  if (fatfs.begin(&flash, true, 1, 0)) {
    Serial.println("Flash chip successfully formatted with new empty filesystem!");
  } else {
    Serial.println("Formatting failed. Good luck friend.");
    while (true)
      ;
  }
}

// Reads data from flash device.
int32_t mscReadCb(uint32_t lba, void* buffer, uint32_t bufsize) {
  return flash.readBlocks(lba, (uint8_t*)buffer, bufsize / 512) ? bufsize : -1;
}

// Writes data to the flash device.
int32_t mscWriteCb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  hostMounted = true;
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Flushes writes to flash.
void mscFlushCb(void) {
  flash.syncBlocks();
}

bool openRoot() {
  return root.openRoot(&fatfs);
}

void closeRoot() {
  root.close();
}

// Removes files which shouldn't be present and applies defaults for missing files.
void flashCleanup() {
  // Open root filesystem.
  if (!openRoot()) {
    Serial.println("failed to open rootfs");
  }

  // Write the default settings if settings file is missing.
  if (!root.exists(SETTINGS_TXT)) {
    saveDefaultSettings();
    Serial.println("No settings file found. Writing default settings.");
  }

  // Write the default audio if audio file is missing.
  if (!root.exists(AUDIO_WAV)) {
    saveDefaultAudio();
    Serial.println("No audio file found. Writing default audio.");
  }

  // Close the filesytem.
  closeRoot();
}

// Saves the default settings struct to the settings.txt file in flash.
void saveDefaultSettings() {
  char buffer[512];
  snprintf(
    buffer,
    sizeof(buffer),
    "CALLSIGN=%s\nITU_ZONE=%u\nFREQ_MHZ=%.6f\nDUTY_CYCLE=%u\nATTENUATION=%u\nMORSE_WPM=%u\nMORSE_FARNSWORTH_WPM=%u\nMORSE_TONE=%u\nMORSE_TONE_VOL=%u\n",
    DEFAULT_CALLSIGN,
    DEFAULT_ITU_ZONE,
    DEFAULT_FREQ_MHZ,
    DEFAULT_DUTY_CYCLE,
    DEFAULT_ATTENUATION,
    DEFAULT_WPM,
    DEFAULT_FARNSWORTH_WPM,
    DEFAULT_MORSE_TONE_HZ,
    DEFAULT_TONE_AMPLITUDE_PERCENT);
  file.open(&root, SETTINGS_TXT, O_RDWR | O_CREAT);
  file.write(buffer, strlen(buffer));
  file.close();
}

// Saves the default audio wav to the audio.wav file in flash.
void saveDefaultAudio() {
  file.open(&root, AUDIO_WAV, O_RDWR | O_CREAT);
  file.write(wavHeader, sizeof(wavHeader));  // Write wav header once.
  // Loop default audio to produce longer playback between IDs and transmitter cycling.
  for (int i = 0; i < DEFAULT_AUDIO_LOOPS; i++) {
    file.write(defaultAudio, sizeof(defaultAudio));
  }
  file.close();
}

// Loads the settings.txt file in flash into a settings struct.
void loadSettings() {
  openRoot();
  if (!file.open(&root, SETTINGS_TXT, O_RDONLY)) {
    Serial.println("Failed to open settings.txt");
    return;
  }

  String line = "";
  int c;
  settingsCrc = 0;
  while ((c = file.read()) >= 0) {
    settingsCrc += c;
    char ch = (char)c;
    ch = toupper(ch);
    if (ch == '\n') {
      int sep = line.indexOf('=');
      if (sep == -1) continue;
      String key = line.substring(0, sep);
      String val = line.substring(sep + 1);
      if (key == "CALLSIGN") strncpy(settings.callsign, val.c_str(), sizeof(settings.callsign) - 1);
      else if (key == "ITU_ZONE") settings.ituZone = val.toInt();
      else if (key == "FREQ_MHZ") settings.transmitFreqMHz = val.toDouble();
      else if (key == "DUTY_CYCLE") settings.dutyCyclePercent = val.toInt();
      else if (key == "ATTENUATION") settings.attenuation = val.toInt();
      else if (key == "MORSE_WPM") settings.morseWPM = val.toInt();
      else if (key == "MORSE_FARNSWORTH_WPM") settings.farnsworthWPM = val.toInt();
      else if (key == "MORSE_TONE") settings.morseToneHz = val.toInt();
      else if (key == "MORSE_TONE_VOL") settings.toneAmplitudePercent = val.toInt();
      line = "";
    } else {
      line += ch;
    }
  }
  file.close();
  closeRoot();

  // Validate config and apply defaults when config is weird.
  if (settings.dutyCyclePercent > 100) {
    settings.dutyCyclePercent = DEFAULT_DUTY_CYCLE;
  }
  if (settings.attenuation < 0) {
    settings.attenuation = 0;
  } else if (settings.attenuation > 127) {
    settings.attenuation = 127;
  }
  if (settings.morseWPM == 0 || settings.morseWPM > 30) {
    settings.morseWPM = DEFAULT_WPM;
  }
  if (settings.farnsworthWPM == 0 || settings.farnsworthWPM < settings.morseWPM) {
    settings.farnsworthWPM = settings.morseWPM;
  }
  if (settings.morseToneHz < 100 || settings.morseToneHz >= 2500) {
    settings.morseToneHz = DEFAULT_MORSE_TONE_HZ;
  }
  if (settings.toneAmplitudePercent == 0 || settings.toneAmplitudePercent > 100) {
    settings.toneAmplitudePercent = DEFAULT_TONE_AMPLITUDE_PERCENT;
  }

  // Disable transmitter if no callsign is set.
  settings.isConfigured = (strlen(settings.callsign) > 0);

  // Disable transmitter if transmit frequency is out of band.
  uint32_t transmitFreqHz = settings.transmitFreqMHz * 1e6;
  uint16_t deviationMarginHz = FM_DEVIATION_HZ * 2;
  uint32_t minFreqHz = MIN_FREQ_MHZ * 1e6 + deviationMarginHz;
  uint32_t maxFreqHz;
  if (settings.ituZone == 1) {
    maxFreqHz = MAX_FREQ_MHZ_ITU1 * 1e6 - deviationMarginHz;
  } else {
    maxFreqHz = MAX_FREQ_MHZ * 1e6 - deviationMarginHz;
  }
  if (transmitFreqHz < minFreqHz || transmitFreqHz > maxFreqHz) {
    settings.isConfigured = false;
  }
}

bool settingsChanged() {
  bool changed = false;

  // If the marker file can be opened for reading check for a match, assume regen needed if file open fails.
  if (file.open(&root, SETTINGS_CRC, O_RDONLY)) {
    int savedCrc;
    file.read((void*)&savedCrc, sizeof(savedCrc));
    file.close();
    if (savedCrc == settingsCrc) {
      return false;
    }
  }

  file.close();
  return true;
}

// Generates an audio morse code ID from the provided callsign.
void generateMorseAudio() {
  openRoot();
  if (!file.open(&root, CALLSIGN_WAV, O_RDWR | O_CREAT)) {
    Serial.println("Failed to open .callsign.wav");
    return;
  }

  // Remove the existing data, if any.
  file.remove();
  if (!file.open(&root, CALLSIGN_WAV, O_RDWR | O_CREAT)) {
    Serial.println("Failed to re-open .callsign.wav after nuking the old one.");
    return;
  }

  // Write wav header.
  file.write(wavHeader, sizeof(wavHeader));

  const int sampleRate = AUDIO_SAMPLE_RATE_HZ;
  const int toneFreq = settings.morseToneHz;
  const double ditLengthSec = 1.2f / settings.morseWPM;
  const double interCharLengthSec = 1.2f / settings.farnsworthWPM * 3;
  const double riseFall = (1.0f / 3.0f) * ditLengthSec * sampleRate;

  const int amplitude = (settings.toneAmplitudePercent * 32767) / 100;
  const double toneStep = 2.0f * PI * toneFreq / sampleRate;

  auto writeTone = [&](double durationSec) {
    int samples = (int)(durationSec * sampleRate);
    for (int i = 0; i < samples; i++) {
      double envelope = 1.0f;
      if (i < riseFall) envelope = i / riseFall;
      else if (i > samples - riseFall) envelope = (samples - i) / riseFall;
      int16_t sample = (int16_t)(amplitude * envelope * sinf(toneStep * i));
      file.write((uint8_t*)&sample, 2);
    }
  };

  auto writeSilence = [&](double durationSec) {
    int samples = (int)(durationSec * sampleRate);
    int16_t zero = 0;
    for (int i = 0; i < samples; i++) {
      file.write((uint8_t*)&zero, 2);
    }
  };

  const char* morseTable[36] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",
    ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.",
    "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..",
    "-----", ".----", "..---", "...--", "....-",
    ".....", "-....", "--...", "---..", "----."
  };

  auto getMorse = [&](char c) -> const char* {
    if (c >= 'A' && c <= 'Z') return morseTable[c - 'A'];
    if (c >= '0' && c <= '9') return morseTable[c - '0' + 26];
    return "";
  };

  writeSilence(interCharLengthSec);
  for (int i = 0; settings.callsign[i] && i < 12; i++) {
    const char* symbol = getMorse(settings.callsign[i]);
    for (int j = 0; symbol[j]; j++) {
      if (j != 0) writeSilence(ditLengthSec);
      if (symbol[j] == '.') writeTone(ditLengthSec);
      else if (symbol[j] == '-') writeTone(3 * ditLengthSec);
    }
    writeSilence(interCharLengthSec);
  }
  file.close();

  // Write a marker to indicate that the current callsign was generated.
  file.open(&root, SETTINGS_CRC, O_RDWR | O_CREAT);
  file.write((const void*)&settingsCrc, sizeof(settingsCrc));
  file.close();

  closeRoot();
}

// Regenerates morse code ID audio if the callsign has changed.
void generateMorseIfNeeded() {
  openRoot();
  bool regenerate = true;

  // Check for changes in settings file.
  if (!settingsChanged()) {
    regenerate = false;
  }

  // Force regen if the callsign audio file is missing.
  if (!root.exists(CALLSIGN_WAV)) {
    regenerate = true;
  }
  closeRoot();

  if (regenerate) {
    Serial.println("Detected settings change, generating new callsign audio");
    generateMorseAudio();
  }
}

// Remove junk, load settings and audio data. Generate morse ID audio if necessary.
void loadFlashData() {
  flashCleanup();
  loadSettings();
  generateMorseIfNeeded();
}

// Starts the shared I2C bus if it is not already running.
void startWire() {
  if (wireRunning) return;

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  wireRunning = true;
}

// Stops the shared I2C bus so the attenuator can reclaim the pins.
void stopWire() {
  if (!wireRunning) return;

  Wire.end();
  wireRunning = false;
}

// Programs the BVA1761 attenuator. This temporarily reclaims the shared SDA/SCL pins.
void programAttenuator(uint8_t attenuation) {
  if (revision != 3) return;

  if (attenuation > 127) {
    attenuation = 127;
  }

  stopWire();

  pinMode(ATTN_LE, OUTPUT);
  digitalWrite(ATTN_LE, LOW);

  // Hold SCL low before touching SDA so the shared bus does not see an I2C START.
  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, LOW);
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);

  // Attenuation level is first byte LSB first.
  // All address bits zero, written after attn byte.
  uint8_t mask = 1;
  for (int i = 0; i < 16; i++) {
    if (i < 8) {
      if ((attenuation & mask) > 0) {
        pinMode(SDA, INPUT);
      } else {
        pinMode(SDA, OUTPUT);
        digitalWrite(SDA, LOW);
      }
      mask = mask << 1;
    } else {
      pinMode(SDA, OUTPUT);
      digitalWrite(SDA, LOW);
    }

    // Pulse clock to write the current bit.
    pinMode(SCL, INPUT);
    delayMicroseconds(1);
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(1);
  }

  // Pulse the LE pin to apply the specified attenuation level.
  digitalWrite(ATTN_LE, HIGH);
  delayMicroseconds(1);
  digitalWrite(ATTN_LE, LOW);

  startWire();
}

// Returns the next attenuation in the repeating base, base+40, base+20 sequence.
uint8_t getAttenuationCycleValue(uint8_t baseAttenuation, uint8_t cycleStep) {
  uint16_t attenuation = baseAttenuation + ATTENUATION_CYCLE_OFFSETS[cycleStep % 3];
  if (attenuation > 127) {
    attenuation = 127;
  }
  return attenuation;
}

// Returns the next carrier offset in the repeating stepped-offset playback sequence.
int32_t getCarrierOffsetHz(uint8_t sequenceStep) {
  return CARRIER_OFFSET_SEQUENCE_HZ[sequenceStep % (sizeof(CARRIER_OFFSET_SEQUENCE_HZ) / sizeof(CARRIER_OFFSET_SEQUENCE_HZ[0]))];
}

// Enables and disables the si5351 clock output and the RF amplifier.
void setSi5351Output(bool enabled) {
  if (enabled && settings.dutyCyclePercent > 0) {
    si5351.output_enable(SI5351_CLOCK_OUTPUT, 1);
    digitalWrite(AMP_EN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    si5351.output_enable(SI5351_CLOCK_OUTPUT, 0);
    digitalWrite(AMP_EN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// Sets the output frequency of the si5351 based on a given deviation from the center frequency.
void setFrequencyOffset(double deviation) {
  uint64_t freq = (uint64_t)(((settings.transmitFreqMHz * 1e6) + deviation) * 100);
  uint64_t pll_freq = freq * SI5351_CLOCK_DIV;
  si5351.set_pll(pll_freq, SI5351_PLL);
}

// Sleeps until it is time to write the next sample, based on the start of the cycle writing the current sample.
void delayForNextSample(uint32_t start) {
  uint32_t sampleDelta = 1000000UL / AUDIO_SAMPLE_RATE_HZ + SAMPLE_WRITE_CORR_US;

  // At the boundary of micros() overflow. Sleep until the appropriate time AFTER the overflow.
  // Otherwise just sleep for the appropriate delta.
  if ((MAX_UINT32 - start) < sampleDelta) {
    while (micros() > start || micros() < (sampleDelta - (MAX_UINT32 - start)))
      ;
  } else {
    while (micros() - start < sampleDelta)
      ;
  }
}

// Generates narrowband frequency modulated RF from a given audio file.
uint32_t playAudio(const char* filename, int32_t carrierOffsetHz) {
  if (hostMounted) return 0;  // Refuse to play audio if the host is trying to use the filesystem.
  openRoot();
  if (!file.open(&root, filename, O_RDWR | O_CREAT)) {
    Serial.println("Failed to open audio file.");
    return 0;
  }

  // Load WAV header and validate
  uint32_t dataStart = 44;
  char header[44];
  if (file.read(header, 44) != 44) return 0;
  if (strncmp(header, "RIFF", 4) != 0 || strncmp(header + 8, "WAVE", 4) != 0) return 0;

  #ifdef DEBUG
  uint32_t startMicros = micros();
  #endif

  uint32_t numSamples = 0;
  int16_t sample;
  while (file.read((uint8_t*)&sample, 2) == 2) {
    // End playback early if host is trying to use the filesystem.
    if (hostMounted) {
      return 0;
    }
    uint32_t start = micros();

    // Frequency deviation mapping
    double deviation = carrierOffsetHz + (((double)sample * FM_DEVIATION_HZ) / 32767);
    setFrequencyOffset(deviation);

    // Wait until next sample time
    numSamples++;
    delayForNextSample(start);
  }

  #ifdef DEBUG
  uint32_t endMicros = micros();
  Serial.print("Total time (us) ");
  Serial.print(endMicros - startMicros);
  Serial.print(" total samples ");
  Serial.print(numSamples);
  Serial.print(" us per sample ");
  Serial.println((double)(endMicros - startMicros) / numSamples);
  #endif

  file.close();
  closeRoot();
  uint32_t audioLengthMs = numSamples * 1000 / AUDIO_SAMPLE_RATE_HZ;
  return audioLengthMs;
}

// Plays the given audio file and generated morse code id repeatedly.
void audioTask() {
  uint8_t attenuationCycleStep = 1;  // setup() already programs the base attenuation, so stage the next level first.
  uint8_t carrierSequenceStep = 0;

  while (true) {
    if (hostMounted) {  // Cancel task if the host is trying to use the filesystem.
      setSi5351Output(false);
      break;
    }

    if (settings.isConfigured && settings.dutyCyclePercent > 0) {
      int32_t carrierOffsetHz = getCarrierOffsetHz(carrierSequenceStep);
      carrierSequenceStep = (carrierSequenceStep + 1) % (sizeof(CARRIER_OFFSET_SEQUENCE_HZ) / sizeof(CARRIER_OFFSET_SEQUENCE_HZ[0]));
      setFrequencyOffset(carrierOffsetHz);
      setSi5351Output(true);
      uint32_t audioLengthMs = playAudio(AUDIO_WAV, carrierOffsetHz);
      if (!test_mode) { // No generated morse in test mode.
        carrierOffsetHz = getCarrierOffsetHz(carrierSequenceStep);
        carrierSequenceStep = (carrierSequenceStep + 1) % (sizeof(CARRIER_OFFSET_SEQUENCE_HZ) / sizeof(CARRIER_OFFSET_SEQUENCE_HZ[0]));
        setFrequencyOffset(carrierOffsetHz);
        audioLengthMs += playAudio(CALLSIGN_WAV, carrierOffsetHz);
      }
      if (settings.dutyCyclePercent < 100) {
        uint32_t offTime = ((100 - settings.dutyCyclePercent) * audioLengthMs) / 100;
        setSi5351Output(false);
        programAttenuator(getAttenuationCycleValue(settings.attenuation, attenuationCycleStep));
        attenuationCycleStep = (attenuationCycleStep + 1) % 3;
        delay(offTime);
      }
    } else {
      delay(1000);
    }
  }
}

void setup() {
  // Initialize flash device.
  while (!flash.begin()) {
    Serial.println("Flash setup failed.");
    delay(1000);
  }

  Serial.println("Formatting flash...");
  formatFat16();
  Serial.println("Format complete.");

  // Init file system on the flash
  if (!fatfs.begin(&flash, true, 1, 0)) {
    Serial.println("Flash not formatted, attempting to format.");
    formatFat16();
  }
  loadFlashData();

  // Init USB mass storage device.
  usb_msc.setID("AI6YM", "PicoFox", "2.0");
  usb_msc.setReadWriteCallback(mscReadCb, mscWriteCb, mscFlushCb);
  usb_msc.setCapacity(flash.size() / 512, 512);
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  // Forces reenumeration when the device is reset.
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // Setup serial, after USB setup or logs in setup will get eaten.
  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial) delay(10);
  Serial.println("Serial setup complete.");
#endif

  // Check for test mode.
  pinMode(TEST_MODE_PIN, INPUT_PULLDOWN);
  test_mode = digitalRead(TEST_MODE_PIN);

  // Initialize board ID pins and ID the board revisions.
  pinMode(ID0, INPUT_PULLDOWN);
  pinMode(ID1, INPUT_PULLDOWN);
  pinMode(ID2, INPUT_PULLDOWN);
  pinMode(ID3, INPUT_PULLDOWN);
  bool id0 = digitalRead(ID0);
  bool id1 = digitalRead(ID1);
  bool id2 = digitalRead(ID2);
  bool id3 = digitalRead(ID3);
  if (!id0 && !id1 && !id2 && !id3) {
    revision = 2;
  } else if (id0 && !id1 && !id2 && !id3) {
    revision = 3;
  } else {
    revision = 0;  // This should never happen, lol.
  }
  Serial.print("Revision: ");
  Serial.println(revision);

  // Initialize pins for LED and Amp control.
  pinMode(AMP_EN, OUTPUT);
  digitalWrite(AMP_EN, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize the BVA1761 if present.
  if (revision == 3) {
    programAttenuator(settings.attenuation);
  }

  // Initialize the si5351.
  startWire();
  si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0);  // Set load expected by external TCXO.
  // Set drive strength.
  if (revision == 3) {
    si5351.drive_strength(SI5351_CLOCK_OUTPUT, SI5351_DRIVE_LEVEL_R3);
  } else {
    si5351.drive_strength(SI5351_CLOCK_OUTPUT, SI5351_DRIVE_LEVEL_R2);
  }
  if (settings.isConfigured || test_mode) {
    si5351.set_int(SI5351_CLOCK_OUTPUT, 1);  // Clock will be set to a fixed int mult, fine adjustment done on the PLL mult.
    si5351.set_pll((settings.transmitFreqMHz * 1e8) * SI5351_CLOCK_DIV, SI5351_PLL);  // Set the PLL frequency based on the target frequency.
    si5351.set_freq_manual(settings.transmitFreqMHz * 1e8, si5351.plla_freq, SI5351_CLOCK_OUTPUT);  // Let the library calculate and set the simple div by 6 MS.
    si5351.pll_reset(SI5351_PLL);  // Soft reset to get the PLL moving.
    setSi5351Output(true);
    Serial.println("si5351 setup complete.");
  }
  
  // Start the audio playback task on the second core to prevent timing issues.
  multicore_launch_core1(audioTask);
}

void loop() {
}
