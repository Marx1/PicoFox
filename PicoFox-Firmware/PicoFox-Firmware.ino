/*
  PicoFox - Production cleanup v35
  Based on PicoFox firmware by Giorgi Enterprises LLC dba AI6YM.radio.
  Original project:
  https://github.com/Marx1/PicoFox

  Original license:
  Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International.
  COMMERCIAL USE OF THIS CODE AND ANY DERIVATIVE IS FORBIDDEN WITHOUT
  PERMISSION FROM THE COPYRIGHT HOLDER.

  Stage 1:
    - Preserve flash contents across reboot. Flash is formatted only when mount fails.

  Stage 2:
    - Add SSTV_ENABLE and SSTV_MODE settings.

  Stage 3:
    - Add a real-time SSTV audio-tone DDS/NCO.
    - IMPORTANT: SSTV tone frequency is AUDIO frequency. A constant Si5351
      frequency offset would demodulate as DC, not as a 1200-2300 Hz SSTV tone.
      This code therefore synthesizes a sine-wave FM deviation just like PCM audio.

  Stage 4:
    - Add Robot 36 VIS/header and scan timing.
    - Add a built-in 256x240 monochrome diagnostic pattern.
    - SSTV modulation is intentionally held at 5 kHz for timing stability.
    - Measure Si5351 update duration, missed sample deadlines, and real frame time.
    - When SSTV_ENABLE=1, one Robot 36 diagnostic image is sent once at boot,
      then the normal PicoFox audio/Morse loop starts.
    - RF output remains disabled for 5 seconds after boot before any transmit.
    - JPEG decoding, alternating NORMAL/SSTV scheduling, and file cycling are
      intentionally NOT implemented yet.

  Arduino environment:
    - Earle Philhower RP2040 core
    - Etherkit Si5351
    - Adafruit SPIFlash
    - Adafruit TinyUSB
    - SdFat - Adafruit Fork

  No JPEG library is required for stages 1-4.
*/

#include <Arduino.h>
#include <Wire.h>
#include <si5351.h>
#include "SPI.h"
#include "SdFat_Adafruit_Fork.h"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"
#include <JPEGDEC.h>

#include "audio.h"

// #define DEBUG 1

// -----------------------------------------------------------------------------
// Hardware / original PicoFox constants
// -----------------------------------------------------------------------------

#define ID0 22u
#define ID1 23u
#define ID2 24u
#define ID3 25u

#define SDA 0u
#define SCL 1u
#define I2C_CLOCK_HZ 1000000

#define AMP_EN 2u
#define ATTN_LE 4u

#define AUDIO_SAMPLE_RATE_HZ 5000
#define AUDIO_FM_DEVIATION_HZ 5000
#define SSTV_FM_DEVIATION_HZ 3000

#define SI5351_PLL SI5351_PLLA
#define SI5351_CLOCK_OUTPUT SI5351_CLK0
#define SI5351_PLL_HZ 900000000
#define SI5351_CLOCK_DIV 6

#define SI5351_DRIVE_LEVEL_R2 SI5351_DRIVE_2MA
#define SI5351_DRIVE_LEVEL_R3 SI5351_DRIVE_2MA

#define SAMPLE_WRITE_CORR_US -3
#define TEST_MODE_PIN 14u

#define MIN_FREQ_MHZ 144
#define MAX_FREQ_MHZ_ITU1 146
#define MAX_FREQ_MHZ 148

// -----------------------------------------------------------------------------
// SSTV stage 1-4 constants
// -----------------------------------------------------------------------------

// SSTV uses a higher modulation update rate than the legacy WAV/Morse path.
// 10 kHz gives ~4.35 samples/cycle at the 2300 Hz video upper limit while
// NOTE: if the Si5351/I2C update cannot complete inside 100 us, decoded timing
// will run long. DEBUG timing can be added later to measure that directly.
#define SSTV_SAMPLE_RATE_HZ 8000UL
#define SSTV_SAMPLE_PERIOD_US (1000000UL / SSTV_SAMPLE_RATE_HZ)

// Keep the transmitter RF-silent briefly after power-up/reset.
#define BOOT_TX_DELAY_MS 5000UL

// Re-use the same FM deviation that the existing PCM player uses.
// This is the PEAK RF deviation of the synthesized audio sine wave.

// Timing-critical SSTV PLL path.
// The 8 kHz modulation loop has a 125 us sample period and the Si5351 I2C
// transaction consumes most of that budget. Keep filesystem access, Serial
// output, JPEG decoding, RGB conversion, floating-point work, and other
// variable-latency operations outside the real-time transmit loop.
#define SSTV_PLL_TABLE_LEVELS 512
#define SI5351_I2C_ADDRESS 0x60
#define SI5351_PLLA_PARAMETERS_REG 26
#define SI5351_RFRAC_DENOM 1000000ULL
#define SI5351_REF_FREQ_X100 2500000000ULL

#define SSTV_MODE_ROBOT36 36
#define PICOFOX_FIRMWARE_VERSION "v35"
#define DEFAULT_VOICE_ENABLE true
#define DEFAULT_SSTV_ENABLE false
#define DEFAULT_SSTV_MODE SSTV_MODE_ROBOT36

// SSTV standard tone frequencies.
#define SSTV_VIS_LEADER_HZ 1900
#define SSTV_VIS_BREAK_HZ 1200
#define SSTV_VIS_BIT_1_HZ 1100
#define SSTV_VIS_BIT_0_HZ 1300
#define SSTV_SYNC_HZ 1200
#define SSTV_BLACK_HZ 1500
#define SSTV_MID_HZ 1900
#define SSTV_WHITE_HZ 2300

// Robot 36 timing.
#define R36_WIDTH 256
#define R36_HEIGHT 240
#define R36_SYNC_US 9000UL
#define R36_SYNC_PORCH_US 3000UL
#define R36_Y_SCAN_US 88000UL
#define R36_SEPARATOR_US 4500UL
#define R36_CHROMA_PORCH_US 1500UL
#define R36_CHROMA_SCAN_US 44000UL

// VIS code for Robot 36.
#define R36_VIS_CODE 8

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

bool test_mode = false;
uint16_t revision = 0;
bool wireRunning = false;

Si5351 si5351;

Adafruit_FlashTransport_RP2040 flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

FatFormatter formatter;
FatVolume fatfs;
FatFile root;
FatFile file;

Adafruit_USBD_MSC usb_msc;
volatile bool hostMounted = false;
volatile bool filesystemBusy = false;  // Prevent USB MSC writes racing active SdFat reads.

const char SETTINGS_TXT[] = "settings.txt";
const char AUDIO_WAV[] = "audio.wav";
const char CALLSIGN_WAV[] = ".callsign.wav";
const char SETTINGS_CRC[] = ".settings_crc.bin";

const char SSTV_JPEG_FILE[] = "sstv.jpg";
const char SSTV_JPEG_PREFIX[] = "sstv";
const char SSTV_JPEG_EXT[] = ".jpg";
const char AUDIO_SEQUENCE_PREFIX[] = "audio";
const char AUDIO_SEQUENCE_EXT[] = ".wav";
#define FILE_SEQUENCE_MAX 999U

#define SSTV_IMAGE_WIDTH 320
#define SSTV_IMAGE_HEIGHT 240

// 320 x 240 x 2 = 153,600 bytes.
static uint16_t sstvImage[SSTV_IMAGE_WIDTH * SSTV_IMAGE_HEIGHT];

JPEGDEC jpeg;
FatFile jpegRoot;
FatFile jpegFile;

int jpegSourceWidth = 0;
int jpegSourceHeight = 0;

volatile uint32_t sstvPixelBoundsErrors = 0;
volatile uint32_t sstvLastLineBoundsErrors = 0;

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

const int32_t CARRIER_OFFSET_SEQUENCE_HZ[] = {
  0, 1000, 3000, 5000, 3000, 1000,
  0, -1000, -3000, -5000, -3000, -1000
};

// SSTV timing diagnostics. Keep the type near the top because Arduino's
// .ino preprocessor may generate prototypes before later declarations.
struct SstvTimingStats {
  uint32_t updates;
  uint64_t totalUpdateUs;
  uint32_t minUpdateUs;
  uint32_t maxUpdateUs;
  uint32_t missedDeadlines;
  uint32_t frameStartUs;
  uint32_t frameEndUs;
};

struct SstvPllEntry {
  uint8_t reg[8];
};

struct SstvRgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

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

  // Normal audio.wav + Morse callsign transmission.
  // Set VOICE_ENABLE=0 for SSTV-only operation.
  bool voiceEnabled;

  // SSTV configuration.
  bool sstvEnabled;
  uint8_t sstvMode;
};

Settings settings = {
  .callsign = "",
  .ituZone = DEFAULT_ITU_ZONE,
  .transmitFreqMHz = DEFAULT_FREQ_MHZ,
  .dutyCyclePercent = DEFAULT_DUTY_CYCLE,
  .morseWPM = DEFAULT_WPM,
  .farnsworthWPM = DEFAULT_FARNSWORTH_WPM,
  .morseToneHz = DEFAULT_MORSE_TONE_HZ,
  .toneAmplitudePercent = DEFAULT_TONE_AMPLITUDE_PERCENT,
  .isConfigured = false,
  .attenuation = DEFAULT_ATTENUATION,
  .voiceEnabled = DEFAULT_VOICE_ENABLE,
  .sstvEnabled = DEFAULT_SSTV_ENABLE,
  .sstvMode = DEFAULT_SSTV_MODE
};

int settingsCrc = 0;

// DDS phase. Keeping phase continuous across SSTV frequency changes avoids
// unnecessary discontinuities at pixel/timing boundaries.
double sstvPhase = 0.0;

SstvTimingStats sstvStats = {0, 0, 0xFFFFFFFFUL, 0, 0, 0, 0};

SstvPllEntry sstvPllTable[SSTV_PLL_TABLE_LEVELS];
bool sstvPllTableReady = false;
int32_t sstvPllCarrierOffsetHz = 0;
uint32_t sstvPllWriteErrors = 0;

// Real-time SSTV fast path.
// The expensive floating-point sine and PLL-table mapping are precomputed.
// During transmission each modulation update only:
//   1. reads a PLL table index from the sine LUT,
//   2. writes the 8-byte PLL block over I2C,
//   3. advances a 32-bit phase accumulator.
uint16_t sstvSinePllIndex[256];
uint32_t sstvTonePhaseIncrement[256];
uint32_t sstvPhase32 = 0;

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------

void formatFat16();
void saveDefaultSettings();
void saveDefaultAudio();
void setSi5351Output(bool enabled);
void setFrequencyOffset(double deviation);
void programAttenuator(uint8_t attenuation);
uint32_t playAudio(const char* filename, int32_t carrierOffsetHz);
void audioTask();

// SSTV stages 3-4.
void sendSSTVTone(uint16_t toneHz, uint32_t durationUs, int32_t carrierOffsetHz);
void sendRobot36VIS(int32_t carrierOffsetHz);
void sendRobot36Jpeg(const char* filename, int32_t carrierOffsetHz);
uint16_t getSstvPixelSafe(uint16_t x, uint16_t line);
bool flashFileExists(const char* filename);
bool nextSequencedFile(const char* prefix,
                       const char* extension,
                       uint16_t* index,
                       char* outName,
                       size_t outNameSize);
uint32_t playNextNormalTransmission(uint8_t* carrierSequenceStep,
                                    uint16_t* audioSequenceIndex);
uint32_t sendNextSstvTransmission(uint8_t* carrierSequenceStep,
                                  uint16_t* sstvSequenceIndex);
void applyDutyCycleOff(uint32_t activeLengthMs,
                       uint8_t* attenuationCycleStep);
bool loadRobot36Jpeg(const char* filename);
void* jpegOpenCallback(const char* filename, int32_t* fileSize);
void jpegCloseCallback(void* handle);
int32_t jpegReadCallback(JPEGFILE* handle, uint8_t* buffer, int32_t length);
int32_t jpegSeekCallback(JPEGFILE* handle, int32_t position);
int jpegDrawCallback(JPEGDRAW* draw);
void resetSstvTimingStats();
void recordSstvUpdateTime(uint32_t updateUs, uint32_t sampleBudgetUs);
void reportSstvTimingStats();
void calculateSstvPllRegisters(int32_t totalOffsetHz, uint8_t out[8]);
bool buildSstvPllTable(int32_t carrierOffsetHz);
bool writeSstvPllEntry(uint16_t index);
void setFastSstvDeviation(double audioDeviationHz);
void buildSstvRealtimeTables();
static inline void writeFastSstvSample(uint32_t phaseIncrement);
static inline uint32_t phaseIncrementForTone(uint16_t toneHz);
void preconvertSstvFramebuffer();

// -----------------------------------------------------------------------------
// Filesystem
// -----------------------------------------------------------------------------

void formatFat16() {
  uint8_t workbuf[4096];
  formatter.format(&flash, workbuf);

  if (fatfs.begin(&flash, true, 1, 0)) {
    Serial.println("Flash formatted with a new FAT filesystem.");
  } else {
    Serial.println("Flash formatting failed.");
    while (true) {
      delay(1000);
    }
  }
}

int32_t mscReadCb(uint32_t lba, void* buffer, uint32_t bufsize) {
  return flash.readBlocks(lba, (uint8_t*)buffer, bufsize / 512) ? bufsize : -1;
}

int32_t mscWriteCb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  // Tell the playback core to get out of the filesystem before the host
  // modifies FAT sectors underneath it.
  hostMounted = true;

  // playAudio() checks hostMounted every sample, so this should normally
  // clear within a few hundred microseconds. Give it some margin.
  uint32_t waitStart = millis();
  while (filesystemBusy && (millis() - waitStart) < 100) {
    delayMicroseconds(100);
  }

  // If the firmware still owns an open filesystem object, fail this WRITE10
  // rather than corrupting the volume. The host can retry.
  if (filesystemBusy) {
    return -1;
  }

  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

void mscFlushCb() {
  flash.syncBlocks();

  // The host has changed FAT sectors behind SdFat's back. Drop cached
  // filesystem data so it can never be reused as though it were current.
  fatfs.cacheClear();
}

bool openRoot() {
  return root.openRoot(&fatfs);
}

void closeRoot() {
  root.close();
}

void saveDefaultSettings() {
  char buffer[768];

  snprintf(
    buffer,
    sizeof(buffer),
    "CALLSIGN=%s\n"
    "ITU_ZONE=%u\n"
    "FREQ_MHZ=%.6f\n"
    "DUTY_CYCLE=%u\n"
    "ATTENUATION=%u\n"
    "MORSE_WPM=%u\n"
    "MORSE_FARNSWORTH_WPM=%u\n"
    "MORSE_TONE=%u\n"
    "MORSE_TONE_VOL=%u\n"
    "VOICE_ENABLE=%u\n"
    "SSTV_ENABLE=%u\n"
    "SSTV_MODE=ROBOT36\n",
    DEFAULT_CALLSIGN,
    DEFAULT_ITU_ZONE,
    DEFAULT_FREQ_MHZ,
    DEFAULT_DUTY_CYCLE,
    DEFAULT_ATTENUATION,
    DEFAULT_WPM,
    DEFAULT_FARNSWORTH_WPM,
    DEFAULT_MORSE_TONE_HZ,
    DEFAULT_TONE_AMPLITUDE_PERCENT,
    DEFAULT_VOICE_ENABLE ? 1 : 0,
    DEFAULT_SSTV_ENABLE ? 1 : 0
  );

  if (!file.open(&root, SETTINGS_TXT, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("Failed to create default settings.txt");
    return;
  }

  file.write(buffer, strlen(buffer));
  file.close();
}

void saveDefaultAudio() {
  if (!file.open(&root, AUDIO_WAV, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("Failed to create default audio.wav");
    return;
  }

  file.write(wavHeader, sizeof(wavHeader));

  for (int i = 0; i < DEFAULT_AUDIO_LOOPS; i++) {
    file.write(defaultAudio, sizeof(defaultAudio));
  }

  file.close();
}

void ensureSstvSettingsPresent() {
  if (!openRoot()) {
    return;
  }

  if (!file.open(&root, SETTINGS_TXT, O_RDONLY)) {
    closeRoot();
    return;
  }

  bool haveVoiceEnable = false;
  bool haveEnable = false;
  bool haveMode = false;
  String line = "";

  int c;
  while ((c = file.read()) >= 0) {
    char ch = (char)c;

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      String check = line;
      check.trim();
      check.toUpperCase();

      if (check.startsWith("VOICE_ENABLE=")) {
        haveVoiceEnable = true;
      } else if (check.startsWith("SSTV_ENABLE=")) {
        haveEnable = true;
      } else if (check.startsWith("SSTV_MODE=")) {
        haveMode = true;
      }

      line = "";
    } else {
      line += ch;
    }
  }

  if (line.length()) {
    String check = line;
    check.trim();
    check.toUpperCase();

    if (check.startsWith("VOICE_ENABLE=")) {
      haveVoiceEnable = true;
    } else if (check.startsWith("SSTV_ENABLE=")) {
      haveEnable = true;
    } else if (check.startsWith("SSTV_MODE=")) {
      haveMode = true;
    }
  }

  file.close();

  if (!haveVoiceEnable || !haveEnable || !haveMode) {
    if (file.open(&root, SETTINGS_TXT, O_RDWR | O_AT_END)) {
      // Ensure appended keys begin on a fresh line even if the old file did
      // not end with a newline.
      file.write("\n", 1);

      if (!haveVoiceEnable) {
        const char voiceLine[] = "VOICE_ENABLE=1\n";
        file.write(voiceLine, sizeof(voiceLine) - 1);
      }

      if (!haveEnable) {
        const char enableLine[] = "SSTV_ENABLE=0\n";
        file.write(enableLine, sizeof(enableLine) - 1);
      }

      if (!haveMode) {
        const char modeLine[] = "SSTV_MODE=ROBOT36\n";
        file.write(modeLine, sizeof(modeLine) - 1);
      }

      file.close();
      Serial.println("Added missing voice/SSTV settings to settings.txt");
    }
  }

  closeRoot();
}

void flashCleanup() {
  if (!openRoot()) {
    Serial.println("Failed to open root filesystem.");
    return;
  }

  if (!root.exists(SETTINGS_TXT)) {
    Serial.println("No settings.txt found; creating defaults.");
    saveDefaultSettings();
  }

  if (!root.exists(AUDIO_WAV)) {
    Serial.println("No audio.wav found; creating default audio.");
    saveDefaultAudio();
  }

  closeRoot();

  // Existing PicoFox installations already have settings.txt, so add the new
  // stage-2 keys without destroying the user's current configuration.
  ensureSstvSettingsPresent();
}

void loadSettings() {
  if (!openRoot()) {
    Serial.println("Failed to open filesystem while loading settings.");
    return;
  }

  if (!file.open(&root, SETTINGS_TXT, O_RDONLY)) {
    Serial.println("Failed to open settings.txt");
    closeRoot();
    return;
  }

  String line = "";
  settingsCrc = 0;

  int c;
  while ((c = file.read()) >= 0) {
    settingsCrc += c;

    char ch = (char)c;

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      int sep = line.indexOf('=');
      if (sep >= 0) {
        String key = line.substring(0, sep);
        String val = line.substring(sep + 1);

        key.trim();
        val.trim();
        key.toUpperCase();

        if (key == "CALLSIGN") {
          val.toUpperCase();
          strncpy(settings.callsign, val.c_str(), sizeof(settings.callsign) - 1);
          settings.callsign[sizeof(settings.callsign) - 1] = '\0';
        } else if (key == "ITU_ZONE") {
          settings.ituZone = val.toInt();
        } else if (key == "FREQ_MHZ") {
          settings.transmitFreqMHz = val.toDouble();
        } else if (key == "DUTY_CYCLE") {
          settings.dutyCyclePercent = val.toInt();
        } else if (key == "ATTENUATION") {
          settings.attenuation = val.toInt();
        } else if (key == "MORSE_WPM") {
          settings.morseWPM = val.toInt();
        } else if (key == "MORSE_FARNSWORTH_WPM") {
          settings.farnsworthWPM = val.toInt();
        } else if (key == "MORSE_TONE") {
          settings.morseToneHz = val.toInt();
        } else if (key == "MORSE_TONE_VOL") {
          settings.toneAmplitudePercent = val.toInt();
        } else if (key == "VOICE_ENABLE") {
          settings.voiceEnabled = (val.toInt() != 0);
        } else if (key == "SSTV_ENABLE") {
          settings.sstvEnabled = (val.toInt() != 0);
        } else if (key == "SSTV_MODE") {
          val.toUpperCase();
          if (val == "ROBOT36" || val == "36") {
            settings.sstvMode = SSTV_MODE_ROBOT36;
          }
        }
      }

      line = "";
    } else {
      line += ch;
    }
  }

  // Handle a final settings line that does not end in newline.
  if (line.length() > 0) {
    int sep = line.indexOf('=');
    if (sep >= 0) {
      String key = line.substring(0, sep);
      String val = line.substring(sep + 1);
      key.trim();
      val.trim();
      key.toUpperCase();

      if (key == "VOICE_ENABLE") {
        settings.voiceEnabled = (val.toInt() != 0);
      } else if (key == "SSTV_ENABLE") {
        settings.sstvEnabled = (val.toInt() != 0);
      } else if (key == "SSTV_MODE") {
        val.toUpperCase();
        if (val == "ROBOT36" || val == "36") {
          settings.sstvMode = SSTV_MODE_ROBOT36;
        }
      }
    }
  }

  file.close();
  closeRoot();

  if (settings.dutyCyclePercent > 100) {
    settings.dutyCyclePercent = DEFAULT_DUTY_CYCLE;
  }

  if (settings.attenuation > 127) {
    settings.attenuation = 127;
  }

  if (settings.morseWPM == 0 || settings.morseWPM > 30) {
    settings.morseWPM = DEFAULT_WPM;
  }

  if (settings.farnsworthWPM == 0 ||
      settings.farnsworthWPM < settings.morseWPM) {
    settings.farnsworthWPM = settings.morseWPM;
  }

  if (settings.morseToneHz < 100 || settings.morseToneHz >= 2500) {
    settings.morseToneHz = DEFAULT_MORSE_TONE_HZ;
  }

  if (settings.toneAmplitudePercent == 0 ||
      settings.toneAmplitudePercent > 100) {
    settings.toneAmplitudePercent = DEFAULT_TONE_AMPLITUDE_PERCENT;
  }

  settings.isConfigured = (strlen(settings.callsign) > 0);

  uint32_t transmitFreqHz = (uint32_t)(settings.transmitFreqMHz * 1e6);
  uint32_t deviationMarginHz = AUDIO_FM_DEVIATION_HZ * 2UL;
  uint32_t minFreqHz = MIN_FREQ_MHZ * 1000000UL + deviationMarginHz;

  uint32_t maxFreqHz;
  if (settings.ituZone == 1) {
    maxFreqHz = MAX_FREQ_MHZ_ITU1 * 1000000UL - deviationMarginHz;
  } else {
    maxFreqHz = MAX_FREQ_MHZ * 1000000UL - deviationMarginHz;
  }

  if (transmitFreqHz < minFreqHz || transmitFreqHz > maxFreqHz) {
    settings.isConfigured = false;
  }

  if (settings.sstvMode != SSTV_MODE_ROBOT36) {
    settings.sstvMode = SSTV_MODE_ROBOT36;
  }
}

bool settingsChanged() {
  bool changed = true;

  if (!openRoot()) {
    return true;
  }

  if (file.open(&root, SETTINGS_CRC, O_RDONLY)) {
    int savedCrc = 0;

    if (file.read((void*)&savedCrc, sizeof(savedCrc)) == sizeof(savedCrc)) {
      changed = (savedCrc != settingsCrc);
    }

    file.close();
  }

  closeRoot();
  return changed;
}

// -----------------------------------------------------------------------------
// Morse generation
// -----------------------------------------------------------------------------

void generateMorseAudio() {
  if (!openRoot()) {
    return;
  }

  if (file.open(&root, CALLSIGN_WAV, O_RDWR | O_CREAT)) {
    file.remove();
  }

  if (!file.open(&root, CALLSIGN_WAV, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("Failed to create .callsign.wav");
    closeRoot();
    return;
  }

  file.write(wavHeader, sizeof(wavHeader));

  const int sampleRate = AUDIO_SAMPLE_RATE_HZ;
  const int toneFreq = settings.morseToneHz;
  const double ditLengthSec = 1.2 / settings.morseWPM;
  const double interCharLengthSec = (1.2 / settings.farnsworthWPM) * 3.0;
  const double riseFall = (1.0 / 3.0) * ditLengthSec * sampleRate;
  const int amplitude =
    (settings.toneAmplitudePercent * 32767L) / 100L;
  const double toneStep =
    2.0 * PI * toneFreq / sampleRate;

  auto writeTone = [&](double durationSec) {
    int samples = (int)(durationSec * sampleRate);

    for (int i = 0; i < samples; i++) {
      double envelope = 1.0;

      if (i < riseFall) {
        envelope = i / riseFall;
      } else if (i > samples - riseFall) {
        envelope = (samples - i) / riseFall;
      }

      int16_t sample =
        (int16_t)(amplitude * envelope * sin(toneStep * i));

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
    if (c >= 'A' && c <= 'Z') {
      return morseTable[c - 'A'];
    }

    if (c >= '0' && c <= '9') {
      return morseTable[c - '0' + 26];
    }

    return "";
  };

  writeSilence(interCharLengthSec);

  for (int i = 0; settings.callsign[i] && i < 12; i++) {
    const char* symbol = getMorse(settings.callsign[i]);

    for (int j = 0; symbol[j]; j++) {
      if (j != 0) {
        writeSilence(ditLengthSec);
      }

      if (symbol[j] == '.') {
        writeTone(ditLengthSec);
      } else if (symbol[j] == '-') {
        writeTone(3.0 * ditLengthSec);
      }
    }

    writeSilence(interCharLengthSec);
  }

  file.close();

  if (file.open(&root, SETTINGS_CRC, O_RDWR | O_CREAT | O_TRUNC)) {
    file.write((const void*)&settingsCrc, sizeof(settingsCrc));
    file.close();
  }

  closeRoot();
}

void generateMorseIfNeeded() {
  if (!settings.voiceEnabled) {
    Serial.println("VOICE_ENABLE=0; skipping Morse audio generation.");

    // Record the current settings CRC even though no Morse file needs
    // regeneration. This prevents settingsChanged() from firing every boot.
    if (openRoot()) {
      if (file.open(&root, SETTINGS_CRC, O_RDWR | O_CREAT | O_TRUNC)) {
        file.write((const void*)&settingsCrc, sizeof(settingsCrc));
        file.close();
      }
      closeRoot();
    }

    return;
  }

  bool regenerate = settingsChanged();

  if (openRoot()) {
    if (!root.exists(CALLSIGN_WAV)) {
      regenerate = true;
    }
    closeRoot();
  }

  if (regenerate) {
    Serial.println("Generating Morse callsign audio.");
    generateMorseAudio();
  }
}

void loadFlashData() {
  flashCleanup();
  loadSettings();
  generateMorseIfNeeded();
}

// -----------------------------------------------------------------------------
// I2C / attenuator / RF
// -----------------------------------------------------------------------------

void startWire() {
  if (wireRunning) {
    return;
  }

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  wireRunning = true;
}

void stopWire() {
  if (!wireRunning) {
    return;
  }

  Wire.end();
  wireRunning = false;
}

void programAttenuator(uint8_t attenuation) {
  if (revision != 3) {
    return;
  }

  if (attenuation > 127) {
    attenuation = 127;
  }

  stopWire();

  pinMode(ATTN_LE, OUTPUT);
  digitalWrite(ATTN_LE, LOW);

  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, LOW);

  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);

  uint8_t mask = 1;

  for (int i = 0; i < 16; i++) {
    if (i < 8) {
      if (attenuation & mask) {
        pinMode(SDA, INPUT);
      } else {
        pinMode(SDA, OUTPUT);
        digitalWrite(SDA, LOW);
      }

      mask <<= 1;
    } else {
      pinMode(SDA, OUTPUT);
      digitalWrite(SDA, LOW);
    }

    pinMode(SCL, INPUT);
    delayMicroseconds(1);

    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(ATTN_LE, HIGH);
  delayMicroseconds(1);
  digitalWrite(ATTN_LE, LOW);

  startWire();
}

uint8_t getAttenuationCycleValue(uint8_t baseAttenuation,
                                 uint8_t cycleStep) {
  uint16_t attenuation =
    baseAttenuation +
    ATTENUATION_CYCLE_OFFSETS[cycleStep % 3];

  if (attenuation > 127) {
    attenuation = 127;
  }

  return (uint8_t)attenuation;
}

int32_t getCarrierOffsetHz(uint8_t sequenceStep) {
  const uint8_t count =
    sizeof(CARRIER_OFFSET_SEQUENCE_HZ) /
    sizeof(CARRIER_OFFSET_SEQUENCE_HZ[0]);

  return CARRIER_OFFSET_SEQUENCE_HZ[sequenceStep % count];
}

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

void setFrequencyOffset(double deviation) {
  uint64_t freq =
    (uint64_t)(((settings.transmitFreqMHz * 1e6) + deviation) * 100.0);

  uint64_t pll_freq = freq * SI5351_CLOCK_DIV;
  si5351.set_pll(pll_freq, SI5351_PLL);
}

void delayForSampleRate(uint32_t startUs, uint32_t sampleRateHz) {
  uint32_t sampleDelta =
    (1000000UL / sampleRateHz) + SAMPLE_WRITE_CORR_US;

  while ((uint32_t)(micros() - startUs) < sampleDelta) {
    // busy wait for consistent modulation timing
  }
}


// SSTV must not inherit SAMPLE_WRITE_CORR_US. Robot36 timing is generated
// against an absolute 8 kHz clock so per-sample execution overhead cannot
// accumulate into line slant.
static inline bool timeBeforeUs(uint32_t nowUs, uint32_t deadlineUs) {
  return (int32_t)(nowUs - deadlineUs) < 0;
}

static inline void waitForSstvDeadline(uint32_t deadlineUs) {
  while (timeBeforeUs(micros(), deadlineUs)) {
    tight_loop_contents();
  }
}


// -----------------------------------------------------------------------------
// Full-register precomputed PLLA SSTV path
// -----------------------------------------------------------------------------

// Reproduce the Etherkit Si5351 pll_calc()/set_pll() math for the PicoFox
// configuration (25 MHz XO, zero correction, PLLA, frequency units x100).
//
// Etherkit uses RFRAC_DENOM = 1,000,000 for normal PLL calculations:
//   a = freq / ref
//   b = ((freq % ref) * 1,000,000) / ref
//   c = b ? 1,000,000 : 1
//
// Then:
//   P1 = 128*a + floor(128*b/c) - 512
//   P2 = 128*b - c*floor(128*b/c)
//   P3 = c
//
// This function converts those values into the exact eight bytes used by
// Si5351::set_pll() for registers 26..33.
void calculateSstvPllRegisters(int32_t totalOffsetHz, uint8_t out[8]) {
  int64_t carrierHz =
    (int64_t)(settings.transmitFreqMHz * 1000000.0 + 0.5);

  int64_t outputHz = carrierHz + (int64_t)totalOffsetHz;

  // PicoFox keeps CLK0 at integer divide-by-6 and moves PLLA.
  // Etherkit PLL frequency arguments are Hz * 100.
  uint64_t pllFreqX100 =
    (uint64_t)outputHz * SI5351_CLOCK_DIV * 100ULL;

  uint64_t refFreqX100 = SI5351_REF_FREQ_X100;

  uint32_t a = (uint32_t)(pllFreqX100 / refFreqX100);
  uint64_t rem = pllFreqX100 % refFreqX100;

  uint32_t b = (uint32_t)(
    (rem * SI5351_RFRAC_DENOM) / refFreqX100
  );

  uint32_t c = b ? (uint32_t)SI5351_RFRAC_DENOM : 1U;

  uint32_t frac128 = (128UL * b) / c;
  uint32_t p1 = 128UL * a + frac128 - 512UL;
  uint32_t p2 = 128UL * b - c * frac128;
  uint32_t p3 = c;

  out[0] = (uint8_t)((p3 >> 8) & 0xFF);
  out[1] = (uint8_t)(p3 & 0xFF);
  out[2] = (uint8_t)((p1 >> 16) & 0x03);
  out[3] = (uint8_t)((p1 >> 8) & 0xFF);
  out[4] = (uint8_t)(p1 & 0xFF);
  out[5] =
    (uint8_t)(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F));
  out[6] = (uint8_t)((p2 >> 8) & 0xFF);
  out[7] = (uint8_t)(p2 & 0xFF);
}

bool buildSstvPllTable(int32_t carrierOffsetHz) {
  sstvPllTableReady = false;
  sstvPllCarrierOffsetHz = carrierOffsetHz;
  sstvPllWriteErrors = 0;

  for (uint16_t level = 0;
       level < SSTV_PLL_TABLE_LEVELS;
       level++) {

    int32_t audioDeviationHz =
      -SSTV_FM_DEVIATION_HZ +
      (int32_t)(
        ((int64_t)level *
         (2LL * SSTV_FM_DEVIATION_HZ)) /
        (SSTV_PLL_TABLE_LEVELS - 1)
      );

    calculateSstvPllRegisters(
      carrierOffsetHz + audioDeviationHz,
      sstvPllTable[level].reg
    );
  }

  sstvPllTableReady = true;
  buildSstvRealtimeTables();

  Serial.print("SSTV PLL table built: ");
  Serial.print(SSTV_PLL_TABLE_LEVELS);
  Serial.println(" levels; full 8-byte PLLA writes.");

  return true;
}

bool writeSstvPllEntry(uint16_t index) {
  if (index >= SSTV_PLL_TABLE_LEVELS) {
    index = SSTV_PLL_TABLE_LEVELS - 1;
  }

  Wire.beginTransmission(SI5351_I2C_ADDRESS);
  Wire.write((uint8_t)SI5351_PLLA_PARAMETERS_REG);

  // Write the same complete register range as Etherkit set_pll(): 26..33.
  for (uint8_t i = 0; i < 8; i++) {
    Wire.write(sstvPllTable[index].reg[i]);
  }

  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    sstvPllWriteErrors++;
    return false;
  }

  return true;
}

void buildSstvRealtimeTables() {
  // Map 256 phase positions directly to the already-precomputed PLL table.
  // This executes once before a frame, while timing is not critical.
  for (uint16_t i = 0; i < 256; i++) {
    double phase = (2.0 * PI * (double)i) / 256.0;
    double deviation =
      sin(phase) * (double)SSTV_FM_DEVIATION_HZ;

    double normalized =
      (deviation + SSTV_FM_DEVIATION_HZ) /
      (2.0 * SSTV_FM_DEVIATION_HZ);

    uint16_t tableIndex =
      (uint16_t)(
        normalized * (SSTV_PLL_TABLE_LEVELS - 1) + 0.5
      );

    if (tableIndex >= SSTV_PLL_TABLE_LEVELS) {
      tableIndex = SSTV_PLL_TABLE_LEVELS - 1;
    }

    sstvSinePllIndex[i] = tableIndex;
  }

  // Precompute phase increment for each possible 8-bit SSTV video level.
  // tone = 1500 + level * 800 / 255.
  for (uint16_t level = 0; level < 256; level++) {
    uint32_t toneHz =
      SSTV_BLACK_HZ +
      (((uint32_t)level *
        (SSTV_WHITE_HZ - SSTV_BLACK_HZ) + 127UL) / 255UL);

    sstvTonePhaseIncrement[level] =
      (uint32_t)(
        (((uint64_t)toneHz << 32) + (SSTV_SAMPLE_RATE_HZ / 2)) /
        SSTV_SAMPLE_RATE_HZ
      );
  }
}

static inline uint32_t phaseIncrementForTone(uint16_t toneHz) {
  return (uint32_t)(
    (((uint64_t)toneHz << 32) + (SSTV_SAMPLE_RATE_HZ / 2)) /
    SSTV_SAMPLE_RATE_HZ
  );
}

// REAL-TIME CRITICAL: PLL UPDATE
// Do not add logging, allocation, file I/O, or floating-point processing here.
static inline void writeFastSstvSample(uint32_t phaseIncrement) {
  uint8_t sineIndex = (uint8_t)(sstvPhase32 >> 24);
  writeSstvPllEntry(sstvSinePllIndex[sineIndex]);
  sstvPhase32 += phaseIncrement;
}

void setFastSstvDeviation(double audioDeviationHz) {
  if (!sstvPllTableReady) {
    // Safety fallback: use the known-good Etherkit path if the table was not
    // prepared for some reason.
    setFrequencyOffset(
      (double)sstvPllCarrierOffsetHz + audioDeviationHz
    );
    return;
  }

  if (audioDeviationHz > SSTV_FM_DEVIATION_HZ) {
    audioDeviationHz = SSTV_FM_DEVIATION_HZ;
  } else if (audioDeviationHz < -SSTV_FM_DEVIATION_HZ) {
    audioDeviationHz = -SSTV_FM_DEVIATION_HZ;
  }

  double normalized =
    (audioDeviationHz + SSTV_FM_DEVIATION_HZ) /
    (2.0 * SSTV_FM_DEVIATION_HZ);

  uint16_t index =
    (uint16_t)(
      normalized * (SSTV_PLL_TABLE_LEVELS - 1) + 0.5
    );

  writeSstvPllEntry(index);
}

// -----------------------------------------------------------------------------
// Existing WAV playback
// -----------------------------------------------------------------------------

uint32_t playAudio(const char* filename, int32_t carrierOffsetHz) {
  if (hostMounted) {
    return 0;
  }

  filesystemBusy = true;

  if (hostMounted) {
    filesystemBusy = false;
    return 0;
  }

  if (!openRoot()) {
    filesystemBusy = false;
    return 0;
  }

  if (!file.open(&root, filename, O_RDONLY)) {
    Serial.println("Failed to open audio file.");
    closeRoot();
    filesystemBusy = false;
    return 0;
  }

  char header[44];

  if (file.read(header, 44) != 44) {
    file.close();
    closeRoot();
    filesystemBusy = false;
    return 0;
  }

  if (strncmp(header, "RIFF", 4) != 0 ||
      strncmp(header + 8, "WAVE", 4) != 0) {
    file.close();
    closeRoot();
    filesystemBusy = false;
    return 0;
  }

#ifdef DEBUG
  uint32_t startMicros = micros();
#endif

  uint32_t numSamples = 0;
  int16_t sample;

  while (file.read((uint8_t*)&sample, 2) == 2) {
    // USB MSC sets hostMounted before modifying any FAT sectors. Close our
    // file immediately so the host can safely take over the volume.
    if (hostMounted) {
      file.close();
      closeRoot();
      filesystemBusy = false;
      return 0;
    }

    uint32_t start = micros();

    double deviation =
      carrierOffsetHz +
      (((double)sample * AUDIO_FM_DEVIATION_HZ) / 32767.0);

    setFrequencyOffset(deviation);

    numSamples++;
    delayForSampleRate(start, AUDIO_SAMPLE_RATE_HZ);
  }

#ifdef DEBUG
  uint32_t endMicros = micros();

  Serial.print("Audio total us: ");
  Serial.println(endMicros - startMicros);

  Serial.print("Audio samples: ");
  Serial.println(numSamples);

  if (numSamples) {
    Serial.print("Audio us/sample: ");
    Serial.println((double)(endMicros - startMicros) / numSamples);
  }
#endif

  file.close();
  closeRoot();
  filesystemBusy = false;

  return (numSamples * 1000UL) / AUDIO_SAMPLE_RATE_HZ;
}

// -----------------------------------------------------------------------------
// Stage 3: real-time SSTV DDS/NCO tone synthesis
// -----------------------------------------------------------------------------

/*
  Generate an AUDIO tone by continuously varying RF frequency deviation.

  Why this is necessary:
    - A constant +1900 Hz RF offset is NOT a 1900 Hz audio tone.
    - After an FM receiver, a constant offset is essentially DC.
    - To generate a 1900 Hz SSTV tone, the RF instantaneous frequency must
      swing sinusoidally at 1900 cycles/sec.

  The existing PicoFox WAV path does this indirectly from PCM samples.
  This routine does it directly with an NCO so no SSTV WAV is required.
*/
void sendSSTVTone(uint16_t toneHz,
                  uint32_t durationUs,
                  int32_t carrierOffsetHz) {
  (void)carrierOffsetHz;

  if (durationUs == 0 || hostMounted) {
    return;
  }

  uint32_t samples =
    (uint32_t)(((uint64_t)durationUs * SSTV_SAMPLE_RATE_HZ + 500000ULL) /
               1000000ULL);

  if (samples < 1) {
    samples = 1;
  }

  const uint32_t phaseIncrement =
    phaseIncrementForTone(toneHz);

  uint32_t nextDeadlineUs = micros();

  for (uint32_t i = 0; i < samples; i++) {
    if (hostMounted) {
      return;
    }

    uint32_t updateStart = micros();
    writeFastSstvSample(phaseIncrement);
    uint32_t updateUs = (uint32_t)(micros() - updateStart);

    recordSstvUpdateTime(updateUs, SSTV_SAMPLE_PERIOD_US);

    nextDeadlineUs += SSTV_SAMPLE_PERIOD_US;

    uint32_t nowUs = micros();
    if (timeBeforeUs(nowUs, nextDeadlineUs)) {
      waitForSstvDeadline(nextDeadlineUs);
    } else if (updateUs < SSTV_SAMPLE_PERIOD_US) {
      // Do NOT re-anchor the clock. A rare slow I2C transaction may overrun
      // one sample, but subsequent ~112 us writes can catch back up to the
      // original Robot36 timeline.
      sstvStats.missedDeadlines++;
    }
  }
}

// Send a scan where the desired SSTV tone may change across the scan.
// This keeps the FM synthesis running at SSTV_SAMPLE_RATE_HZ while mapping
// modulation samples onto virtual image pixels.
void resetSstvTimingStats() {
  sstvStats.updates = 0;
  sstvStats.totalUpdateUs = 0;
  sstvStats.minUpdateUs = 0xFFFFFFFFUL;
  sstvStats.maxUpdateUs = 0;
  sstvStats.missedDeadlines = 0;
  sstvStats.frameStartUs = micros();
  sstvStats.frameEndUs = sstvStats.frameStartUs;
}

void recordSstvUpdateTime(uint32_t updateUs, uint32_t sampleBudgetUs) {
  sstvStats.updates++;
  sstvStats.totalUpdateUs += updateUs;

  if (updateUs < sstvStats.minUpdateUs) {
    sstvStats.minUpdateUs = updateUs;
  }

  if (updateUs > sstvStats.maxUpdateUs) {
    sstvStats.maxUpdateUs = updateUs;
  }

  if (updateUs >= sampleBudgetUs) {
    sstvStats.missedDeadlines++;
  }
}

void reportSstvTimingStats() {
  Serial.println();
  Serial.println("---- SSTV timing diagnostics ----");

  Serial.print("I2C clock: ");
  Serial.print(I2C_CLOCK_HZ);
  Serial.println(" Hz");

  Serial.print("Audio FM deviation scale: ");
  Serial.print(AUDIO_FM_DEVIATION_HZ);
  Serial.println(" Hz");

  Serial.print("SSTV FM deviation scale: ");
  Serial.print(SSTV_FM_DEVIATION_HZ);
  Serial.println(" Hz");

  Serial.print("PLL table levels: ");
  Serial.println(SSTV_PLL_TABLE_LEVELS);

  Serial.print("Requested SSTV sample rate: ");
  Serial.print(SSTV_SAMPLE_RATE_HZ);
  Serial.println(" Hz");

  Serial.println("Realtime path: fixed-point phase + preconverted Y/chroma + PLL-index sine LUT");

  Serial.print("Exact SSTV sample period: ");
  Serial.print(SSTV_SAMPLE_PERIOD_US);
  Serial.println(" us");

  Serial.print("SSTV timing correction: ");
  Serial.println("0 us (SAMPLE_WRITE_CORR_US applies only to normal audio)");

  Serial.print("Si5351 updates measured: ");
  Serial.println(sstvStats.updates);

  if (sstvStats.updates > 0) {
    Serial.print("Average setFrequencyOffset time: ");
    Serial.print(
      (double)sstvStats.totalUpdateUs / (double)sstvStats.updates,
      2
    );
    Serial.println(" us");

    Serial.print("Minimum setFrequencyOffset time: ");
    Serial.print(sstvStats.minUpdateUs);
    Serial.println(" us");

    Serial.print("Maximum setFrequencyOffset time: ");
    Serial.print(sstvStats.maxUpdateUs);
    Serial.println(" us");
  }

  Serial.print("Missed sample deadlines: ");
  Serial.println(sstvStats.missedDeadlines);

  Serial.print("I2C PLL write errors: ");
  Serial.println(sstvPllWriteErrors);

  uint32_t frameUs =
    (uint32_t)(sstvStats.frameEndUs - sstvStats.frameStartUs);

  Serial.print("Actual Robot36 frame time: ");
  Serial.print(frameUs / 1000000.0, 3);
  Serial.println(" seconds");

  if (frameUs > 0) {
    Serial.print("Effective average update rate: ");
    Serial.print(
      ((double)sstvStats.updates * 1000000.0) / (double)frameUs,
      1
    );
    Serial.println(" updates/sec");
  }

  Serial.println("---------------------------------");
  Serial.println();
}

static inline uint16_t sstvLevelToTone(uint8_t level) {
  return SSTV_BLACK_HZ +
    (uint16_t)(((uint32_t)level *
      (SSTV_WHITE_HZ - SSTV_BLACK_HZ)) / 255UL);
}

// Built-in monochrome diagnostic source, 256 x 240.
//   Lines   0-79:  left-to-right grayscale ramp.
//   Lines  80-159: eight vertical grayscale bars.
//   Lines 160-239: 16x16-pixel black/white checkerboard.
// Send the 88 ms Robot36 luminance scan. The test image is 256 pixels wide;
// modulation samples are mapped across those 256 source values.
// -----------------------------------------------------------------------------
// Stages 7-8: file sequencing and transmission scheduler
// -----------------------------------------------------------------------------

bool flashFileExists(const char* filename) {
  if (hostMounted) {
    return false;
  }

  filesystemBusy = true;

  if (hostMounted) {
    filesystemBusy = false;
    return false;
  }

  FatFile localRoot;
  FatFile localFile;
  bool exists = false;

  if (localRoot.openRoot(&fatfs)) {
    exists = localFile.open(&localRoot, filename, O_RDONLY);
    if (exists) {
      localFile.close();
    }
    localRoot.close();
  }

  filesystemBusy = false;
  return exists;
}

// Sequence rules:
//   SSTV:  sstv.jpg, sstv1.jpg, sstv2.jpg, ...
//   Audio: audio.wav, audio1.wav, audio2.wav, ...
//
// The unnumbered file is index 0. When the next numbered file is absent,
// wrap back to the unnumbered file.
bool nextSequencedFile(const char* prefix,
                       const char* extension,
                       uint16_t* index,
                       char* outName,
                       size_t outNameSize) {
  if (!prefix || !extension || !index || !outName || outNameSize == 0) {
    return false;
  }

  // Sequence:
  //   index 0 -> sstv.jpg / audio.wav
  //   index 1 -> sstv1.jpg / audio1.wav
  //   index 2 -> sstv2.jpg / audio2.wav
  //   ...
  //
  // The first missing numbered file wraps the sequence back to the
  // unnumbered base file.

  if (*index > FILE_SEQUENCE_MAX) {
    *index = 0;
  }

  if (*index == 0) {
    snprintf(outName, outNameSize, "%s%s", prefix, extension);

    if (!flashFileExists(outName)) {
      return false;
    }

    *index = 1;
    return true;
  }

  snprintf(outName, outNameSize, "%s%u%s",
           prefix, (unsigned)*index, extension);

  if (flashFileExists(outName)) {
    (*index)++;

    if (*index > FILE_SEQUENCE_MAX) {
      *index = 0;
    }

    return true;
  }

  // Missing numbered file: wrap immediately to the base file.
  *index = 0;
  snprintf(outName, outNameSize, "%s%s", prefix, extension);

  if (!flashFileExists(outName)) {
    return false;
  }

  *index = 1;
  return true;
}

void applyDutyCycleOff(uint32_t activeLengthMs,
                       uint8_t* attenuationCycleStep) {
  if (settings.dutyCyclePercent >= 100 ||
      settings.dutyCyclePercent == 0 ||
      activeLengthMs == 0) {
    return;
  }

  uint32_t offTime =
    ((100 - settings.dutyCyclePercent) *
     activeLengthMs) / 100;

  setSi5351Output(false);

  programAttenuator(
    getAttenuationCycleValue(
      settings.attenuation,
      *attenuationCycleStep
    )
  );

  *attenuationCycleStep =
    (*attenuationCycleStep + 1) % 3;

  delay(offTime);
}

uint32_t playNextNormalTransmission(uint8_t* carrierSequenceStep,
                                    uint16_t* audioSequenceIndex) {
  if (hostMounted) {
    return 0;
  }

  char audioFilename[32];

  // Prefer audio.wav, then audio1.wav, audio2.wav, ...
  bool haveAudio =
    nextSequencedFile(
      AUDIO_SEQUENCE_PREFIX,
      AUDIO_SEQUENCE_EXT,
      audioSequenceIndex,
      audioFilename,
      sizeof(audioFilename)
    );

  if (!haveAudio) {
    strncpy(audioFilename, AUDIO_WAV, sizeof(audioFilename) - 1);
    audioFilename[sizeof(audioFilename) - 1] = '\0';
  }

  int32_t carrierOffsetHz =
    getCarrierOffsetHz(*carrierSequenceStep);

  *carrierSequenceStep =
    (*carrierSequenceStep + 1) %
    (sizeof(CARRIER_OFFSET_SEQUENCE_HZ) /
     sizeof(CARRIER_OFFSET_SEQUENCE_HZ[0]));

  setFrequencyOffset(carrierOffsetHz);
  setSi5351Output(true);

  Serial.print("Normal TX: ");
  Serial.println(audioFilename);

  uint32_t activeLengthMs =
    playAudio(audioFilename, carrierOffsetHz);

  if (!test_mode && !hostMounted) {
    carrierOffsetHz =
      getCarrierOffsetHz(*carrierSequenceStep);

    *carrierSequenceStep =
      (*carrierSequenceStep + 1) %
      (sizeof(CARRIER_OFFSET_SEQUENCE_HZ) /
       sizeof(CARRIER_OFFSET_SEQUENCE_HZ[0]));

    setFrequencyOffset(carrierOffsetHz);

    activeLengthMs +=
      playAudio(CALLSIGN_WAV, carrierOffsetHz);
  }

  return activeLengthMs;
}

uint32_t sendNextSstvTransmission(uint8_t* carrierSequenceStep,
                                  uint16_t* sstvSequenceIndex) {
  (void)carrierSequenceStep;  // SSTV does not use or advance the offset sequence.

  if (hostMounted ||
      !settings.sstvEnabled ||
      settings.sstvMode != SSTV_MODE_ROBOT36) {
    return 0;
  }

  char sstvFilename[32];

  if (!nextSequencedFile(
        SSTV_JPEG_PREFIX,
        SSTV_JPEG_EXT,
        sstvSequenceIndex,
        sstvFilename,
        sizeof(sstvFilename))) {
    Serial.println("SSTV enabled but sstv.jpg was not found.");
    return 0;
  }

  // SSTV always stays centered on the configured transmit frequency.
  // CARRIER_OFFSET_SEQUENCE_HZ remains active only for normal audio/callsign.
  const int32_t carrierOffsetHz = 0;

  Serial.print("SSTV TX at zero carrier offset: ");
  Serial.println(sstvFilename);

  uint32_t startMs = millis();
  sendRobot36Jpeg(sstvFilename, carrierOffsetHz);
  return millis() - startMs;
}

// -----------------------------------------------------------------------------
// Stage 6: JPEG decode from PicoFox FAT flash
// -----------------------------------------------------------------------------

void* jpegOpenCallback(const char* filename, int32_t* fileSize) {
  if (hostMounted) {
    return nullptr;
  }

  jpegFile.close();
  jpegRoot.close();

  if (!jpegRoot.openRoot(&fatfs)) {
    return nullptr;
  }

  if (!jpegFile.open(&jpegRoot, filename, O_RDONLY)) {
    jpegRoot.close();
    return nullptr;
  }

  if (fileSize) {
    *fileSize = (int32_t)jpegFile.fileSize();
  }

  return (void*)&jpegFile;
}

void jpegCloseCallback(void* handle) {
  (void)handle;
  jpegFile.close();
  jpegRoot.close();
}

int32_t jpegReadCallback(JPEGFILE* handle, uint8_t* buffer, int32_t length) {
  (void)handle;

  if (hostMounted || !jpegFile.isOpen()) {
    return 0;
  }

  int32_t result = jpegFile.read(buffer, length);
  return result < 0 ? 0 : result;
}

int32_t jpegSeekCallback(JPEGFILE* handle, int32_t position) {
  (void)handle;

  if (hostMounted || !jpegFile.isOpen() || position < 0) {
    return 0;
  }

  return jpegFile.seekSet((uint32_t)position) ? position : 0;
}

int jpegDrawCallback(JPEGDRAW* draw) {
  if (!draw || hostMounted) {
    return 0;
  }

  // Stage 6 v26 assumes the source JPEG is already exactly 320x240.
  // JPEGDEC gives us RGB565 blocks; copy each decoded pixel directly to the
  // matching framebuffer coordinate. No crop, no scaling, no aspect-ratio
  // conversion.
  for (int by = 0; by < draw->iHeight; by++) {
    int y = draw->y + by;

    if (y < 0 || y >= SSTV_IMAGE_HEIGHT) {
      continue;
    }

    uint16_t* dst =
      &sstvImage[(uint32_t)y * SSTV_IMAGE_WIDTH];

    for (int bx = 0; bx < draw->iWidth; bx++) {
      int x = draw->x + bx;

      if (x < 0 || x >= SSTV_IMAGE_WIDTH) {
        continue;
      }

      dst[x] = draw->pPixels[by * draw->iWidth + bx];
    }
  }

  return 1;
}

bool loadRobot36Jpeg(const char* filename) {
  if (hostMounted) {
    return false;
  }

  filesystemBusy = true;

  memset(sstvImage, 0, sizeof(sstvImage));

  Serial.print("Loading JPEG: ");
  Serial.println(filename);

  int opened = jpeg.open(
    (char*)filename,
    jpegOpenCallback,
    jpegCloseCallback,
    jpegReadCallback,
    jpegSeekCallback,
    jpegDrawCallback
  );

  if (!opened) {
    Serial.print("JPEG open failed, error ");
    Serial.println(jpeg.getLastError());
    jpeg.close();
    filesystemBusy = false;
    return false;
  }

  jpegSourceWidth = jpeg.getWidth();
  jpegSourceHeight = jpeg.getHeight();

  Serial.print("JPEG source: ");
  Serial.print(jpegSourceWidth);
  Serial.print("x");
  Serial.println(jpegSourceHeight);

  // No crop or scaling. The SSTV JPEG must already be 320x240.
  if (jpegSourceWidth != SSTV_IMAGE_WIDTH ||
      jpegSourceHeight != SSTV_IMAGE_HEIGHT) {
    Serial.println("JPEG must be exactly 320x240; decode aborted.");
    jpeg.close();
    filesystemBusy = false;
    return false;
  }

  // JPEGDEC resets pixel type when opening an image, so select RGB565 after
  // open() and before decode().
  jpeg.setPixelType(RGB565_LITTLE_ENDIAN);

  int decoded = jpeg.decode(0, 0, 0);
  int err = jpeg.getLastError();
  jpeg.close();

  filesystemBusy = false;

  if (!decoded || hostMounted) {
    Serial.print("JPEG decode failed/aborted, error ");
    Serial.println(err);
    return false;
  }

  Serial.println("JPEG decoded directly into 320x240 RGB565 buffer.");

  // Do all RGB -> Y/Cb/Cr work before RF transmission begins.
  preconvertSstvFramebuffer();

  return true;
}


uint16_t getSstvPixelSafe(uint16_t x, uint16_t line) {
  if (x >= SSTV_IMAGE_WIDTH || line >= SSTV_IMAGE_HEIGHT) {
    sstvPixelBoundsErrors++;
    if (line >= 200U) {
      sstvLastLineBoundsErrors++;
    }

    // Return black rather than reading outside the framebuffer.
    return 0x0000;
  }

  uint32_t index =
    (uint32_t)line * SSTV_IMAGE_WIDTH + x;

  if (index >=
      ((uint32_t)SSTV_IMAGE_WIDTH * SSTV_IMAGE_HEIGHT)) {
    sstvPixelBoundsErrors++;
    if (line >= 200U) {
      sstvLastLineBoundsErrors++;
    }
    return 0x0000;
  }

  return sstvImage[index];
}

static inline SstvRgb rgb565ToSstvRgb(uint16_t pixel) {
  SstvRgb rgb;

  uint8_t r5 = (pixel >> 11) & 0x1F;
  uint8_t g6 = (pixel >> 5) & 0x3F;
  uint8_t b5 = pixel & 0x1F;

  rgb.r = (uint8_t)((r5 * 255U + 15U) / 31U);
  rgb.g = (uint8_t)((g6 * 255U + 31U) / 63U);
  rgb.b = (uint8_t)((b5 * 255U + 15U) / 31U);

  return rgb;
}

void preconvertSstvFramebuffer() {
  Serial.println("Preconverting RGB565 framebuffer to packed Y/chroma...");

  for (uint16_t line = 0; line < SSTV_IMAGE_HEIGHT; line++) {
    bool oddLine = (line & 1U) != 0U;

    uint32_t rowBase =
      (uint32_t)line * SSTV_IMAGE_WIDTH;

    for (uint16_t x = 0; x < SSTV_IMAGE_WIDTH; x++) {
      SstvRgb rgb =
        rgb565ToSstvRgb(sstvImage[rowBase + x]);

      uint8_t y, cb, cr;
      robot36RgbToYCbCr(rgb, &y, &cb, &cr);

      uint8_t chroma = oddLine ? cb : cr;

      // Reuse the RGB565 buffer. After this point each 16-bit word contains:
      // bits 7:0   = Y
      // bits 15:8  = the Robot36 chroma channel for this line.
      sstvImage[rowBase + x] =
        (uint16_t)y | ((uint16_t)chroma << 8);
    }
  }

  Serial.println("SSTV framebuffer preconversion complete.");
}


// REAL-TIME CRITICAL: ROBOT36 SCAN LOOP
// Image decoding and RGB->Y/chroma conversion must already be complete before
// entering this function. Avoid Serial output or any variable-latency work.
void sendRobot36JpegChannel(
    uint16_t line,
    uint8_t channel,
    uint32_t durationUs,
    int32_t carrierOffsetHz) {
  (void)carrierOffsetHz;

  const uint32_t samples =
    (uint32_t)(((uint64_t)durationUs *
                SSTV_SAMPLE_RATE_HZ +
                500000ULL) /
               1000000ULL);

  if (line >= SSTV_IMAGE_HEIGHT) {
    sstvPixelBoundsErrors++;
    return;
  }

  const uint32_t rowBase =
    (uint32_t)line * SSTV_IMAGE_WIDTH;

  uint32_t nextDeadlineUs = micros();

  for (uint32_t i = 0; i < samples; i++) {
    if (hostMounted) {
      return;
    }

    uint16_t x =
      (uint16_t)(((uint64_t)i * SSTV_IMAGE_WIDTH) / samples);

    if (x >= SSTV_IMAGE_WIDTH) {
      x = SSTV_IMAGE_WIDTH - 1;
    }

    uint16_t packed = sstvImage[rowBase + x];

    // channel 0 = Y (low byte)
    // channel 1/2 = the already-selected line chroma (high byte)
    uint8_t level =
      (channel == 0U)
        ? (uint8_t)(packed & 0xFFU)
        : (uint8_t)(packed >> 8);

    uint32_t phaseIncrement =
      sstvTonePhaseIncrement[level];

    uint32_t updateStart = micros();
    writeFastSstvSample(phaseIncrement);
    uint32_t updateUs = (uint32_t)(micros() - updateStart);

    recordSstvUpdateTime(updateUs, SSTV_SAMPLE_PERIOD_US);

    nextDeadlineUs += SSTV_SAMPLE_PERIOD_US;

    uint32_t nowUs = micros();
    if (timeBeforeUs(nowUs, nextDeadlineUs)) {
      waitForSstvDeadline(nextDeadlineUs);
    } else if (updateUs < SSTV_SAMPLE_PERIOD_US) {
      // Keep the absolute timeline so later fast samples can recover.
      sstvStats.missedDeadlines++;
    }
  }
}

void sendRobot36Jpeg(const char* filename, int32_t carrierOffsetHz) {
  if (hostMounted) {
    return;
  }

  // Decode with RF off.
  setSi5351Output(false);

  if (!loadRobot36Jpeg(filename)) {
    Serial.print(filename); Serial.println(" not available or could not be decoded.");
    return;
  }

  if (hostMounted) {
    return;
  }

  setFrequencyOffset(carrierOffsetHz);
  setSi5351Output(true);

  Serial.println("Starting Robot36 JPEG transmission.");

  sstvPixelBoundsErrors = 0;
  sstvLastLineBoundsErrors = 0;

  sstvPhase = 0.0;
  sstvPhase32 = 0;
  resetSstvTimingStats();

  buildSstvPllTable(carrierOffsetHz);
  sendRobot36VIS(carrierOffsetHz);

  for (uint16_t line = 0;
       line < SSTV_IMAGE_HEIGHT && !hostMounted;
       line++) {

    sendSSTVTone(
      SSTV_SYNC_HZ,
      R36_SYNC_US,
      carrierOffsetHz
    );

    sendSSTVTone(
      SSTV_BLACK_HZ,
      R36_SYNC_PORCH_US,
      carrierOffsetHz
    );

    sendRobot36JpegChannel(
      line,
      0U,
      R36_Y_SCAN_US,
      carrierOffsetHz
    );

    bool oddLine = (line & 1U) != 0U;

    sendSSTVTone(
      oddLine ? SSTV_WHITE_HZ : SSTV_BLACK_HZ,
      R36_SEPARATOR_US,
      carrierOffsetHz
    );

    sendSSTVTone(
      SSTV_MID_HZ,
      R36_CHROMA_PORCH_US,
      carrierOffsetHz
    );

    sendRobot36JpegChannel(
      line,
      oddLine ? 1U : 2U,
      R36_CHROMA_SCAN_US,
      carrierOffsetHz
    );

  }

  sstvPllTableReady = false;
  setFrequencyOffset(carrierOffsetHz);

  sstvStats.frameEndUs = micros();

  Serial.println("Robot36 JPEG transmission complete.");
  Serial.print("Framebuffer bounds errors: ");
  Serial.println(sstvPixelBoundsErrors);
  Serial.print("Bounds errors on lines 200-239: ");
  Serial.println(sstvLastLineBoundsErrors);
  reportSstvTimingStats();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Four unmistakable vertical blocks:
// RED | GREEN | BLUE | WHITE
static inline uint8_t clampRobot36Byte(int32_t v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

// Full-range YCbCr conversion for Robot36 test generation.
static inline void robot36RgbToYCbCr(
    const SstvRgb& rgb,
    uint8_t* y,
    uint8_t* cb,
    uint8_t* cr) {

  const int32_t r = rgb.r;
  const int32_t g = rgb.g;
  const int32_t b = rgb.b;

  *y  = clampRobot36Byte((77 * r + 150 * g + 29 * b + 128) >> 8);
  *cb = clampRobot36Byte(128 + ((-43 * r - 85 * g + 128 * b + 128) >> 8));
  *cr = clampRobot36Byte(128 + ((128 * r - 107 * g - 21 * b + 128) >> 8));
}

// channel: 0 = Y, 1 = Cb, 2 = Cr
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void sendRobot36VIS(int32_t carrierOffsetHz) {
  // Calibration header.
  sendSSTVTone(1900, 300000UL, carrierOffsetHz);
  sendSSTVTone(1200,  10000UL, carrierOffsetHz);
  sendSSTVTone(1900, 300000UL, carrierOffsetHz);

  // VIS start bit.
  sendSSTVTone(1200, 30000UL, carrierOffsetHz);

  // Seven VIS data bits, least-significant bit first.
  uint8_t code = R36_VIS_CODE;
  uint8_t ones = 0;

  for (uint8_t bit = 0; bit < 7; bit++) {
    bool one = (code >> bit) & 0x01;

    if (one) {
      ones++;
      sendSSTVTone(SSTV_VIS_BIT_1_HZ,
                   30000UL,
                   carrierOffsetHz);
    } else {
      sendSSTVTone(SSTV_VIS_BIT_0_HZ,
                   30000UL,
                   carrierOffsetHz);
    }
  }

  // Even parity: parity bit is 1 when the seven data bits contain an odd
  // number of ones.
  bool parityOne = (ones & 1) != 0;

  sendSSTVTone(
    parityOne ? SSTV_VIS_BIT_1_HZ : SSTV_VIS_BIT_0_HZ,
    30000UL,
    carrierOffsetHz
  );

  // VIS stop bit.
  sendSSTVTone(1200, 30000UL, carrierOffsetHz);
}

// -----------------------------------------------------------------------------
// Stage 8: alternate normal transmission and SSTV
// -----------------------------------------------------------------------------

void audioTask() {
  uint8_t attenuationCycleStep = 1;
  uint8_t carrierSequenceStep = 0;
  uint16_t audioSequenceIndex = 0;
  uint16_t sstvSequenceIndex = 0;

  // When voice is enabled, preserve the normal alternating behavior:
  // NORMAL -> SSTV -> NORMAL -> SSTV ...
  bool nextIsSstv = false;

  while (true) {
    if (hostMounted) {
      setSi5351Output(false);
      break;
    }

    if (!settings.isConfigured ||
        settings.dutyCyclePercent == 0) {
      setSi5351Output(false);
      delay(1000);
      continue;
    }

    uint32_t activeLengthMs = 0;

    if (!settings.voiceEnabled) {
      // SSTV-only mode. Never play audio.wav and never send the Morse
      // callsign. SSTV files continue to cycle normally.
      if (settings.sstvEnabled &&
          settings.sstvMode == SSTV_MODE_ROBOT36) {
        activeLengthMs =
          sendNextSstvTransmission(
            &carrierSequenceStep,
            &sstvSequenceIndex
          );
      } else {
        // Both normal voice/Morse and SSTV are disabled.
        setSi5351Output(false);
        delay(1000);
        continue;
      }
    } else if (settings.sstvEnabled &&
               settings.sstvMode == SSTV_MODE_ROBOT36 &&
               nextIsSstv) {

      activeLengthMs =
        sendNextSstvTransmission(
          &carrierSequenceStep,
          &sstvSequenceIndex
        );

      nextIsSstv = false;
    } else {
      activeLengthMs =
        playNextNormalTransmission(
          &carrierSequenceStep,
          &audioSequenceIndex
        );

      nextIsSstv =
        settings.sstvEnabled &&
        settings.sstvMode == SSTV_MODE_ROBOT36;
    }

    if (hostMounted) {
      setSi5351Output(false);
      break;
    }

    applyDutyCycleOff(
      activeLengthMs,
      &attenuationCycleStep
    );
  }
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  // Serial is initialized early enough to make stage diagnostics useful.
  Serial.begin(115200);

  Serial.print("PicoFox firmware ");
  Serial.println(PICOFOX_FIRMWARE_VERSION);

  // Initialize flash.
  while (!flash.begin()) {
    Serial.println("Flash setup failed.");
    delay(1000);
  }

  // ---------------------------------------------------------------------------
  // STAGE 1 CHANGE:
  // DO NOT format flash unconditionally.
  // Mount first; format only if no valid filesystem can be mounted.
  // ---------------------------------------------------------------------------
  if (!fatfs.begin(&flash, true, 1, 0)) {
    Serial.println("No valid flash filesystem found; formatting once.");
    formatFat16();

    if (!fatfs.begin(&flash, true, 1, 0)) {
      Serial.println("Unable to mount flash after formatting.");
      while (true) {
        delay(1000);
      }
    }
  } else {
    Serial.println("Existing flash filesystem mounted; contents preserved.");
  }

  loadFlashData();

  Serial.print("VOICE_ENABLE=");
  Serial.println(settings.voiceEnabled ? 1 : 0);
  Serial.print("SSTV_ENABLE=");
  Serial.println(settings.sstvEnabled ? 1 : 0);

  if (!settings.voiceEnabled && settings.sstvEnabled) {
    Serial.println("Transmit mode: SSTV ONLY");
  } else if (settings.voiceEnabled && settings.sstvEnabled) {
    Serial.println("Transmit mode: NORMAL + SSTV alternating");
  } else if (settings.voiceEnabled) {
    Serial.println("Transmit mode: NORMAL audio/Morse only");
  } else {
    Serial.println("Transmit mode: all transmission modes disabled");
  }

  // Start with a clean SdFat cache before exposing the same block device to
  // the USB host.
  fatfs.cacheClear();

  // USB mass storage.
  usb_msc.setID("AI6YM", "PicoFox", "2.0");
  usb_msc.setReadWriteCallback(
    mscReadCb,
    mscWriteCb,
    mscFlushCb
  );
  usb_msc.setCapacity(flash.size() / 512, 512);
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

#ifdef DEBUG
  while (!Serial) {
    delay(10);
  }
#endif

  // Test mode.
  pinMode(TEST_MODE_PIN, INPUT_PULLDOWN);
  test_mode = digitalRead(TEST_MODE_PIN);

  // Board revision ID.
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
    revision = 0;
  }

  Serial.print("Revision: ");
  Serial.println(revision);

  pinMode(AMP_EN, OUTPUT);
  digitalWrite(AMP_EN, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (revision == 3) {
    programAttenuator(settings.attenuation);
  }

  startWire();

  si5351.init(
    SI5351_CRYSTAL_LOAD_10PF,
    0,
    0
  );

  if (revision == 3) {
    si5351.drive_strength(
      SI5351_CLOCK_OUTPUT,
      SI5351_DRIVE_LEVEL_R3
    );
  } else {
    si5351.drive_strength(
      SI5351_CLOCK_OUTPUT,
      SI5351_DRIVE_LEVEL_R2
    );
  }

  if (settings.isConfigured || test_mode) {
    si5351.set_int(SI5351_CLOCK_OUTPUT, 1);

    si5351.set_pll(
      (settings.transmitFreqMHz * 1e8) *
      SI5351_CLOCK_DIV,
      SI5351_PLL
    );

    si5351.set_freq_manual(
      settings.transmitFreqMHz * 1e8,
      si5351.plla_freq,
      SI5351_CLOCK_OUTPUT
    );

    si5351.pll_reset(SI5351_PLL);

    // PLL is configured, but intentionally keep RF off until the boot delay
    // has elapsed. This prevents an immediate carrier at reset/power-up.
    setSi5351Output(false);

    Serial.println("Si5351 setup complete; RF output held off.");
  }

  Serial.print("SSTV enabled: ");
  Serial.println(settings.sstvEnabled ? "yes" : "no");

  Serial.print("SSTV mode: ");
  Serial.println(
    settings.sstvMode == SSTV_MODE_ROBOT36
      ? "ROBOT36"
      : "unsupported"
  );

  // ---------------------------------------------------------------------------
  // BOOT TRANSMIT DELAY
  //
  // Keep both Si5351 output and amplifier disabled for five seconds before
  // SSTV or normal audio/Morse is allowed to transmit.
  // ---------------------------------------------------------------------------
  setSi5351Output(false);

  Serial.print("Waiting ");
  Serial.print(BOOT_TX_DELAY_MS / 1000UL);
  Serial.println(" seconds before enabling transmission...");
  delay(BOOT_TX_DELAY_MS);

  // ---------------------------------------------------------------------------
  // STAGE 4 TEST HOOK
  //
  // Stages 1-4 do not yet contain the normal/SSTV alternating scheduler.
  // To make the encoder testable now, SSTV_ENABLE=1 sends exactly one
  // built-in Robot36 monochrome diagnostic after the 5-second boot delay.
  // After that, normal operation starts.
  // ---------------------------------------------------------------------------
  if (settings.sstvEnabled &&
      settings.sstvMode == SSTV_MODE_ROBOT36 &&
      settings.isConfigured &&
      !hostMounted) {
    setSi5351Output(true);
    // SSTV transmissions are handled by audioTask().
    setSi5351Output(false);
    delay(1000);
  }

  // Existing audio/Morse behavior continues on core 1.
  multicore_launch_core1(audioTask);
}

void loop() {
  // Main core remains available for USB/TinyUSB work handled by the core.
}
