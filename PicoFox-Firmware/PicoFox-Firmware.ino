/*
  PicoFox - random attenuation debug cleanup v51
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
    - Measure Si5351 update duration, missed sample deadlines, and real frame time.
    - RF output remains disabled for 5 seconds after boot before any transmit.

  Arduino environment:
    - Earle Philhower RP2040 core
    - Etherkit Si5351
    - Adafruit SPIFlash
    - Adafruit TinyUSB
    - SdFat - Adafruit Fork
*/

/*
  RTTTL support in this firmware was inspired by and adapted from ideas in
  EvilMog's midijunk project:

  https://github.com/evilmog/evilmog/tree/master/midijunk

  Credit: EvilMog
*/



#include <Arduino.h>
#include <Wire.h>
#include <si5351.h>
#include "SPI.h"
#include "SdFat_Adafruit_Fork.h"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"
#include <JPEGDEC.h>


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

#define AUDIO_SAMPLE_RATE_HZ 8000UL
#define AUDIO_FM_DEVIATION_HZ 5000
#define AUDIO_SAMPLE_PERIOD_US (1000000UL / AUDIO_SAMPLE_RATE_HZ)

// RF sequencing delays.
// Allow the transmitter/amplifier to settle before modulation begins.
#define TX_KEYUP_DELAY_MS 100UL

// Silent keyed-carrier gap between WAV/RTTTL audio and the live Morse ID.
#define AUDIO_TO_MORSE_GAP_MS 1000UL
#define SSTV_FM_DEVIATION_HZ 3000

#define SI5351_PLL SI5351_PLLA
#define SI5351_CLOCK_OUTPUT SI5351_CLK0
#define SI5351_PLL_HZ 900000000
#define SI5351_CLOCK_DIV 6

#define SI5351_DRIVE_LEVEL_R2 SI5351_DRIVE_2MA
#define SI5351_DRIVE_LEVEL_R3 SI5351_DRIVE_2MA

#define TEST_MODE_PIN 14u

#define MIN_FREQ_MHZ 144
#define MAX_FREQ_MHZ_ITU1 146
#define MAX_FREQ_MHZ 148

// -----------------------------------------------------------------------------
// SSTV stage 1-4 constants
// -----------------------------------------------------------------------------

// SSTV and normal audio now both use an 8 kHz modulation update rate.
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
#define AUDIO_PLL_TABLE_LEVELS 512
#define SI5351_I2C_ADDRESS 0x60
#define SI5351_PLLA_PARAMETERS_REG 26
#define SI5351_RFRAC_DENOM 1000000ULL
#define SI5351_REF_FREQ_X100 2500000000ULL

#define SSTV_MODE_ROBOT36 36
#define PICOFOX_FIRMWARE_VERSION "v51"
#define AUDIO_MODE_WAV 0
#define AUDIO_MODE_RTTTL 1

#define ATTENUATION_MODE_FIXED 0
#define ATTENUATION_MODE_RANDOM 1
#define DEFAULT_AUDIO_MODE AUDIO_MODE_WAV
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

const char SSTV_JPEG_FILE[] = "sstv.jpg";
const char SSTV_JPEG_PREFIX[] = "sstv";
const char SSTV_JPEG_EXT[] = ".jpg";
const char AUDIO_SEQUENCE_PREFIX[] = "audio";
const char AUDIO_SEQUENCE_EXT[] = ".wav";
const char SONGS_TXT[] = "songs.txt";
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
const uint16_t DEFAULT_MORSE_TONE_HZ = 600;
const uint8_t DEFAULT_TONE_AMPLITUDE_PERCENT = 70;
const uint8_t DEFAULT_ATTENUATION = 0;
const uint8_t DEFAULT_ATTENUATION_MODE = ATTENUATION_MODE_FIXED;
const uint8_t DEFAULT_ATTENUATION_MIN = 0;
const uint8_t DEFAULT_ATTENUATION_MAX = 127;

// Built-in fallback tune. Replaces the old embedded PCM audio.h waveform.
const char DEFAULT_RTTTL[] =
  "PicoFox:d=4,o=4,b=150:"
  "g,b,g,b,g,b,g,b,g,b,g,b,"
  "g,b,g,b,g,b,g,b,g,b,g,b,"
  "g,b,g,b,g,b,g,b,g,b,g,b";

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

struct AudioPllEntry {
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
  uint16_t morseToneHz;
  uint8_t toneAmplitudePercent;
  bool isConfigured;

  // Attenuator configuration. Values are limited to 0..127.
  uint8_t attenuation;
  uint8_t attenuationMode;
  uint8_t attenuationMin;
  uint8_t attenuationMax;

  // Normal audio.wav + Morse callsign transmission.
  // Set VOICE_ENABLE=0 for SSTV-only operation.
  bool voiceEnabled;
  uint8_t audioMode;

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
  .morseToneHz = DEFAULT_MORSE_TONE_HZ,
  .toneAmplitudePercent = DEFAULT_TONE_AMPLITUDE_PERCENT,
  .isConfigured = false,
  .attenuation = DEFAULT_ATTENUATION,
  .attenuationMode = DEFAULT_ATTENUATION_MODE,
  .attenuationMin = DEFAULT_ATTENUATION_MIN,
  .attenuationMax = DEFAULT_ATTENUATION_MAX,
  .voiceEnabled = DEFAULT_VOICE_ENABLE,
  .audioMode = DEFAULT_AUDIO_MODE,
  .sstvEnabled = DEFAULT_SSTV_ENABLE,
  .sstvMode = DEFAULT_SSTV_MODE
};


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

// Separate 8 kHz normal-audio modulation state. This deliberately does not
// share the SSTV lookup tables, preserving the known-good SSTV core.
AudioPllEntry audioPllTable[AUDIO_PLL_TABLE_LEVELS];
uint16_t audioSinePllIndex[256];
uint32_t audioPhase32 = 0;
int32_t audioPllCarrierOffsetHz = 0;
uint32_t audioPllWriteErrors = 0;

// State used only by RANDOM attenuation mode.
uint8_t lastRandomAttenuation = 0xFF;
uint32_t attenuationRandomState = 0x6D2B79F5UL;

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------

void formatFat16();
void saveDefaultSettings();
void ensureSettingsComments();
void setSi5351Output(bool enabled);
void setFrequencyOffset(double deviation);
void programAttenuator(uint8_t attenuation);
uint32_t nextAttenuationRandom();
uint8_t selectAttenuationForTransmit();
uint32_t playAudio(const char* filename, int32_t carrierOffsetHz);
uint32_t playRtttl(const char* rtttl, int32_t carrierOffsetHz);
uint32_t sendMorseCallsign(int32_t carrierOffsetHz);
bool buildAudioPllTable(int32_t carrierOffsetHz);
bool writeAudioPllEntry(uint16_t index);
void buildAudioSineTable(uint8_t amplitudePercent);
uint32_t audioPhaseIncrementForTone(uint16_t toneHz);
uint32_t sendAudioTone(uint16_t toneHz, uint32_t durationUs, uint8_t amplitudePercent);
void sendAudioSilence(uint32_t durationUs);
void audioTask();

// SSTV stages 3-4.
void sendSSTVTone(uint16_t toneHz, uint32_t durationUs, int32_t carrierOffsetHz);
void sendRobot36VIS(int32_t carrierOffsetHz);
void sendRobot36Jpeg(const char* filename, int32_t carrierOffsetHz);
uint16_t getSstvPixelSafe(uint16_t x, uint16_t line);
bool flashFileExists(const char* filename);
void ensureSongsFilePresent();
bool getNextRtttlSong(uint16_t* songIndex, String* outSong);
bool readFatFileLine(FatFile* f, String* outLine);
bool nextSequencedFile(const char* prefix,
                       const char* extension,
                       uint16_t* index,
                       char* outName,
                       size_t outNameSize);
uint32_t playNextNormalTransmission(uint8_t* carrierSequenceStep,
                                    uint16_t* audioSequenceIndex,
                                    uint16_t* songSequenceIndex);
uint32_t sendNextSstvTransmission(uint8_t* carrierSequenceStep,
                                  uint16_t* sstvSequenceIndex);
void applyDutyCycleOff(uint32_t activeLengthMs);
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
  char buffer[3072];

  snprintf(
    buffer,
    sizeof(buffer),
    "# Alphanumeric callsign, maximum 12 characters.\n"
    "CALLSIGN=%s\n"
    "\n"
    "# ITU zone where the transmitter operates. Valid values: 1, 2, or 3.\n"
    "ITU_ZONE=%u\n"
    "\n"
    "# Transmit frequency in MHz.\n"
    "FREQ_MHZ=%.6f\n"
    "\n"
    "# Transmit duty cycle percentage, valid range 0 to 100.\n"
    "# If SSTV mode is enabled, this must be less than 80.\n"
    "DUTY_CYCLE=%u\n"
    "\n"
    "# Attenuation mode: FIXED uses the ATTENUATION setting; RANDOM selects a random value each TX.\n"
    "ATTENUATION_MODE=FIXED\n"
    "\n"
    "# ATTENUATION setting used when ATTENUATION_MODE=FIXED. Valid range: 0 to 127, approximately 0.25 dB per step.\n"
    "ATTENUATION=%u\n"
    "\n"
    "# ATTENUATION_MIN is the lowest value used in RANDOM mode. Valid range: 0 to 127, approximately 0.25 dB per step.\n"
    "# ATTENUATION_MIN must be less than ATTENUATION_MAX.\n"
    "ATTENUATION_MIN=%u\n"
    "\n"
    "# ATTENUATION_MAX is the highest value used in RANDOM mode. Valid range: 0 to 127, approximately 0.25 dB per step.\n"
    "# ATTENUATION_MAX must be greater than ATTENUATION_MIN.\n"
    "ATTENUATION_MAX=%u\n"
    "\n"
    "# Morse ID speed in words per minute.\n"
    "MORSE_WPM=%u\n"
    "\n"
    "# Morse tone frequency in Hz. Valid range: 100 to 2000 Hz.\n"
    "MORSE_TONE=%u\n"
    "\n"
    "# Morse tone volume percentage. Valid range: 1 to 100.\n"
    "MORSE_TONE_VOL=%u\n"
    "\n"
    "# Enables the Voice/Audio file and Morse code mode. 1=enabled, 0=disabled.\n"
    "VOICE_ENABLE=%u\n"
    "\n"
    "# Selects the normal audio source. Valid values: WAV or RTTTL.\n"
    "# WAV uses audio.wav, audio1.wav, audio2.wav, etc.\n"
    "# RTTTL uses one song per line from songs.txt and rotates through the list.\n"
    "# RTTTL format and example songs:\n"
    "# https://1j01.github.io/rtttl.js/\n"
    "# https://github.com/neverfa11ing/FlipperMusicRTTTL\n"
    "AUDIO_MODE=WAV\n"
    "\n"
    "# Enables SSTV mode. 1=enabled, 0=disabled.\n"
    "# You MUST set DUTY_CYCLE to less than 80 for SSTV mode to work correctly.\n"
    "SSTV_ENABLE=%u\n"
    "\n"
    "# SSTV encoding mode. Currently ONLY ROBOT36 is supported.\n"
    "SSTV_MODE=ROBOT36\n",
    DEFAULT_CALLSIGN,
    DEFAULT_ITU_ZONE,
    DEFAULT_FREQ_MHZ,
    DEFAULT_DUTY_CYCLE,
    DEFAULT_ATTENUATION,
    DEFAULT_ATTENUATION_MIN,
    DEFAULT_ATTENUATION_MAX,
    DEFAULT_WPM,
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

void ensureSstvSettingsPresent() {
  if (!openRoot()) {
    return;
  }

  if (!file.open(&root, SETTINGS_TXT, O_RDONLY)) {
    closeRoot();
    return;
  }

  bool haveVoiceEnable = false;
  bool haveAudioMode = false;
  bool haveAttenuationMode = false;
  bool haveAttenuationMin = false;
  bool haveAttenuationMax = false;
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
      } else if (check.startsWith("AUDIO_MODE=")) {
        haveAudioMode = true;
      } else if (check.startsWith("ATTENUATION_MODE=")) {
        haveAttenuationMode = true;
      } else if (check.startsWith("ATTENUATION_MIN=")) {
        haveAttenuationMin = true;
      } else if (check.startsWith("ATTENUATION_MAX=")) {
        haveAttenuationMax = true;
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
    } else if (check.startsWith("AUDIO_MODE=")) {
      haveAudioMode = true;
    } else if (check.startsWith("ATTENUATION_MODE=")) {
      haveAttenuationMode = true;
    } else if (check.startsWith("ATTENUATION_MIN=")) {
      haveAttenuationMin = true;
    } else if (check.startsWith("ATTENUATION_MAX=")) {
      haveAttenuationMax = true;
    } else if (check.startsWith("SSTV_ENABLE=")) {
      haveEnable = true;
    } else if (check.startsWith("SSTV_MODE=")) {
      haveMode = true;
    }
  }

  file.close();

  if (!haveVoiceEnable || !haveAudioMode || !haveAttenuationMode || !haveAttenuationMin || !haveAttenuationMax || !haveEnable || !haveMode) {
    if (file.open(&root, SETTINGS_TXT, O_RDWR | O_AT_END)) {
      // Ensure appended keys begin on a fresh line even if the old file did
      // not end with a newline.
      file.write("\n", 1);

      if (!haveVoiceEnable) {
        const char voiceLine[] = "VOICE_ENABLE=1\n";
        file.write(voiceLine, sizeof(voiceLine) - 1);
      }
      if (!haveAudioMode) {
        const char audioModeLine[] =
          "# Normal audio source: WAV uses audio.wav/audio1.wav/...; RTTTL uses songs.txt.\n"
          "AUDIO_MODE=WAV\n";
        file.write(audioModeLine, sizeof(audioModeLine) - 1);
      }

      if (!haveAttenuationMode) {
        const char modeLine[] =
          "# Attenuation mode: FIXED uses the ATTENUATION setting; RANDOM selects a random value each TX.\n"
          "ATTENUATION_MODE=FIXED\n";
        file.write(modeLine, sizeof(modeLine) - 1);
      }
      if (!haveAttenuationMin) {
        const char minLine[] =
          "# Minimum random attenuator value when ATTENUATION_MODE=RANDOM. Valid range: 0 to 127.\n"
          "ATTENUATION_MIN=0\n";
        file.write(minLine, sizeof(minLine) - 1);
      }
      if (!haveAttenuationMax) {
        const char maxLine[] =
          "# Maximum random attenuator value when ATTENUATION_MODE=RANDOM. Valid range: 0 to 127.\n"
          "ATTENUATION_MAX=127\n";
        file.write(maxLine, sizeof(maxLine) - 1);
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
      Serial.println("Added missing voice/audio/attenuation/SSTV settings to settings.txt");
    }
  }

  closeRoot();
}


bool readFatFileLine(FatFile* f, String* outLine) {
  if (!f || !outLine) {
    return false;
  }

  outLine->remove(0);

  bool gotAny = false;
  int c;

  while ((c = f->read()) >= 0) {
    gotAny = true;

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      break;
    }

    *outLine += (char)c;
  }

  return gotAny;
}

void ensureSongsFilePresent() {
  if (!openRoot()) return;
  if (!root.exists(SONGS_TXT)) {
    if (file.open(&root, SONGS_TXT, O_RDWR | O_CREAT | O_TRUNC)) {
      file.write((const uint8_t*)DEFAULT_RTTTL, strlen(DEFAULT_RTTTL));
      file.write((const uint8_t*)"\n", 1);
      file.close();
      Serial.println("Created songs.txt with default PicoFox RTTTL tune.");
    }
  }
  closeRoot();
}

bool getNextRtttlSong(uint16_t* songIndex, String* outSong) {
  if (!songIndex || !outSong || hostMounted) return false;
  filesystemBusy = true;
  if (!openRoot()) { filesystemBusy=false; return false; }
  if (!file.open(&root, SONGS_TXT, O_RDONLY)) {
    closeRoot(); filesystemBusy=false; return false;
  }

  uint16_t n=0;
  String line;
  while (file.available()) {
    if (!readFatFileLine(&file, &line)) break;
    line.trim();
    if (!line.length() || line[0]=='#') continue;
    if (n==*songIndex) {
      *outSong=line;
      (*songIndex)++;
      file.close(); closeRoot(); filesystemBusy=false;
      return true;
    }
    n++;
  }

  file.rewind();
  while (file.available()) {
    if (!readFatFileLine(&file, &line)) break;
    line.trim();
    if (!line.length() || line[0]=='#') continue;
    *outSong=line;
    *songIndex=1;
    file.close(); closeRoot(); filesystemBusy=false;
    return true;
  }

  file.close(); closeRoot(); filesystemBusy=false;
  return false;
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
    Serial.println("No audio.wav found; built-in RTTTL fallback will be used.");
  }

  closeRoot();

  // Existing PicoFox installations already have settings.txt, so add the new
  // stage-2 keys without destroying the user's current configuration.
  ensureSstvSettingsPresent();
  ensureSongsFilePresent();
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

  int c;
  while ((c = file.read()) >= 0) {
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
        } else if (key == "ATTENUATION_MODE") {
          val.toUpperCase();
          settings.attenuationMode =
            ((val == "RANDOM") || (val == "AUTO")) ? ATTENUATION_MODE_RANDOM : ATTENUATION_MODE_FIXED;
        } else if (key == "ATTENUATION") {
          settings.attenuation = val.toInt();
        } else if (key == "ATTENUATION_MIN") {
          settings.attenuationMin = val.toInt();
        } else if (key == "ATTENUATION_MAX") {
          settings.attenuationMax = val.toInt();
        } else if (key == "MORSE_WPM") {
          settings.morseWPM = val.toInt();
        } else if (key == "MORSE_TONE") {
          settings.morseToneHz = val.toInt();
        } else if (key == "MORSE_TONE_VOL") {
          settings.toneAmplitudePercent = val.toInt();
        } else if (key == "VOICE_ENABLE") {
          settings.voiceEnabled = (val.toInt() != 0);
        } else if (key == "AUDIO_MODE") {
          val.toUpperCase();
          settings.audioMode = (val == "RTTTL") ? AUDIO_MODE_RTTTL : AUDIO_MODE_WAV;
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

      if (key == "ATTENUATION_MODE") {
        val.toUpperCase();
        settings.attenuationMode =
          ((val == "RANDOM") || (val == "AUTO")) ? ATTENUATION_MODE_RANDOM : ATTENUATION_MODE_FIXED;
      } else if (key == "ATTENUATION") {
        settings.attenuation = val.toInt();
      } else if (key == "ATTENUATION_MIN") {
        settings.attenuationMin = val.toInt();
      } else if (key == "ATTENUATION_MAX") {
        settings.attenuationMax = val.toInt();
      } else if (key == "VOICE_ENABLE") {
        settings.voiceEnabled = (val.toInt() != 0);
      } else if (key == "AUDIO_MODE") {
        val.toUpperCase();
        settings.audioMode = (val == "RTTTL") ? AUDIO_MODE_RTTTL : AUDIO_MODE_WAV;
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
  if (settings.attenuationMin > 127) {
    settings.attenuationMin = 127;
  }
  if (settings.attenuationMax > 127) {
    settings.attenuationMax = 127;
  }
  // RANDOM attenuation requires ATTENUATION_MIN <= ATTENUATION_MAX.
  // If the values are reversed in settings.txt, normalize them automatically.
  if (settings.attenuationMin > settings.attenuationMax) {
    uint8_t temp = settings.attenuationMin;
    settings.attenuationMin = settings.attenuationMax;
    settings.attenuationMax = temp;
  }

  if (settings.morseWPM == 0 || settings.morseWPM > 30) {
    settings.morseWPM = DEFAULT_WPM;
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

void ensureSettingsComments() {
  if (!openRoot()) {
    return;
  }

  if (!file.open(&root, SETTINGS_TXT, O_RDONLY)) {
    closeRoot();
    return;
  }

  bool currentComments = false;
  String line;

  while (readFatFileLine(&file, &line)) {
    line.trim();
    if (line == "# Alphanumeric callsign, maximum 12 characters.") {
      currentComments = true;
      break;
    }
  }

  file.close();

  if (currentComments) {
    closeRoot();
    return;
  }

  char buffer[3072];

  snprintf(
    buffer,
    sizeof(buffer),
    "# Alphanumeric callsign, maximum 12 characters.\n"
    "CALLSIGN=%s\n"
    "\n"
    "# ITU zone where the transmitter operates. Valid values: 1, 2, or 3.\n"
    "ITU_ZONE=%u\n"
    "\n"
    "# Transmit frequency in MHz.\n"
    "FREQ_MHZ=%.6f\n"
    "\n"
    "# Transmit duty cycle percentage, valid range 0 to 100.\n"
    "# If SSTV mode is enabled, this must be less than 80.\n"
    "DUTY_CYCLE=%u\n"
    "\n"
    "# Attenuation mode: FIXED uses the ATTENUATION setting; RANDOM selects a random value each TX.\n"
    "ATTENUATION_MODE=%s\n"
    "\n"
    "# ATTENUATION setting used when ATTENUATION_MODE=FIXED. Valid range: 0 to 127, approximately 0.25 dB per step.\n"
    "ATTENUATION=%u\n"
    "\n"
    "# ATTENUATION_MIN is the lowest value used in RANDOM mode. Valid range: 0 to 127, approximately 0.25 dB per step.\n"
    "# ATTENUATION_MIN must be less than ATTENUATION_MAX.\n"
    "ATTENUATION_MIN=%u\n"
    "\n"
    "# ATTENUATION_MAX is the highest value used in RANDOM mode. Valid range: 0 to 127, approximately 0.25 dB per step.\n"
    "# ATTENUATION_MAX must be greater than ATTENUATION_MIN.\n"
    "ATTENUATION_MAX=%u\n"
    "\n"
    "# Morse ID speed in words per minute.\n"
    "MORSE_WPM=%u\n"
    "\n"
    "# Morse tone frequency in Hz. Valid range: 100 to 2000 Hz.\n"
    "MORSE_TONE=%u\n"
    "\n"
    "# Morse tone volume percentage. Valid range: 1 to 100.\n"
    "MORSE_TONE_VOL=%u\n"
    "\n"
    "# Enables the Voice/Audio file and Morse code mode. 1=enabled, 0=disabled.\n"
    "VOICE_ENABLE=%u\n"
    "\n"
    "# Selects the normal audio source. Valid values: WAV or RTTTL.\n"
    "# WAV uses audio.wav, audio1.wav, audio2.wav, etc.\n"
    "# RTTTL uses one song per line from songs.txt and rotates through the list.\n"
    "# RTTTL format and example songs:\n"
    "# https://1j01.github.io/rtttl.js/\n"
    "# https://github.com/neverfa11ing/FlipperMusicRTTTL\n"
    "AUDIO_MODE=%s\n"
    "\n"
    "# Enables SSTV mode. 1=enabled, 0=disabled.\n"
    "# You MUST set DUTY_CYCLE to less than 80 for SSTV mode to work correctly.\n"
    "SSTV_ENABLE=%u\n"
    "\n"
    "# SSTV encoding mode. Currently ONLY ROBOT36 is supported.\n"
    "SSTV_MODE=ROBOT36\n",
    settings.callsign,
    settings.ituZone,
    settings.transmitFreqMHz,
    settings.dutyCyclePercent,
    settings.attenuationMode == ATTENUATION_MODE_RANDOM ? "RANDOM" : "FIXED",
    settings.attenuation,
    settings.attenuationMin,
    settings.attenuationMax,
    settings.morseWPM,
    settings.morseToneHz,
    settings.toneAmplitudePercent,
    settings.voiceEnabled ? 1 : 0,
    settings.audioMode == AUDIO_MODE_RTTTL ? "RTTTL" : "WAV",
    settings.sstvEnabled ? 1 : 0
  );

  if (file.open(&root, SETTINGS_TXT, O_RDWR | O_CREAT | O_TRUNC)) {
    file.write(buffer, strlen(buffer));
    file.close();
    Serial.println("Updated settings.txt with configuration comments.");
  }

  closeRoot();
}

// -----------------------------------------------------------------------------
// Morse generation
// -----------------------------------------------------------------------------

void loadFlashData() {
  flashCleanup();
  loadSettings();
  ensureSettingsComments();
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

uint32_t nextAttenuationRandom() {
  // xorshift32 gives a simple, fast PRNG with a full 32-bit state. This avoids
  // relying on core-specific Arduino random() behavior for attenuation.
  uint32_t x = attenuationRandomState;

  if (x == 0) {
    x = 0x6D2B79F5UL;
  }

  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;

  attenuationRandomState = x;
  return x;
}

uint8_t selectAttenuationForTransmit() {
  uint8_t selected = settings.attenuation;

  if (settings.attenuationMode == ATTENUATION_MODE_RANDOM) {
    uint8_t minValue = settings.attenuationMin;
    uint8_t maxValue = settings.attenuationMax;

    // Settings validation normally guarantees this already, but keep the TX
    // path safe even if values are changed later.
    if (minValue > maxValue) {
      uint8_t temp = minValue;
      minValue = maxValue;
      maxValue = temp;
    }

    if (minValue == maxValue) {
      selected = minValue;
    } else {
      // Inclusive range. For 0..127 this produces a span of 128 values.
      uint16_t span =
        (uint16_t)maxValue - (uint16_t)minValue + 1U;

      selected =
        (uint8_t)(
          (uint16_t)minValue +
          (nextAttenuationRandom() % span)
        );

      // RANDOM mode should visibly change each TX whenever the range contains
      // at least two values.
      if (selected == lastRandomAttenuation) {
        uint16_t offset =
          ((uint16_t)selected - (uint16_t)minValue + 1U) % span;
        selected = (uint8_t)((uint16_t)minValue + offset);
      }
    }

    lastRandomAttenuation = selected;
  }

  programAttenuator(selected);

  Serial.print("TX attenuation: ");
  Serial.print(selected);
  Serial.print(" (");
  Serial.print(
    settings.attenuationMode == ATTENUATION_MODE_RANDOM ? "RANDOM" : "FIXED"
  );
  Serial.println(")");

  return selected;
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
  uint32_t sampleDelta = 1000000UL / sampleRateHz;

  while ((uint32_t)(micros() - startUs) < sampleDelta) {
    // busy wait for consistent modulation timing
  }
}


// Robot36 timing uses its own absolute 8 kHz clock so per-sample execution
// overhead cannot accumulate into line slant.
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
// 8 kHz normal-audio / RTTTL / live-Morse synthesizer
// -----------------------------------------------------------------------------

bool buildAudioPllTable(int32_t carrierOffsetHz) {
  audioPllCarrierOffsetHz = carrierOffsetHz;
  audioPllWriteErrors = 0;

  for (uint16_t level = 0; level < AUDIO_PLL_TABLE_LEVELS; level++) {
    int32_t audioDeviationHz =
      -AUDIO_FM_DEVIATION_HZ +
      (int32_t)(((int64_t)level * (2LL * AUDIO_FM_DEVIATION_HZ)) /
                (AUDIO_PLL_TABLE_LEVELS - 1));

    calculateSstvPllRegisters(
      carrierOffsetHz + audioDeviationHz,
      audioPllTable[level].reg
    );
  }

  return true;
}

bool writeAudioPllEntry(uint16_t index) {
  if (index >= AUDIO_PLL_TABLE_LEVELS) {
    index = AUDIO_PLL_TABLE_LEVELS - 1;
  }

  Wire.beginTransmission(SI5351_I2C_ADDRESS);
  Wire.write((uint8_t)SI5351_PLLA_PARAMETERS_REG);
  for (uint8_t i = 0; i < 8; i++) {
    Wire.write(audioPllTable[index].reg[i]);
  }

  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    audioPllWriteErrors++;
    return false;
  }
  return true;
}

void buildAudioSineTable(uint8_t amplitudePercent) {
  if (amplitudePercent > 100) amplitudePercent = 100;

  for (uint16_t i = 0; i < 256; i++) {
    double phase = (2.0 * PI * (double)i) / 256.0;
    double deviation =
      sin(phase) * (double)AUDIO_FM_DEVIATION_HZ *
      ((double)amplitudePercent / 100.0);

    double normalized =
      (deviation + AUDIO_FM_DEVIATION_HZ) /
      (2.0 * AUDIO_FM_DEVIATION_HZ);

    uint16_t tableIndex =
      (uint16_t)(normalized * (AUDIO_PLL_TABLE_LEVELS - 1) + 0.5);

    if (tableIndex >= AUDIO_PLL_TABLE_LEVELS) {
      tableIndex = AUDIO_PLL_TABLE_LEVELS - 1;
    }

    audioSinePllIndex[i] = tableIndex;
  }
}

uint32_t audioPhaseIncrementForTone(uint16_t toneHz) {
  return (uint32_t)(
    (((uint64_t)toneHz << 32) + (AUDIO_SAMPLE_RATE_HZ / 2)) /
    AUDIO_SAMPLE_RATE_HZ
  );
}

static inline bool timeBeforeAudioUs(uint32_t nowUs, uint32_t deadlineUs) {
  return (int32_t)(nowUs - deadlineUs) < 0;
}

static inline void waitForAudioDeadline(uint32_t deadlineUs) {
  while (timeBeforeAudioUs(micros(), deadlineUs)) {
    tight_loop_contents();
  }
}

uint32_t sendAudioTone(uint16_t toneHz,
                       uint32_t durationUs,
                       uint8_t amplitudePercent) {
  if (toneHz == 0 || durationUs == 0 || hostMounted) return 0;

  buildAudioSineTable(amplitudePercent);

  const uint32_t phaseIncrement = audioPhaseIncrementForTone(toneHz);
  uint32_t samples =
    (uint32_t)(((uint64_t)durationUs * AUDIO_SAMPLE_RATE_HZ + 500000ULL) /
               1000000ULL);
  if (samples < 1) samples = 1;

  uint32_t nextDeadlineUs = micros();

  for (uint32_t i = 0; i < samples; i++) {
    if (hostMounted) break;

    uint8_t sineIndex = (uint8_t)(audioPhase32 >> 24);
    writeAudioPllEntry(audioSinePllIndex[sineIndex]);
    audioPhase32 += phaseIncrement;

    nextDeadlineUs += AUDIO_SAMPLE_PERIOD_US;
    uint32_t nowUs = micros();

    if (timeBeforeAudioUs(nowUs, nextDeadlineUs)) {
      waitForAudioDeadline(nextDeadlineUs);
    }
  }

  return (samples * 1000UL) / AUDIO_SAMPLE_RATE_HZ;
}

void sendAudioSilence(uint32_t durationUs) {
  if (durationUs == 0 || hostMounted) return;

  writeAudioPllEntry((AUDIO_PLL_TABLE_LEVELS - 1) / 2);

  uint32_t deadlineUs = micros() + durationUs;
  while (timeBeforeAudioUs(micros(), deadlineUs)) {
    if (hostMounted) return;
    tight_loop_contents();
  }
}

static uint16_t rtttlNoteFrequency(uint8_t semitone, uint8_t octave) {
  int midiNote = 12 * ((int)octave + 1) + semitone;
  double hz = 440.0 * pow(2.0, ((double)midiNote - 69.0) / 12.0);
  if (hz < 1.0) return 1;
  if (hz > 3999.0) return 3999;
  return (uint16_t)(hz + 0.5);
}

static void skipRtttlSpaces(const char*& p) {
  while (*p == ' ' || *p == '\t') p++;
}

static uint16_t parseRtttlNumber(const char*& p) {
  uint16_t value = 0;
  while (*p >= '0' && *p <= '9') {
    value = (uint16_t)(value * 10U + (uint16_t)(*p - '0'));
    p++;
  }
  return value;
}

uint32_t playRtttl(const char* rtttl, int32_t carrierOffsetHz) {
  if (!rtttl || !*rtttl || hostMounted) return 0;

  uint16_t defaultDuration = 4;
  uint8_t defaultOctave = 6;
  uint16_t bpm = 63;

  const char* p = strchr(rtttl, ':');
  if (!p) {
    Serial.println("Invalid RTTTL: missing defaults separator.");
    return 0;
  }
  p++;

  while (*p && *p != ':') {
    skipRtttlSpaces(p);

    char key = (char)tolower((unsigned char)*p);
    if (*p) p++;

    if (*p == '=') {
      p++;
      uint16_t value = parseRtttlNumber(p);

      if (key == 'd' && value > 0) defaultDuration = value;
      else if (key == 'o' && value >= 3 && value <= 7) defaultOctave = (uint8_t)value;
      else if (key == 'b' && value >= 25 && value <= 900) bpm = value;
    }

    while (*p && *p != ',' && *p != ':') p++;
    if (*p == ',') p++;
  }

  if (*p != ':') {
    Serial.println("Invalid RTTTL: missing note separator.");
    return 0;
  }
  p++;

  buildAudioPllTable(carrierOffsetHz);
  audioPhase32 = 0;

  uint32_t totalMs = 0;
  const uint32_t wholeNoteUs = 240000000UL / bpm;

  while (*p && !hostMounted) {
    skipRtttlSpaces(p);
    if (!*p) break;

    uint16_t durationDivisor = parseRtttlNumber(p);
    if (durationDivisor == 0) durationDivisor = defaultDuration;

    char noteChar = (char)tolower((unsigned char)*p);
    if (*p) p++;

    bool rest = (noteChar == 'p');
    int8_t semitone = -1;

    switch (noteChar) {
      case 'c': semitone = 0; break;
      case 'd': semitone = 2; break;
      case 'e': semitone = 4; break;
      case 'f': semitone = 5; break;
      case 'g': semitone = 7; break;
      case 'a': semitone = 9; break;
      case 'b': semitone = 11; break;
      case 'p': break;
      default:
        while (*p && *p != ',') p++;
        if (*p == ',') p++;
        continue;
    }

    if (!rest && *p == '#') {
      semitone++;
      if (semitone > 11) semitone = 0;
      p++;
    }

    bool dotted = false;
    if (*p == '.') {
      dotted = true;
      p++;
    }

    uint16_t explicitOctave = parseRtttlNumber(p);
    uint8_t octave = explicitOctave ? (uint8_t)explicitOctave : defaultOctave;

    if (*p == '.') {
      dotted = true;
      p++;
    }

    uint32_t durationUs = wholeNoteUs / durationDivisor;
    if (dotted) durationUs += durationUs / 2;

    if (rest) {
      sendAudioSilence(durationUs);
      totalMs += durationUs / 1000UL;
    } else {
      uint16_t toneHz = rtttlNoteFrequency((uint8_t)semitone, octave);
      totalMs += sendAudioTone(toneHz, durationUs, 100);
    }

    while (*p && *p != ',') p++;
    if (*p == ',') p++;
  }

  return totalMs;
}

uint32_t sendMorseCallsign(int32_t carrierOffsetHz) {
  if (hostMounted || !settings.callsign[0]) return 0;

  buildAudioPllTable(carrierOffsetHz);
  audioPhase32 = 0;

  const char* morseTable[36] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",
    ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.",
    "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..",
    "-----", ".----", "..---", "...--", "....-",
    ".....", "-....", "--...", "---..", "----."
  };

  auto getMorse = [&](char c) -> const char* {
    if (c >= 'a' && c <= 'z') c -= 32;

    if (c >= 'A' && c <= 'Z') {
      return morseTable[c - 'A'];
    }

    if (c >= '0' && c <= '9') {
      return morseTable[c - '0' + 26];
    }

    if (c == '/') {
      return "-..-.";
    }

    return "";
  };

  // Standard PARIS timing:
  //   dot             = 1 unit
  //   dash            = 3 units
  //   element gap     = 1 unit
  //   character gap   = 3 units
  //   word gap        = 7 units
  //
  // A standard word is 50 units, giving:
  //   dot length = 1200 / MORSE_WPM milliseconds.
  //
  // All timing is intentionally derived from MORSE_WPM so the transmitted
  // callsign speed matches that setting exactly.
  const uint32_t ditUs = 1200000UL / settings.morseWPM;
  const uint32_t dahUs = 3UL * ditUs;
  const uint32_t elementGapUs = ditUs;
  const uint32_t characterGapUs = 3UL * ditUs;
  const uint32_t wordGapUs = 7UL * ditUs;

  uint32_t totalMs = 0;
  bool previousWasCharacter = false;

  for (uint8_t i = 0;
       settings.callsign[i] && i < sizeof(settings.callsign);
       i++) {

    char c = settings.callsign[i];

    if (c == ' ') {
      // The previous character already ended without an added gap. Emit the
      // complete standard 7-unit word space here.
      sendAudioSilence(wordGapUs);
      totalMs += wordGapUs / 1000UL;
      previousWasCharacter = false;
      continue;
    }

    const char* symbols = getMorse(c);
    if (!symbols[0]) {
      continue;
    }

    // Insert exactly the standard 3-unit gap between adjacent characters.
    if (previousWasCharacter) {
      sendAudioSilence(characterGapUs);
      totalMs += characterGapUs / 1000UL;
    }

    for (uint8_t j = 0; symbols[j]; j++) {
      if (j != 0) {
        sendAudioSilence(elementGapUs);
        totalMs += elementGapUs / 1000UL;
      }

      uint32_t toneUs =
        (symbols[j] == '-') ? dahUs : ditUs;

      totalMs +=
        sendAudioTone(
          settings.morseToneHz,
          toneUs,
          settings.toneAmplitudePercent
        );
    }

    previousWasCharacter = true;
  }

  return totalMs;
}

// -----------------------------------------------------------------------------
// Existing WAV playback
// -----------------------------------------------------------------------------

uint32_t playAudio(const char* filename, int32_t carrierOffsetHz) {
  if (hostMounted) return 0;

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
    closeRoot();
    filesystemBusy = false;
    return 0;
  }

  uint8_t header[44];

  if (file.read(header, sizeof(header)) != (int)sizeof(header)) {
    Serial.println("WAV header is incomplete.");
    file.close();
    closeRoot();
    filesystemBusy = false;
    return 0;
  }

  if (memcmp(header, "RIFF", 4) != 0 ||
      memcmp(header + 8, "WAVE", 4) != 0 ||
      memcmp(header + 12, "fmt ", 4) != 0) {
    Serial.println("Unsupported WAV header.");
    file.close();
    closeRoot();
    filesystemBusy = false;
    return 0;
  }

  uint16_t audioFormat = (uint16_t)header[20] | ((uint16_t)header[21] << 8);
  uint16_t channels = (uint16_t)header[22] | ((uint16_t)header[23] << 8);
  uint32_t sampleRate =
    (uint32_t)header[24] |
    ((uint32_t)header[25] << 8) |
    ((uint32_t)header[26] << 16) |
    ((uint32_t)header[27] << 24);
  uint16_t bitsPerSample = (uint16_t)header[34] | ((uint16_t)header[35] << 8);

  if (audioFormat != 1 ||
      channels != 1 ||
      sampleRate != AUDIO_SAMPLE_RATE_HZ ||
      bitsPerSample != 16) {
    Serial.print("WAV must be 8000 Hz, mono, 16-bit PCM. Found ");
    Serial.print(sampleRate);
    Serial.print(" Hz, ");
    Serial.print(channels);
    Serial.print(" channel(s), ");
    Serial.print(bitsPerSample);
    Serial.println(" bit.");
    file.close();
    closeRoot();
    filesystemBusy = false;
    return 0;
  }

  buildAudioPllTable(carrierOffsetHz);

  uint32_t numSamples = 0;
  uint8_t buffer[512];

  while (!hostMounted) {
    int bytesRead = file.read(buffer, sizeof(buffer));
    if (bytesRead <= 0) break;
    bytesRead &= ~1;

    uint32_t nextDeadlineUs = micros();

    for (int pos = 0; pos < bytesRead; pos += 2) {
      if (hostMounted) break;

      int16_t sample =
        (int16_t)((uint16_t)buffer[pos] | ((uint16_t)buffer[pos + 1] << 8));

      uint32_t unsignedSample = (uint32_t)((int32_t)sample + 32768L);
      uint16_t tableIndex =
        (uint16_t)((unsignedSample * (AUDIO_PLL_TABLE_LEVELS - 1UL)) / 65535UL);

      writeAudioPllEntry(tableIndex);
      numSamples++;

      nextDeadlineUs += AUDIO_SAMPLE_PERIOD_US;
      uint32_t nowUs = micros();
      if (timeBeforeAudioUs(nowUs, nextDeadlineUs)) {
        waitForAudioDeadline(nextDeadlineUs);
      }
    }
  }

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
  Serial.println("0 us (SSTV uses its own exact timing path)");

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

void applyDutyCycleOff(uint32_t activeLengthMs) {
  if (settings.dutyCyclePercent >= 100 ||
      settings.dutyCyclePercent == 0 ||
      activeLengthMs == 0) {
    return;
  }

  uint32_t offTime =
    (uint32_t)(
      ((uint64_t)activeLengthMs *
       (100U - settings.dutyCyclePercent)) /
      settings.dutyCyclePercent
    );

  setSi5351Output(false);

  Serial.print("Duty cycle: ");
  Serial.print(settings.dutyCyclePercent);
  Serial.print("%, TX=");
  Serial.print(activeLengthMs);
  Serial.print(" ms, OFF=");
  Serial.print(offTime);
  Serial.println(" ms");

  delay(offTime);
}

uint32_t playNextNormalTransmission(uint8_t* carrierSequenceStep,
                                    uint16_t* audioSequenceIndex,
                                    uint16_t* songSequenceIndex) {
  if (hostMounted) return 0;

  int32_t carrierOffsetHz = getCarrierOffsetHz(*carrierSequenceStep);
  *carrierSequenceStep =
    (*carrierSequenceStep + 1) %
    (sizeof(CARRIER_OFFSET_SEQUENCE_HZ) / sizeof(CARRIER_OFFSET_SEQUENCE_HZ[0]));

  // Key the transmitter on a clean carrier, then allow 100 ms for the RF
  // path and receiving radio to settle before beginning WAV/RTTTL audio.
  setFrequencyOffset(carrierOffsetHz);
  setSi5351Output(true);
  delay(TX_KEYUP_DELAY_MS);

  // Key-up and inter-segment carrier time are real RF-on time and therefore
  // must be included when calculating the configured duty cycle.
  uint32_t activeLengthMs = TX_KEYUP_DELAY_MS;

  if (settings.audioMode == AUDIO_MODE_RTTTL) {
    String song;
    if (getNextRtttlSong(songSequenceIndex, &song)) {
      Serial.print("Normal TX RTTTL: ");
      int colon= song.indexOf(':');
      Serial.println(colon > 0 ? song.substring(0, colon) : String("(unnamed)"));
      activeLengthMs += playRtttl(song.c_str(), carrierOffsetHz);
    } else {
      Serial.println("No valid songs.txt entries; using built-in RTTTL fallback.");
      activeLengthMs += playRtttl(DEFAULT_RTTTL, carrierOffsetHz);
    }
  } else {
    char audioFilename[32];
    bool haveAudio = nextSequencedFile(
      AUDIO_SEQUENCE_PREFIX, AUDIO_SEQUENCE_EXT,
      audioSequenceIndex, audioFilename, sizeof(audioFilename));

    if (haveAudio) {
      Serial.print("Normal TX WAV: ");
      Serial.println(audioFilename);
      activeLengthMs += playAudio(audioFilename, carrierOffsetHz);
    } else {
      Serial.println("audio.wav missing; using built-in RTTTL fallback.");
      activeLengthMs += playRtttl(DEFAULT_RTTTL, carrierOffsetHz);
    }
  }

  // Morse always follows either WAV or RTTTL audio.
  if (!test_mode && !hostMounted) {
    // Return to an unmodulated carrier for 250 ms before the Morse ID.
    // The transmitter remains keyed for this entire interval.
    setFrequencyOffset(carrierOffsetHz);
    delay(AUDIO_TO_MORSE_GAP_MS);
    activeLengthMs += AUDIO_TO_MORSE_GAP_MS;

    carrierOffsetHz = getCarrierOffsetHz(*carrierSequenceStep);
    *carrierSequenceStep =
      (*carrierSequenceStep + 1) %
      (sizeof(CARRIER_OFFSET_SEQUENCE_HZ) / sizeof(CARRIER_OFFSET_SEQUENCE_HZ[0]));

    setFrequencyOffset(carrierOffsetHz);

    Serial.print("Morse TX (live): ");
    Serial.println(settings.callsign);

    activeLengthMs += sendMorseCallsign(carrierOffsetHz);
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

  // Build all modulation lookup data with RF off. Once keyed, provide a
  // consistent 100 ms unmodulated carrier before the Robot36 VIS header.
  buildSstvPllTable(carrierOffsetHz);

  setFrequencyOffset(carrierOffsetHz);
  setSi5351Output(true);
  delay(TX_KEYUP_DELAY_MS);

  Serial.println("Starting Robot36 JPEG transmission.");

  sstvPixelBoundsErrors = 0;
  sstvLastLineBoundsErrors = 0;

  sstvPhase = 0.0;
  sstvPhase32 = 0;
  resetSstvTimingStats();

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
  uint8_t carrierSequenceStep = 0;
  uint16_t audioSequenceIndex = 0;
  uint16_t songSequenceIndex = 0;
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

    // Program the attenuator immediately before every transmission. In RANDOM
    // mode this selects a new value in the configured inclusive min/max range.
    selectAttenuationForTransmit();

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
          &audioSequenceIndex,
          &songSequenceIndex
        );

      nextIsSstv =
        settings.sstvEnabled &&
        settings.sstvMode == SSTV_MODE_ROBOT36;
    }

    if (hostMounted) {
      setSi5351Output(false);
      break;
    }

    applyDutyCycleOff(activeLengthMs);
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
  Serial.print("AUDIO_MODE=");
  Serial.println(settings.audioMode == AUDIO_MODE_RTTTL ? "RTTTL" : "WAV");
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

  // Seed the dedicated RANDOM attenuation PRNG. Mix timing with current
  // frequency/settings so the starting sequence is not always identical.
  attenuationRandomState =
    micros() ^
    ((uint32_t)(settings.transmitFreqMHz * 1000000.0)) ^
    ((uint32_t)settings.attenuationMin << 8) ^
    (uint32_t)settings.attenuationMax ^
    0xA5C31F27UL;

  if (attenuationRandomState == 0) {
    attenuationRandomState = 0x6D2B79F5UL;
  }

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

  Serial.print("Attenuation mode: ");
  Serial.println(
    settings.attenuationMode == ATTENUATION_MODE_RANDOM ? "RANDOM" : "FIXED"
  );
  if (settings.attenuationMode == ATTENUATION_MODE_RANDOM) {
    Serial.print("Random attenuation range loaded from settings: ");
    Serial.print(settings.attenuationMin);
    Serial.print(" to ");
    Serial.println(settings.attenuationMax);
  } else {
    Serial.print("Fixed attenuation: ");
    Serial.println(settings.attenuation);
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
  // Existing audio/Morse behavior continues on core 1.
  multicore_launch_core1(audioTask);
}

void loop() {
  // Main core remains available for USB/TinyUSB work handled by the core.
}
