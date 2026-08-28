# PicoFox – A Hackable 2m Fox Transmitter
**Pacificon Edition**

*This version is modified from the origional by AI6YM. It's designed to be a badge (Defcon/Ham Radio Village Style), doesn't have the expansion header, and runs sightly diffrent firmware.*

----------
**Partially AI CODED**
The modifications to this code was mainly done with AI. I want to be clear about this. 
I'm not a full-time coder, I do understand code, and don't have time to learn this platform for this one-off project.
I *HATE* when people pass off code as thiers when it's been AI/Vibe coded, so I wanted to be super clear here.
I did verify everything and validate the RF purity on my IFR 1900. It is a bit heavy on the deviation, so if you need it to be  more narrow, adjust AUDIO_FM_DEVIATION_HZ and SSTV_FM_DEVIATION_HZ as needed.
-- Trevor KG6MDW

----------

**PicoFox** is an open source fox transmitter for the 2-meter amateur band, built around the
RP2040 microcontroller (the same chip used in the Raspberry Pi Pico). It’s designed to be simple,
functional, and easy to modify.

PicoFox works well as a standalone fox for portable or walking hunts, but it’s also a great platform
for experimentation. There's ample CPU, flash,~~and GPIO~~ available if you want to add features ~~like
sensors or interactivity~~. The transmitter can generate a frequency or phase modulated signal with up
to 3kHz (data) bandwidth.


## Key Features
- Transmits FM in the 144–148 MHz band (ITU Zone 1: 144–146 MHz).
- Configured by editing a text file — no software or terminal needed.
- Transmits 8kHz 16-bit PCM WAV files.
- Automatic Morse code ID with configurable tone and speed.
- RF output > 18dBm at full power. Down to ~ -20dBm with full attenuation.
- Charges over USB.

## Extra Features in the Pacificon Firmware
- Built-in SSTV and APRS transmissions directly from JPEG and text files.
- RTTTL ringtone/song playback from a text file.
- Automatic cycling through WAV, RTTTL, JPEG, and APRS message files.
- Interleaves enabled Voice/Audio, SSTV, and APRS transmission modes.
- Dynamic TX power with a random attenuation value selected within a configured range for each transmission.
- Configurable transmit duty cycle based on the average TX time of recent transmissions.

## REMOVED Features in This Version:
- Expansion header


### Getting One

~~Fully assembled PicoFox units are available from [AI6YM.radio](https://ai6ym.radio/picofox). An
enclosure, battery, and antenna are included. I ship nearly anywhere in the world.~~

You can also build your own - both the hardware and software are open source for non-commercial use.
For commercial inquiries, I am not providing ANY production of the hardware. However for the origional design contact [justin@ai6ym.radio](mailto:justin@ai6ym.radio), then load the firmware from this repository.


## Configuration

To configure your PicoFox, connect it to any computer using a micro USB cable. It will appear as a
USB mass storage device (like a flash drive). Open `settings.txt` with a text editor and adjust the
configuration values as needed. Eject the drive, disconnect USB, and power cycle the PicoFox to
apply the new settings.

To reset to factory settings, delete `settings.txt`, eject the drive, disconnect USB, and power
cycle the PicoFox. A default settings file will be regenerated.


### Settings Overview

__THIS SECTION IS STILL UNDER DEVELOPMENT AS THE NEW SOFTWARE IS BEING WRITTEN STILL__

- `CALLSIGN`: Alphanumeric callsign (max 12 characters).
- `ITU_ZONE`: ITU zone where the transmitter operates (`1`, `2`, or `3`).
- `FREQ_MHZ`: Transmit frequency in MHz.
- `DUTY_CYCLE`: Transmit duty cycle percentage (`0–100`). If SSTV mode is on, this must be < 80.
- `ATTENUATION_MODE`: Sets the attenuation mode. `FIXED` uses the `ATTENUATION` setting, `RANDOM` selects a random attenuation value on each transmission.
- `ATTENUATION`: Attenuation level used when `ATTENUATION_MODE=FIXED`. Valid range is `0` - `127`, approximately 0.25dB steps.
- `ATTENUATION_MIN`: Minimum attenuation value used when `ATTENUATION_MODE=RANDOM`. Valid range is `0` - `127` and must be less than `ATTENUATION_MAX`.
- `ATTENUATION_MAX`: Maximum attenuation value used when `ATTENUATION_MODE=RANDOM`. Valid range is `0` - `127` and must be greater than `ATTENUATION_MIN`.
- `MORSE_WPM`: Morse ID speed in words per minute.
- `MORSE_TONE`: Morse tone frequency (100–2000 Hz).
- `MORSE_TONE_VOL`: Morse tone volume percentage (`1–100`). Default is 70. Over 70 may cause distortion. Adust as needed to make the Morse sound good.
- `VOICE_ENABLE`: Enables the Voice/Audio file and Morse code mode.
- `AUDIO_MODE`: Selects the normal audio source. `WAV` uses `audio.wav`, `audio1.wav`, `audio2.wav`, etc. `RTTTL` uses songs from `songs.txt`.
- `SSTV_ENABLE`: Enables SSTV Mode - YOU MUST set a duty cycle < 80 for this mode to work.
- `SSTV_MODE`: SSTV encoding mode, currently ONLY `ROBOT36` is supported.
- `APRS_ENABLE`: Enables APRS message transmissions. Messages are read from `messages.txt`, one message per line.

**Note:** Invalid or missing values may disable the transmitter or revert to safe defaults.


## Using the Transmitter

After configuring settings as described above. The transmitter will turn on following the
provided settings the next time it is turned on.

The transmitter may be powered by an external USB power supply or the internal battery. The internal
battery charges over USB but no indication is available to show battery charging.

**Note: The power switch must be ON to charge the battery over USB. The transmitter will run on USB
power with the switch OFF but the battery will NOT be charged.**

A full battery will last about five hours at 100% duty cycle with at 1200mAH battery. (The default for a PicoFox)


Transmit cycle is:
  - SSTV (If enabled)
  - APRS (If Enabled)
  - Audio/Morse


## Replacing the Audio File

To change the audio transmission, replace the `audio.wav` file in the device’s flash storage. The
audio must be sampled at **8kHz**, **mono** (one channel), and encoded as **signed 16-bit PCM WAV**.

*Note: Audio does not sound very good. I recomend using RTTTL.*

Using **Audacity** (free software available for all major OSs):

1. Set "Project Rate (Hz)" to `8000`.
2. Import, record, or generate your audio.
3. Mix down to a single mono track.
4. Export as WAV using **Signed 16-bit PCM** encoding.
5. Name the file `audio.wav` and copy it to the device.
6. Eject the drive and disconnect USB.
7. Power cycle the PicoFox.

**Notes:**

- Improper format may prevent audio transmission or cause noise to be transmitted instead.
- The audio is transmitted, followed by the morse ID, and then the transmitter turns off if
  `DUTY_CYCLE < 100`. The transmitter DOES take into account SSTV transmit times, as well as APRS.
- To restore the default audio, delete all  `audio.wav`, eject the drive, disconnect USB, and power cycle
  the PicoFox.
- To use muliple audio files, append a 1 to the file name. IE `audio1.wav`, `audio2.wav` etc.

## SSTV Mode

To use SSTV mode, Upload 320x240 JPEG images named `sstv.jpg` to the flash, and enable SSTV_MODE in the config file.

Make sure you have a valid callsign hard baked into your images. They should be 100% ready to transmit. the Picofox will NOT send a morse ID after SSTV.


**Notes:**
- To use muliple SSTV files, add sditional files with sqequencial numbers to the file name. IE `sstv.jpg`, `sstv1.jpg`, `sstv2.jpg` etc.
- The image MUST be 320x240 otherwise it will skip it.
- Due to timings etc, you may need to use auto-slant or high-precision slant correction. Some android apps (IE Robot36) seem to be OK
- DO NOT use a 100% duty cycle. there needs to be a break inbetween for the Pico to load the image. 99% should be fine if you want basically constant tx. In reality a lower duty cycle like 20-40% is better for harder hunts.

## APRS Mode

To use APRS mode, Upload a text file  named `message.txt` to the flash with a message on each line. To comment out a line use a #. this also means you can't just send a hashtag message. Enable `APRS_ENABLE` in the config file to transmit this mode.

The APRS messages are sent as a plain APRS message sent from the callsign configured in the config file and sent as a broadcast/beacon.  

__DO NOT USE THIS ON 144.39mhz Use a simplex/fox frequency.__

**Notes:**
- The messages are sent DIRECT so they wont be digipeated.

## Random TX power / Attenuation

Incuded in the config is e settings: `ATTENUATION_MODE` , `ATTENUATION_MIN` and `ATTENUATION_MAX`

if `ATTENUATION_MODE` is set to `RANDOM` on each transmit a random atteunation value will be chosen between `ATTENUATION_MIN` and `ATTENUATION_MAX` and applied to the transmission..

## RTTTL Mode

This was inspired by a discussion had on the [Ham Radio Village](http://hamradiovillage.org/) Discord with delphi and Ralph in #software-shenanigans.

This is built from code from [EvilMog's RTTTL](https://github.com/evilmog/evilmog/tree/master/midijunk) work. As well as others.

To use this mode, enable it, and modify the `VOICE_ENABLE` to `1` and set `AUDIO_MODE` to `RTTTL`.
Then restart. This will create `songs.txt` 

To add songs/tunes, add a song per line following the FlipperMusicTTL format:

![FlipperMusicTTL Format](https://user-images.githubusercontent.com/6899421/171048290-1e95c9ba-5c26-4e6b-a969-ecd6003c6423.gif)

You fan find details of this here, as well as exmaple tunes: https://github.com/neverfa11ing/FlipperMusicRTTTL and https://1j01.github.io/rtttl.js/



## Design Brief

An SI5351 clock generator, locked to a TCXO, provides the RF signal. Modulation is applied in
software by the RP2040. To achieve smooth FM output, modulation is handled on core 1 with
I<sup>2</sup>C running at 1 MHz. Register writes are minimized and no other devices are on
I<sup>2</sup>C0.

Core 0 manages flash operations. Most extensions or new features should run on core 0.

**Note:** Mounting the device as a USB mass storage device will interrupt clean modulation as both
cores are blocked during flash operations.

The SPI flash device is split into program memory and a FAT16 filesystem. The filesystem contains
a text file for user-supplied settings, an audio file for transmission, a second audio file
containing the generated morse code ID, and a hash file used to detect changes to the settings file.
Unexpected files are automatically deleted and missing files are re-created from defaults in the
firmware binary.

The design intentionally includes large amounts of free CPU cycles, RAM, and flash so that cool
things can be built on the PicoFox.


## Development Guide

### Build Environment Setup

- Install Arduino IDE >= 2.3.
- Install libraries - Adafruit SPIFlash, Adafruit TinyUSB, Etherkit Si5351.
- Install board package from [my fork of Arduino-Pico](https://github.com/ai6ym/arduino-pico). This
  will be merged upstream eventually but for now you'll need the changes in my fork.
- In `Tools -> Board` select the `Raspberry Pi RP2040/RP2350 Boards -> AI6YM PicoFox`.
- Connect your PicoFox and select the appropriate TTY / COM port.

On Linux it may be necessary to add udev rules for this device. The relevant rules from my system
are below. I have no idea how this works on other operating systems. Mac is probably similar,
Windows should not be used for software developement of any kind (or really anything).

```
# Make an RP2040 in BOOTSEL mode writable by all users, so you can `picotool`
# without `sudo`. 
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666"

# Symlink an RP2040 running MicroPython from /dev/pico.
#
# Then you can `mpr connect $(realpath /dev/pico)`.
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", SYMLINK+="pico"

# Adafruit TinyUSB stack.
SUBSYSTEM=="tty", ATTRS{idVendor}=="239a", ATTRS{idProduct}=="cafe", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="239a", ATTRS{idProduct}=="cafe", MODE="0666"
```

### Compilation Settings

- `Flash Size`: `16MB (Sketch: 1MB, FS: 15MB)`
- `CPU Speed`: `200mhz`
- `USB Stack`: `Adafruit TinyUSB`

If needed for some reason you could change the partition layout to give more space for the sketch.
Doing so would destroy all data but the code automatically formats the partition and writes
defaults.

All other settings are fine at default values.


### Recovering a Board/Flashing manually (IE from one of the release files)

In general the UF2 upload process will work fine. Click the upload button, after compilation the IDE
will push the board into UF2 update mode, write the new binary, and then reset the board to normal
operation.

If that doesn't work try the following:
- Reset the board and try again. Just tap the `RESET` button on the board to reset the RP2040.
- Force the device into `BOOTSEL` mode. Hold the `BOOTSEL` button, tap the `RESET` button, and then
  release the `BOOTSEL` button.
- Force the device into `BOOTSEL` mode **AND** change `Tools -> Upload Method` to `PicoTool`.
- When all else fails grab a PicoProbe (or a Pico / Pico2 flashed with the PicoProbe binary),
  connect that to the debug header of the PicoFox, and change `Tools -> Upload Method` to
  `Picoprobe`.
- If all of the above fails, well, he's dead Jim.
