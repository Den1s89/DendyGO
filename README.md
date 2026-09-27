# DendyGO 1.0 — NES/Dendy on ESP32-S3 (N16R8) + ST7789 + MAX98357

<img width="2000" height="1500" alt="Consol3" src="https://github.com/user-attachments/assets/88b0bcd7-e1c9-4000-818c-3164782d0da6" />

A handheld NES/Dendy emulator: the core (6502 CPU + 2C02 PPU + 2A03 APU + mappers). ROMs (`.nes`) live in
LittleFS in the internal flash (no microSD card needed), the frame goes to an ST7789 SPI
display, sound goes to a MAX98357 I2S DAC, and the buttons are read through an SX1509 I/O
expander.

> The firmware, its on-screen text and its diagnostic log lines are in Russian; this file
> is the English documentation (the Russian original is [`README_RU.md`](README_RU.md)).
> The deep-dive document [`docs/TECH_NOTES.md`](docs/TECH_NOTES.md) is in Russian as well.

* **Video:** 2" ST7789 240×320 (320×240 landscape), RGB565, SPI,
  frame transfer without DMA.
* **Audio:** 44 100 Hz, 16 bit, ring buffer + a separate I2S task.
* **TV region:** NTSC 60 Hz / PAL 50 Hz — AUTO from the ROM header or forced manually
  (a button in the menu); volume and TV mode are stored in LittleFS.
* **Mappers:** 0 (NROM, including NROM-128), 1 (MMC1), 2 (UxROM), 3 (CNROM), 4 (MMC3),
  7 (AxROM), 11 (Color Dreams), 23 (VRC2b), 66 (GxROM), 71 (Camerica). ROMs with other
  mappers are listed in the menu, but the core rejects them at launch: `Load error`.
* **Menu:** ROM list from LittleFS (up to 64 files), OSD for brightness/volume/color
  order, auto-start of a single ROM.
* Accuracy tests (nestest, blargg) have not been run yet — compatibility was verified on
  games and against documented behaviour (reference: a real console / Dendy).

## 🛠 3D Printed Case & Enclosure
The official, custom-designed 3D printable case for the **DendyGO** console is available on MakerWorld.

* **Get the 3D models here:** [DendyGO 3D Printable Case on MakerWorld](https://makerworld.com/en/models/3354561-dendygo-esp32-s3-game-console#profileId-3813213)

<img width="2000" height="1500" alt="Console4" src="https://github.com/user-attachments/assets/20bb1ddf-df41-40c9-8c97-a9a184cbbc5c" />

## Contents

1. [Project layout](#project-layout)
2. [What to install](#what-to-install)
3. [Board settings (Tools)](#board-settings-tools)
4. [Pinout and wiring](#pinout-and-wiring)
5. [Controls](#controls)
6. [Brightness, volume, TV region](#brightness-volume-tv-region)
7. [How to upload ROMs](#how-to-upload-roms)
8. [Building](#building)
9. [Troubleshooting (short)](#troubleshooting-short)
10. [Technical notes](#technical-notes)
11. [License](#license)
12. [Credits and sources](#credits-and-sources)

## Project layout

| File | Purpose |
|---|---|
| `DendyGO.ino` | sketch: display, audio, buttons, menu, OSD, `setup()`/`loop()` |
| `NesCore.h/.cpp` | emulator core: bus, cartridge, frame, audio, public API |
| `NesCpu.h/.cpp` | 6502 CPU (all unofficial opcodes) |
| `NesPpu.h/.cpp` | 2C02 PPU: background and sprites, sprite-0 hit, NMI |
| `NesApu.h/.cpp` | APU: pulses (with sweep), triangle, noise, DMC, mixing |
| `NesMapper.h/.cpp` | mappers 0, 1, 2, 3, 4, 7, 11, 23, 66, 71 |
| `DendyConfig.h` | the **single place** for pins and settings |
| `DendyFast.h` | `-O3` for the hot core files and `IRAM_ATTR` for hot functions |
| `LcdTest.ino` | LCD output test without the emulator (`DENDY_LCD_TEST 1`) |
| `partitions.csv` | partitions: 3 MB application + 9.9 MB LittleFS for ROMs |
| `upload_fs.ps1` | script: validate ROM names, build `littlefs.bin`, flash it |
| `data/roms/*.nes` | YOUR ROMs (names up to 32 characters). No ROMs in the repository |
| `README.md` / `README_RU.md` | this documentation: English / Russian |

> Not committed (see `.gitignore`): ROMs `data/roms/*.nes` and the generated
> `littlefs.bin` — `upload_fs.ps1` builds that image from the `data/` folder in seconds.

## What to install

1. **Arduino IDE 2.x** + the **esp32 by Espressif** core (tested on 3.3.0, ≥ 3.0 needed).
2. Libraries via "Manage Libraries":
   * **LovyanGFX** (by lovyan03);
   * **SparkFun SX1509 I/O Expander** (by SparkFun Electronics).
3. A **USB-C cable** that supports data transfer.

## Board settings (Tools)

| Setting | Value |
|---|---|
| Board | **ESP32S3 Dev Module** |
| USB CDC On Boot | **Enabled** (otherwise `Serial` writes to UART0, not to the USB port) |
| CPU Frequency | 240 MHz |
| Debug Level / Core Debug Level | **None** (with `Debug`/`Verbose` the core and the sketch are built with `-Og`, which makes the emulator several times slower) |
| Flash Mode / Size | QIO 80 MHz / **16MB (128Mb)** |
| PSRAM | **OPI PSRAM** (on an N16R8 module the PSRAM is wired as OPI!) |
| Partition Scheme | **16M Flash (3MB APP/9.9MB FATFS)** |
| Upload Speed | 921600 |
| Arduino/Events Run On | Core 1 |
| Erase All Flash | Disabled |

> The `partitions.csv` from the sketch folder **overrides** the scheme from the menu: the
> application gets 3 MB and the data partition becomes 9.9 MB of **LittleFS** (the menu
> scheme puts FATFS there, which is useless for us). "Partition Scheme" is only selected so
> that the sketch size check does not complain.

## Pinout and wiring

The board is an **ESP32-S3 N16R8** (16 MB flash, 8 MB Octal-PSRAM). GPIO33…GPIO37 **must
not** be used — they are taken by the PSRAM (in `DendyConfig.h` this is enforced by a
`static_assert`).

| Part | Pin | GPIO |
|---|---|---|
| ST7789 | SCLK (SCL) | 11 |
| ST7789 | MOSI (SDA) | 12 |
| ST7789 | CS | 10 |
| ST7789 | DC | 13 |
| ST7789 | RST / RES | 14 |
| ST7789 | BLK (backlight) | 21 |
| MAX98357 | BCLK (BCK) | 16 |
| MAX98357 | LRCK (WS) | 15 |
| MAX98357 | DIN (DOUT) | 17 |
| MAX98357 | SD (shutdown/mode) | **not to GND!** floating or via 1 MΩ to VIN |
| SX1509 | SDA | 8 |
| SX1509 | SCL | 9 |
| SX1509 | INT | 3 (`INPUT_PULLUP`, FALLING) |
| SX1509 | A0, A1 | GND (address `0x3E`) |
| Power | board VIN | 5 V (display/DAC), 3.3 V for the SX1509 |

Buttons are wired between an SX1509 pin and GND (active LOW, the pull-ups are enabled by
the firmware, 8 ms hardware debounce):

| SX1509 pin | Button |
|---|---|
| 0 | Right |
| 1 | Down |
| 2 | Up |
| 3 | Left |
| 4 | REGION — TV selection (AUTO / NTSC / PAL), menu only |
| 5 | Volume "−" (auto-repeat while held) |
| 6 | Volume "+" (auto-repeat while held) |
| 7 | MENU (short press — reset the game, hold — back to the menu) |
| 8 | Screen BRIGHTNESS (5 / 15 / 25 / 50 / 75 / 100 %, 70 % at power-up) |
| 9 | Turbo A (auto-repeat ~15 Hz) |
| 10 | Turbo B (auto-repeat ~15 Hz) |
| 11 | A |
| 12 | B |
| 13 | Select |
| 14 | Start |
| 15 | free |

In `DendyConfig.h` (block 4) the same set is declared **by function** (`SX_PIN_A`,
`SX_PIN_B`, `SX_PIN_UP`, `SX_PIN_VOL_DOWN`, `SX_PIN_BRIGHT`, …); one pin cannot serve two
buttons — the layout is checked by a `static_assert`, so the build fails before flashing.

### Wiring gotchas

* **MAX98357, the SD pin.** The voltage on SD selects the mode: `< 0.16 V` — the amplifier
  is **off** (total silence with perfectly working firmware!), `0.16…0.77 V` — (L+R)/2,
  `0.77…1.4 V` — right channel, `> 1.4 V` — left. Inside the chip SD is pulled to ground
  through 100 kΩ, so "leave it floating" is only correct on boards that have an external
  1 MΩ pull-up to VIN (for example Adafruit #3006). If there is no such pull-up, add 1 MΩ
  between SD and VIN. A quick multimeter check: SD to GND must read more than 0.16 V. The
  speaker is connected **only between `+` and `−`** (BTL bridge output): a second wire to
  GND gives no sound. `GAIN` floating = 9 dB, 100 kΩ to GND = 15 dB.
* **Backlight.** The ST7789 BLK pin on GPIO21 is driven by PWM, so brightness changes on
  the fly — without re-initialising the display and without flicker.
* **SDA/SCL of the SX1509 may be swapped** — at startup auto-detection runs
  (`SX1509_AUTO_SWAP_PINS`): both pin orders and 400/100 kHz are tried, and a warning line
  with the actual pins appears in the serial monitor.
* **Button pull-ups** are enabled by the firmware itself (`io.pinMode(pin, INPUT_PULLUP)`) —
  with plain `INPUT` the inputs float and every button reads as pressed.
* Tie all grounds together (board, display, DAC, expander); power comes from USB-C.
  GPIO0 (the BOOT button) is only used by the display test.

## Controls

| Action | Buttons |
|---|---|
| Play | A, B, Select, Start, D-pad |
| Turbo A / B | Turbo A / Turbo B (repeat ~15 Hz while the button is held) |
| Reset the game | short press of **MENU** (< 400 ms) |
| Back to the ROM menu | hold **MENU** (≥ 400 ms) |
| Volume | pin 6 ("+") / pin 5 ("−"), step 5, auto-repeat, OSD bar |
| Menu: move through the list | Up / Down (with auto-repeat) |
| Menu: page through the list | Left / Right |
| Menu: launch a ROM | **A** or **START** |
| Menu: re-scan the ROMs in LittleFS | **MENU** |
| Menu: audio test (220/440/880 Hz stair) | **TURBO B** |
| Frame color order (`DENDY_COLOR_ORDER`) | **SELECT + Left / Right** (cycles 0→1→2→3) |

## Brightness, volume, TV region

* **Brightness** — the button on SX1509 pin 8 (`SX_PIN_BRIGHT`), works both in a game and
  in the menu: it cycles `5 → 15 → 25 → 50 → 75 → 100 % → 5 …`; it starts at 70 %
  (`DISPLAY_BR_DEFAULT_PCT`). The value is shown in the OSD ("YARK 75 %") and in the menu
  header, and is not stored anywhere.
* **Volume** — buttons 5/6, step 5; the value is stored in LittleFS (`/volume.bin`).
* **TV region** — the button on pin 4 (`SX_PIN_REGION`), menu only:
  `AUTO → NTSC → PAL`. In AUTO the region comes from the `.nes` header (byte 9, bit 0;
  for NES 2.0 — byte 12); if there is no such flag, NTSC is used. The choice is stored
  (`/region.bin`) and applied when a ROM starts: 60.000 fps in NTSC and 50.000 in PAL.
  A PAL ROM in NTSC mode runs about 20 % faster and sounds ~3 semitones higher (and vice
  versa) — that is a property of the original hardware, not of the emulator.

## How to upload ROMs

ROMs are searched for in the **`/roms`** folder inside LittleFS, files are **`*.nes`**
(the case of the extension does not matter). Create a `data/roms` folder next to
`DendyGO.ino` and put the images there:

> Firmware limits: up to `ROM_MAX_FILES` = 64 ROMs in the menu, up to `ROM_MAX_SIZE` = 5 MB
> per ROM, names are truncated to `ROM_NAME_LEN` = 40 characters in the menu. A separate
> limit is 32 characters per file name in LittleFS (see the end of this section).

```text
DendyGO/
├─ DendyGO.ino
├─ DendyConfig.h
├─ Nes*.h/.cpp  ...
├─ partitions.csv
└─ data/
   └─ roms/
      ├─ Battle City (J).nes
      └─ Super Mario Bros (U).nes
```

### Option 0 (the easiest): the `upload_fs.ps1` script from this project

Close the **serial monitor** in the Arduino IDE (otherwise the port is busy) and run this
from the sketch folder:

```powershell
powershell -ExecutionPolicy Bypass -File .\upload_fs.ps1           # build + flash
powershell -ExecutionPolicy Bypass -File .\upload_fs.ps1 -NoFlash  # build the image only
powershell -ExecutionPolicy Bypass -File .\upload_fs.ps1 -Port COM4
```

The script finds `mklittlefs.exe`/`esptool.exe` in the esp32 core by itself, checks that
every name is shorter than 32 characters, takes the address and the size of the `spiffs`
partition from `partitions.csv`, builds `littlefs.bin`, prints its contents and flashes it
into the board. This is the real output of the script (its messages are in Russian, as is
the whole script — see [`README_RU.md`](README_RU.md)):

```text
рома(ов) в data\roms: 4
раздел 'spiffs': адрес 0x610000, размер 10354688 байт (10112 КБ)
образ готов: ...\littlefs.bin (10354688 байт)
содержимое образа:
262160  /roms/Adventure Island III (U) [!].nes  /
40976   /roms/Super Mario Bros. (JU) [!].nes    /
Готово: образ LittleFS залит в COM4. Нажмите RST — в меню появится список ромов.
```

### Option 1 (manual): `mklittlefs` + `esptool`

Both tools already ship with the esp32 core (adjust the version numbers in the paths to
your own):

```powershell
$mk  = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\mklittlefs\3.0.0-gnu12-dc7f933\mklittlefs.exe"
$esp = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.0.0\esptool.exe"

# 1) build the LittleFS image from the data folder (10354688 = 0x9E0000 = the spiffs partition size)
& $mk -c data -p 256 -b 4096 -s 10354688 littlefs.bin

# 2) flash the image into the spiffs partition at 0x610000 (COM5 is your port)
& $esp --chip esp32s3 --port COM5 --baud 921600 write-flash 0x610000 littlefs.bin
```

To check the contents of the generated image (with `-l` you need the same parameters,
otherwise the tool dies with `Assertion failed`):

```powershell
& $mk -l -p 256 -b 4096 -s 10354688 littlefs.bin
```

The same command updates the ROMs only — without re-flashing the sketch (it takes
seconds). The address `0x610000` and the size `10354688` come from `partitions.csv`; with
your own partition table compute them from the `Offset`/`Size` columns. The IDE plugin
"ESP32 LittleFS Data Upload" (`arduino-esp32littlefs-plugin`) and PlatformIO
(`pio run -t uploadfs` with `board_build.filesystem = littlefs`) do exactly the same.

### Limits and verification

> ⚠ **ROM file names must not be longer than 32 characters including `.nes`.** With a
> longer name `mklittlefs` aborts with `unable to open '/roms/...'` + `error adding file!`,
> and **nothing at all** ends up in LittleFS — the menu keeps showing `net fajlov`
> (transliterated "no files"). Example:
> `Hudson's Adventure Island III (U) [!].nes` (37 characters) — error,
> `Adventure Island III (U) [!].nes` (32) — builds fine.

### After flashing

Press RST: Serial (115200) prints the list of ROMs found while the display shows the menu.
If there is only one ROM and you do not need the menu, `ROM_AUTORUN_SINGLE 1` in
`DendyConfig.h` launches it right away.

The menu shows `net fajlov` — no ROMs found? Check in order:

1. **Was the LittleFS image flashed at all?** The `data/` folder on your PC is only a
   template: **uploading the sketch (Ctrl+U) does not flash the image**. The log of an
   unflashed image looks like this: `[FS] /: файлов 0, каталогов 0` (0 files, 0 folders)
   and `[FS] /roms — каталог не найден` (folder not found).
2. **Did the image build stop at a long name?** The `mklittlefs` output must not contain
   `error adding file!` (the 32-character limit).
3. **What is really inside the image:** `& $mk -l -p 256 -b 4096 -s 10354688 littlefs.bin` —
   it should list `/roms/...nes` entries.
4. **Correct write address.** At startup the sketch prints its own partition:
   `[FS] раздел 'spiffs': адрес 0x610000, размер 10354688 байт` (partition, address,
   size) — the address in `write-flash` must match. If the size differs, the board was
   built with another partition table: upload the sketch once more (the project ships
   `partitions.csv`) and repeat the FS upload.
5. **Extension and folder:** files are `*.nes`, the folder is `roms` (lowercase). If the
   folder is missing, the sketch looks for ROMs in the LittleFS root.

## Building

In the IDE: **Verify** (Ctrl+R) / **Upload** (Ctrl+U). The project is one sketch
`DendyGO.ino` plus `.h/.cpp` modules in the same folder; no `src/` structure and no
`#include` paths are needed.

```text
Скетч использует 583227 байт (18%) памяти устройства. Всего доступно 3145728 байт.
Глобальные переменные используют 115676 байт (35%) динамической памяти, оставляя
212004 байт для локальных переменных. Максимум: 327680 байт.
```

(That is the Arduino IDE output with a Russian UI: 583227 bytes = 18 % of the flash,
115676 bytes = 35 % of the static RAM.)

Main knobs in `DendyConfig.h`: `DENDY_CORE_IRAM` and `DENDY_CART_IRAM` (hot core code and
the active cartridge bank windows in internal SRAM — they speed the emulation up
noticeably; the bank cache takes 40 KB of SRAM, so set `DENDY_CART_IRAM 0` if you run
short on memory), `DENDY_DEBUG_SERIAL` / `DENDY_BTN_TRACE` / `DENDY_ROM_TRACE` /
`DENDY_VIDEO_TRACE` (log and traces — keep them 0 for playing), `DENDY_LCD_TEST` (display
test without the emulator), `DENDY_COLOR_ORDER` (color order; mode 1 is correct on this
board and is the default), `DENDY_LOOP_STACK` (the stack of the `loop()` task, 16 KB).

## Troubleshooting (short)

The log goes to Serial at 115200 (`USB CDC On Boot = Enabled`). Flags in `DendyConfig.h`:

```c
#define DENDY_DEBUG_SERIAL 1   // log to Serial at all (115200)
#define DENDY_BTN_TRACE    0   // I2C scan, SX1509 register dump, every button press
#define DENDY_ROM_TRACE    1   // LittleFS listing, FS partition, reason for skipping
#define DENDY_VIDEO_TRACE  0   // [VID] lines: PPU, mapper, PC trace, ASCII frame
```

`DENDY_BTN_TRACE = 1` is a diagnostic mode, not a gaming one: every press prints about
130 characters, and at 115200 baud `Serial.printf()` can slow the game loop down. Button
state is visible in the menu anyway.

| Symptom | What to check |
|---|---|
| `LittleFS error / check partition scheme` | the sketch folder must contain this project's `partitions.csv`, or a partition scheme with SPIFFS must be selected |
| `[FS] LittleFS: свободно … 0 КБ` ("0 KB free") | the partition is mounted but empty — flash `data/` (see "How to upload ROMs") |
| `[ROM] папка /roms не найдена` ("folder not found") | ROMs go to `data/roms/*.nes`, and the folder must be named `roms` (lowercase) |
| `[ROM] нет памяти в PSRAM` ("no PSRAM memory") | in the board menu: **OPI PSRAM**, Flash Size = 16MB |
| `[I2C2] НИКОГО не найдено` / `[SX1509] нет отклика` ("nobody found" / "no response") | 3.3 V power, SDA/SCL, 4.7 kΩ pull-ups, common GND, A0/A1 to GND |
| All buttons read as "pressed" (`pad=0xFF`), `PullUp=0000…` | pin pull-ups are not enabled — this firmware enables them itself; check the wiring and `SX_PIN_*` |
| Buttons "stick" or bounce | increase `SX1509_DEBOUNCE_MS` (8 → 16 ms) |
| `[BTN] подсказка: … INT (GPIO3) ни разу не сработало` ("INT never fired") | the INT wire is not connected: the buttons still work (polled every frame, ~16 ms), but with INT they are polled on an event only |
| Black screen, no backlight | backlight GPIO (21), `DISPLAY_BRIGHTNESS`, `DISPLAY_INVERT`, `DISPLAY_OFFSET_X/Y`, `DISPLAY_BGR`; button 8 restores the brightness (70 % at power-up) |
| The picture is "snowy"/glitchy | lower `DISPLAY_FREQ_WRITE` 120 → 80 → 40 MHz |
| Unclear whether the display or the core is at fault | enable `DENDY_LCD_TEST 1` (display test without the emulator, see "Technical notes") |
| Wrong colors (sky green or pink instead of blue-cyan) | hold **SELECT** and press **Left/Right** to pick `DENDY_COLOR_ORDER` by eye (1 on this board) |
| `Guru Meditation … Stack canary watchpoint triggered (loopTask)` | `loop()` stack overflow; the stack is raised to `DENDY_LOOP_STACK` = 16 KB, increase it if this happens again |
| "Sketch too big" while compiling | select the "16M Flash (3MB APP/9.9MB FATFS)" partition scheme |
| The game runs slower than 60 Hz (`fps emu` < 60) | `Tools → Debug Level = None`, `CPU Frequency = 240 MHz`; in the log look at `emu N ms/kadr`, `fps emu`, `na ekran`, `otdano` and `[GAME] profil` (time inside the emulator) |

## Technical notes

All the deep material lives in [`docs/TECH_NOTES.md`](docs/TECH_NOTES.md) (in Russian) so
that this README stays short:

* frame timing, in-core region switching (NTSC/PAL) and "why not 60.0988 fps";
* audio: buffer sizes ("why the music stutters"), "silent audio", the first sound after
  power-up, `DENDY_MAX_FRAME_SKIP`, `DENDY_APU_ANTIALIAS`, `DENDY_APU_LOWPASS_HZ`, the
  frame-counter step layout (`$4017`), `AUDIO_ADAPTIVE_RATE`,
  `DENDY_PUSH_EVERY_N_FRAMES`;
* diagnostics: the full button/filesystem log, the full symptom table,
  `DENDY_VIDEO_TRACE` (video path, color order `DENDY_COLOR_ORDER`);
* the display test without the emulator `DENDY_LCD_TEST` (9 panel configurations);
* internals: video/audio/buttons/menu, memory (SRAM vs PSRAM), stack.

## License

This project is licensed under the **MIT License**

## Credits and sources

The emulator core (6502 + 2C02 PPU + 2A03 APU + mappers), the menu, audio and the video
output are written **from scratch**: this is not a port, not a fork and not a translation
of somebody else's emulator, no third-party core code was used. Some decisions were guided
by documentation and other people's experience — thanks to these sources.

Libraries and external data:

* **LovyanGFX** (by lovyan03, MIT) — ST7789 driver and backlight PWM;
* **SparkFun SX1509 I/O Expander** (SparkFun Electronics, MIT) — button expander;
* **arduino-esp32** and **ESP-IDF** — I2S, LittleFS, FreeRTOS tasks, USB CDC;
* **2C02 palette** — an open table found online (the original source could not be confirmed).

NES hardware knowledge:

* **NESdev Wiki** (`wiki.nesdev.org`) and the **nesdev.org forum** — the main source:
  PPU timing (rendering, scrolling, sprite-0 hit), the frame-counter step layout and APU
  envelopes, mapper behaviour, the palette;
* **FCEUX** — a behavioural reference (for example, how it copes without a submapper when
  mapper 71 writes to `$9000-$9FFF`). No FCEUX code was copied: only behaviour was
  compared.

ROMs are not part of the repository (third-party copyright) — the `data/roms` folder is a
placeholder only, see `.gitignore`.
