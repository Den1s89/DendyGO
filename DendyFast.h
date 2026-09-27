/**
 * =============================================================================
 *  DendyGO / DendyFast.h
 *  Быстрая сборка горячих файлов ядра (CPU, PPU, APU, шина, маппер).
 *
 *  Зачем: Arduino IDE собирает скетч с оптимизацией, которую задаёт
 *  «Debug Level» в меню Tools. В режиме Debug это -Og, а интерпретатор 6502
 *  вместе с шиной памяти на -Og выдаёт ~30 кадров в секунду вместо 60 —
 *  заметно и по логу ([GAME] fps emu, [VID] emu ... ms), и на слух (APU
 *  не успевает наполнять кольцо I2S, музыка рвётся).
 *
 *  Здесь для этих файлов принудительно включается -O3 (#pragma GCC optimize),
 *  что не зависит от настроек IDE. Никаких «быстрых математик» (-ffast-math)
 *  не включается — поведение эмулятора не меняется, только скорость.
 *
 *  Выключить: DENDY_CORE_O3 = 0 в DendyConfig.h.
 * =============================================================================
 */
#pragma once
#include "DendyConfig.h"

#if DENDY_CORE_O3 && defined(__GNUC__) && !defined(__clang__)
  // unroll-loops: циклы рендера PPU (8 пикселей тайла, 32 тайла строки) короткие
  // с постоянной границей — развёртка убирает накладные расходы цикла.
  #pragma GCC optimize("O3,unroll-loops")
#endif

// «Всегда встраивать». Нужно там, где вызов функции стоит дороже её тела, а
// сборка (Tools -> Debug Level = Debug -> -Og) инлайнинг отключает: обращения
// к памяти картриджа идут по 2..4 раза на инструкцию 6502 и по 2 раза на каждый
// тайл фона/спрайта, то есть десятки тысяч вызовов за кадр. always_inline
// работает при любой оптимизации IDE.
#if defined(__GNUC__)
  #define DENDY_INLINE inline __attribute__((always_inline))
#else
  #define DENDY_INLINE inline
#endif

// Такты CPU (для профиля DENDY_EMU_PROFILE). На Xtensa это чтение счётчика
// циклов одной инструкцией, поэтому им можно измерять даже отдельные строки PPU.
// Если профиль выключен — ноль, и весь код измерения исчезает на этапе компиляции.
#if DENDY_EMU_PROFILE && defined(ESP32)
  #include <xtensa/hal.h>
  #define DENDY_CYCLES()  ((uint32_t)xthal_get_ccount())
#else
  #define DENDY_CYCLES()  (0u)
#endif

// Атрибут «горячая функция» — DENDY_CORE_IRAM в DendyConfig.h.
// На ESP32 это IRAM_ATTR: функция исполняется из IRAM (чтение за такт), а не из
// flash через кэш 16 КБ, мимо которого большой switch интерпретатора 6502
// постоянно промахивается. Атрибут обязан стоять и в объявлении (заголовок), и в
// определении (.cpp) — иначе линковщик может собрать вызов по «flash-адресу».
#if DENDY_CORE_IRAM && defined(ESP32)
  #include "esp_attr.h"
  #define DENDY_FAST_ATTR IRAM_ATTR
#else
  #define DENDY_FAST_ATTR
#endif

