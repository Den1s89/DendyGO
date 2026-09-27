/**
 * =============================================================================
 *  DendyGO / NesApu.h
 *  APU 2A03: 2 импульсных канала (Pulse1/2), треугольник, шум и DMC.
 *  Сэмплирование на AUDIO_SAMPLE_RATE в кольцевой буфер, который читает задача I2S.
 *  Микширование — по классической нелинейной формуле NES (pulse + TND).
 * =============================================================================
 */
#pragma once
#include <stdint.h>
#include "DendyFast.h"                // DENDY_FAST_ATTR (IRAM для горячего кода)
struct Nes;

class NesApu {
public:
  // Размер кольца — из конфига (AUDIO_RING_SIZE в DendyConfig.h, блок 3 «ЗВУК»):
  // там же считается «подушка» звука (AUDIO_PRIME_SAMPLES) и стоит static_assert,
  // что она вместе с блоком DMA помещается сюда.
  static const int RING_SIZE = AUDIO_RING_SIZE;   // сэмплов в кольцевом буфере

  void reset(Nes* nes);
  DENDY_FAST_ATTR void writeRegister(Nes* nes, uint16_t addr, uint8_t value);
  DENDY_FAST_ATTR uint8_t readStatus(Nes* nes);
  DENDY_FAST_ATTR void step(Nes* nes, uint32_t cpuCycles);   // продвинуть APU на такты CPU
  void setVolume(uint8_t volume);            // громкость вывода 0..255 (из меню)

  int drain(int16_t* dst, int maxSamples);   // забрать готовые сэмплы

  // --- Адаптивный темп вывода (AUDIO_ADAPTIVE_RATE в DendyConfig.h) -----------
  // Забрать РОВНО maxOut сэмплов, растягивая то, что лежит в кольце (дробный шаг
  // чтения + линейная интерполяция). Нужно, когда эмулятор идёт медленнее
  // приставки: вместо вставки тишины в музыку («песок») звук растягивается во
  // столько раз, во сколько эмулятор медленнее (при 49,4 кадра/с — шаг 0,823).
  // Когда эмулятор успевает, шаг равен ровно 1:1 и сэмплы уходят бит в бит, как
  // в drain(). Возвращает число записанных сэмплов (0 — кольцо пусто и в кольце
  // нечего растягивать). Адаптивность включает скетч: игра — да, меню — нет
  // (в меню кольцо держит полным serviceMenuSound, растягивать нечего).
  int drainResampled(int16_t* dst, int maxOut);
  void setAdaptive(bool on) { adaptive = on; }
  int  ratePerMille() const;                 // темп вывода: 1000 = 1:1 (в [SND] `temp`)
  // --- Темп ПРОДЮСЕРА от скетча: предварительное знание вместо оценки по кольцу -
  // Скетч (runGame) измеряет фактический период кадра по micros() и публикует
  // здесь отношение «номинальный период / фактический» в Q16: 65536 = эмулятор
  // идёт ровно 60,000 (50,000) кадров/с, 65011 = на 0,8 % медленнее. Это ТОЧНАЯ и
  // ГЛАДКАЯ оценка темпа (время измеряется непрерывно, ошибка ~0,1 % на 16 кадрах
  // и ~0,01 % на секунде), она и становится базой шага чтения кольца. Почему не по
  // кольцу — в DendyConfig.h (AUDIO_PRODUCER_WIN_FRAMES): оценка «сколько сэмплов
  // пришло за 16 блоков» квантована кадровыми пачками APU (735 сэмплов раз в кадр)
  // и гуляет на ±9 %, отчего тон «расплывался» на единицы-десятки центов.
  void setProducerRatioQ16(uint32_t q16);
  // Диапазон шага (в промилле) за время с последнего вызова — для строки [SND]:
  // сразу видно, стоит ли тон ровно (min == max) или «дышит» после правки темпа.
  // Заодно сбрасывает диапазон к текущему шагу. Возвращает текущий шаг.
  int  takeRateRange(int* minPermille, int* maxPermille);
  uint32_t starvedSamples() const { return starved; }   // «зажато» (в [SND] `golod`)

  // --- Регион (NTSC/PAL): тактовая 2A03 из таблицы таймингов -------------------
  // От региона зависит тактовая 2A03 (NTSC 1 786 840 Гц, PAL 1 662 375 Гц), а от
  // неё — границы сэмплов: при одной и той же частоте вывода получается ровно
  // 735 сэмплов на кадр в NTSC и 882 в PAL, то есть APU и I2S идут секунда в
  // секунду. Частота нужна в горячем NesApu::step(), поэтому это ПОЛЕ, а не
  // константа: переключение режима в меню не требует пересборки
  // (см. NesCore::setRegion).
  void setRegion(bool pal) { cpuHz = pal ? DENDY_PAL_CPU_HZ : DENDY_NTSC_CPU_HZ; }
  uint32_t cpuClockHz() const { return cpuHz; }

  // Сбросить ТОЛЬКО состояние ВЫВОДА: регулятор темпа (шаг, окно измерения,
  // «залипший» сэмпл) и признак голода. Нужно на переходах игра <-> меню: иначе
  // регулятор продолжает считать окно от старого продюсера, а удержанный сэмпл
  // держит уровень там, где музыка остановилась (та самая «зацикленная» нота).
  void resetStream();

  // --- Внешний источник звука (нужен меню: эмулятор там не работает) ---
  // Кольцо одно на всех, поэтому «джингл» меню и звук игры расходятся по одному
  // и тому же кольцу -> I2S -> MAX98357. Слышно джингл = тракт звука исправен.
  int  pushExternal(const int16_t* src, int maxSamples);  // протолкнуть сэмплы
  int  pushSilence(int maxSamples);          // протолкнуть тишину («подушка» игры)
  int  freeSpace() const;                    // сколько сэмплов ещё влезет
  int  fill() const;                         // сколько сэмплов лежит в кольце
  void flush();                              // очистить кольцо (меню <-> игра)
  // Диагностика: сколько сэмплов НЕ влезло в кольцо (буфер был полон, pushSample
  // их потерял). Ненулевое значение = в музыке дырки: задача I2S не успевает
  // забирать. Печатается в [SND]/[GAME] как `drob`.
  uint32_t droppedSamples() const { return dropped; }

  bool frameIrqFlag = false;                 // frame counter IRQ (бит 6 в $4015)
  bool dmcIrqFlag   = false;                 // IRQ от DMC (бит 7 в $4015)

  // Диагностика: последнее значение, записанное игрой в $4015 (биты включённых
  // каналов). Ненулевое значение вместе с растущим счётчиком записей APU
  // означает, что игра дошла до настройки музыки — тогда «нет звука» уже не
  // связано с игрой, и причину надо искать в тракте I2S/кодеке.
  uint8_t enabledChannels() const { return channelEnabled; }

private:
  // ------------------------- Кольцевой буфер сэмплов -------------------------
  int16_t  ring[RING_SIZE];
  volatile uint16_t ringHead = 0;            // запись (APU)
  volatile uint16_t ringTail = 0;            // чтение (задача I2S)
  uint32_t dropped = 0;                      // потеряно сэмплов (кольцо было полно)
  void pushSample(int16_t s);

  // --- Состояние адаптивного темпа вывода (см. drainResampled) ---------------
  bool     adaptive = false;                 // включён скетчем (игра — да, меню — нет)
  uint32_t cpuHz    = DENDY_CPU_HZ;          // тактовая 2A03 текущего региона (NTSC/PAL)
  uint32_t outPhase = 0;                     // Q16: позиция чтения внутри кольца
  uint32_t outStep  = 65536;                 // Q16: сэмплов кольца на выходной сэмпл
  int16_t  holdLast = 0;                     // уровень, который держим при голоде
  uint32_t starved  = 0;                     // сэмплов «зажато» (диагностика `golod`)
  // Окно измерения темпа продюсера (APU): сколько сэмплов он реально даёт на один
  // выходной. Так регулятор узнаёт скорость эмулятора, не спрашивая её у скетча.
  int32_t  prodAcc    = 0;                   // прибавлено APU в кольцо за окно
  int32_t  consAcc    = 0;                   // выведено нами за окно
  int32_t  fillPrev   = -1;                  // заполнение кольца на прошлом блоке
  int32_t  fillAvg    = 0;                   // сглаженное заполнение (трим, см. ниже)
  // «Кольцо упёрлось»: СУММА глубины (в сэмплах) по блокам окна, отдельно для низа
  // и верха. Раньше здесь были крайние значения (fillMin/fillMax) — и одного блока,
  // в котором кольцо успело показать мало сэмплов (всплеск: загрузка рома, опрос
  // кнопок по I2C, печать в Serial), хватало, чтобы тон уехал на ВСЁ следующее окно
  // (~93 мс): это и было слышно как «звук чуть плавает». Сумма же даёт СРЕДНЮЮ
  // глубину за окно, то есть одиночный провал — доли процента, а настоящее «кольцо
  // лежит на дне» — полную поправку. Плюс поправка не «залипает»: в следующем окне
  // сумма снова с нуля (у сглаженного заполнения она спадала бы ещё ~0,5 с).
  int32_t  lowSum     = 0;                   // ниже AUDIO_RATE_FLOOR (глубина, сэмплы)
  int32_t  highSum    = 0;                   // выше RING_SIZE-AUDIO_DMA_FRAMES
  uint8_t  ctrlBlocks = 0;                   // блоков DMA в текущем окне
  // --- Темп продюсера, опубликованный скетчем (NesApu::setProducerRatioQ16) -----
  // prodRatioQ16 — что прислал скетч, prodRatioSm — то же после сглаживания (оно
  // и идёт базой шага). prodRatioOk = публикация была; пока её нет, база считается
  // прежним способом (по кольцу) — например в первые кадры после старта игры.
  uint32_t prodRatioQ16 = 65536;
  uint32_t prodRatioSm  = 65536;
  bool     prodRatioOk  = false;
  // Диапазон шага между вызовами (для строки [SND]: «тон стоит» или «дышит»).
  int32_t  rateMinQ16   = 65536;
  int32_t  rateMaxQ16   = 65536;

  // --------------------------- Импульсный канал -----------------------------
  struct Pulse {
    uint8_t  duty = 0;
    bool     lengthHalt = false, constantVolume = false;
    bool     sweepEnable = false, sweepNegate = false;
    uint8_t  volume = 0, envelope = 0, envelopeDivider = 0;
    uint8_t  sweepPeriod = 0, sweepShift = 0;
    uint8_t  sweepDivider = 0;                 // счётчик до следующего шага sweep
    bool     sweepReload  = false;             // перезагрузка счётчика (запись $4001/$4005)
    uint16_t timer = 1, timerPeriod = 0;
    uint8_t  step = 0, lengthCounter = 0;
    bool     sweepMute = false;
    // «Площадь» уровня за окно сэмпла (антиалиасинг, DENDY_APU_ANTIALIAS):
    // сумма (уровень × такты CPU), накопленная с прошлого сэмпла. В единицах
    // тактов CPU, а не APU: один APU-такт = 2 такта CPU (см. clockTimer).
    uint32_t area = 0;
    uint8_t  out() const;
    uint8_t  avg(uint32_t winLen) const;        // средний уровень за окно
  // ВАЖНО (скорость): clockTimer* вызываются по 4 раза на КАЖДЫЙ выходной сэмпл
// (~3000 вызовов за кадр) из NesApu::step(). Пока это были обычные функции, они
// лежали в flash и вызывались через кэш 32 КБ, который на каждом кадре вытесняют
// записи кадрового буфера и чтение кадра из PSRAM. DENDY_INLINE встраивает их в
// step() — тот собран с IRAM_ATTR (DENDY_FAST_ATTR) и потому не кэшируется вовсе.
  DENDY_INLINE void     clockTimer(uint32_t apuTicks);     // ВНИМАНИЕ: APU-такты = CPU/2
    void     clockEnvelope();
    void     clockLength();
    void     clockSweep(bool isPulse1);
  } pulse[2];

  // ------------------------------ Треугольник -------------------------------
  struct Triangle {
    bool     control = false, linearReload = false;
    uint8_t  linearCounter = 0, linearReloadValue = 0, lengthCounter = 0, step = 0;
    uint16_t timer = 1, timerPeriod = 0;
    uint32_t area = 0;                          // см. Pulse::area (в тактах CPU)
    uint8_t  out() const;
    uint8_t  avg(uint32_t winLen) const;
    DENDY_INLINE void     clockTimer(uint32_t cycles);       // тактируется тактом CPU (как в 2A03)
    void     clockLinear();
    void     clockLength();
  } tri;

  // --------------------------------- Шум ------------------------------------
  struct Noise {
    bool     lengthHalt = false, constantVolume = false;
    uint8_t  volume = 0, envelope = 0, envelopeDivider = 0;
    uint16_t timer = 1, timerPeriod = 4;
    uint16_t lfsr = 1;
    bool     mode = false;
    uint8_t  lengthCounter = 0;
    uint32_t area = 0;                          // см. Pulse::area (в тактах CPU)
    uint8_t  out() const;
    uint8_t  avg(uint32_t winLen) const;
    DENDY_INLINE void clockTimer(uint32_t cycles);       // тактируется тактом CPU (период — в CPU-тактах)
    void     clockEnvelope();
    void     clockLength();
  } noise;

  // --------------------------------- DMC ------------------------------------
  struct Dmc {
    bool     irqEnable = false, loop = false;
    uint8_t  rateIndex = 0, outputLevel = 0, sampleAddr = 0, sampleLength = 0;
    uint16_t curAddr = 0, bytesRemaining = 0;
    uint8_t  sampleBuffer = 0, shiftReg = 0, bitsRemaining = 0;
    uint16_t timer = 428;
    bool     silence = true;
    uint32_t area = 0;                          // «площадь» уровня (антиалиасинг)
    uint8_t  avg(uint32_t winLen) const;
    void restart(Nes* nes);
    DENDY_INLINE void clockTimer(Nes* nes, uint32_t cycles);
  } dmc;

  // ------------------------------- Тайминг ----------------------------------
  uint32_t frameCounterAcc = 0;              // аккумулятор тактов frame counter
  uint8_t  frameStep = 0;                    // сколько шагов сделано в цикле (0..5)
  bool     fiveStepMode = false, frameIrqInhibit = false;
  uint32_t sampleAcc = 0;                    // аккумулятор генерации 44100 Гц
  uint8_t  channelEnabled = 0;               // биты разрешения каналов из $4015
  // Фаза делителя «такты CPU -> APU-такты» (1 APU-такт = 2 такта CPU).
  // Нужна, чтобы таймеры пульсов шли с частотой APU, а не CPU: иначе пульсы
  // звучат на октаву выше живой приставки (f = CPU/(8*(t+1)) вместо
  // CPU/(16*(t+1))) и залезают выше 22 кГц — это и был «песок» в игре.
  uint8_t  apuPhase = 0;                     // 0/1: «лишний» такт CPU, перенесённый вперёд
  uint32_t winLen = 0;                       // тактов CPU в текущем окне сэмпла
  // Состояние «аналогового» ФНЧ выхода (DENDY_APU_LOWPASS_HZ) — в Q12.
  int32_t  lpf1 = 0, lpf2 = 0;
  uint16_t lpfCoef = 0;                      // коэффициент ФНЧ в Q12 (0 = выключен)

  void    quarterFrame();                    // огибающие + linear counter треугольника
  void    halfFrame();                       // то же + length counter + sweep
  void    resetAreas();                      // начать новое окно сэмпла
  DENDY_FAST_ATTR int16_t mixSample(uint32_t winLen);   // микс + ФВЧ + ФНЧ + громкость

  // Состояние выходного фильтра/громкости — в ЦЕЛЫХ числах (Q12/Q16).
  // СКОРОСТЬ: раньше здесь были float, и на каждый из 44 100 сэмплов в секунду
  // приходилось до четырёх делений с плавающей точкой (нелинейная формула NES).
  // Целочисленный вариант даёт тот же выход (см. mixSample), но без FPU.
  int32_t  dcPrevQ = 0;                      // предыдущий вход ФВЧ, Q12
  int32_t  dcOutQ  = 0;                      // состояние ФВЧ, Q12
  uint32_t gainQ   = 52429;                  // громкость 0..1.1 в Q16 (0.8 по умолчанию)
  uint16_t pulseQ12[31] = {0};               // таблица «сумма пульсов -> уровень», Q12
  bool     mixTblReady = false;
  void     buildMixTable();                  // заполнить pulseQ12 (вызывается из reset)
};
