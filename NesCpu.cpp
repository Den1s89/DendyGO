/**
 * =============================================================================
 *  DendyGO / NesCpu.cpp
 *  Полная реализация всех официальных инструкций 6502 (Ricoh 2A03).
 *  Такты указаны точно, включая штраф за переход через страницу адреса.
 * =============================================================================
 */
#include "DendyFast.h"            // -O3 для горячего кода (см. DendyConfig.h)
#include "NesCpu.h"
#include "NesCore.h"

// ------------------------------- Флаги регистра P ----------------------------
#define F_C 0x01   // перенос
#define F_Z 0x02   // ноль
#define F_I 0x04   // запрет IRQ
#define F_D 0x08   // десятичный режим (в 2A03 не используется)
#define F_B 0x10   // break
#define F_U 0x20   // всегда 1
#define F_V 0x40   // переполнение
#define F_N 0x80   // знак

// ------------------------------- Микро-макросы -------------------------------
#define DO_ADC(val) { \
    const uint8_t _v = (val); \
    const uint16_t _sum = (uint16_t)a + _v + (uint16_t)(p & F_C); \
    setFlag(F_V, ((~(a ^ _v) & (a ^ (uint8_t)_sum)) & 0x80) != 0); \
    setFlag(F_C, _sum > 0xFF); \
    a = (uint8_t)_sum; \
    setZN(a); }

#define DO_SBC(val)      { DO_ADC((uint8_t)(~(val))); }
#define DO_AND(val)      { a = (uint8_t)(a & (val)); setZN(a); }
#define DO_ORA(val)      { a = (uint8_t)(a | (val)); setZN(a); }
#define DO_EOR(val)      { a = (uint8_t)(a ^ (val)); setZN(a); }
#define DO_CMP(r, val)   { const uint8_t _v = (val); setFlag(F_C, (r) >= _v); setZN((uint8_t)((r) - _v)); }
#define DO_ASL(v)        { const uint8_t _o = (v); setFlag(F_C, (_o & 0x80) != 0); (v) = (uint8_t)(_o << 1); setZN(v); }
#define DO_LSR(v)        { const uint8_t _o = (v); setFlag(F_C, (_o & 0x01) != 0); (v) = (uint8_t)(_o >> 1); setZN(v); }
#define DO_ROL(v)        { const uint8_t _o = (v); const bool _c = (p & F_C) != 0; \
                           setFlag(F_C, (_o & 0x80) != 0); (v) = (uint8_t)((_o << 1) | (_c ? 1 : 0)); setZN(v); }
#define DO_ROR(v)        { const uint8_t _o = (v); const bool _c = (p & F_C) != 0; \
                           setFlag(F_C, (_o & 0x01) != 0); (v) = (uint8_t)((_o >> 1) | (_c ? 0x80 : 0)); setZN(v); }
#define DO_INC(v)        { (v) = (uint8_t)((v) + 1); setZN(v); }
#define DO_DEC(v)        { (v) = (uint8_t)((v) - 1); setZN(v); }
// ВАЖНО: в конце обязателен break! Без него выполнение после ветвления
// «проваливается» в следующий case (обработчик (zp),Y): он съедал лишний байт
// (PC уезжал на +1 — CPU уходил в данные и на RTS возвращался в мусор),
// делал лишнее чтение, а для BCC/BVC-ветки ещё и запись в память.
#define BRANCH(cond)     { int8_t _off = (int8_t)fetch8(nes); \
                           if (cond) { const uint16_t _old = pc; pc = (uint16_t)(pc + _off); \
                             cycles += (((_old ^ pc) & 0xFF00) ? 4 : 3); } else cycles += 2; break; }

// ============================ Вспомогательные ================================
// ВАЖНО (скорость): это самые горячие функции всей эмуляции — они вызываются
// 3..6 раз на КАЖДУЮ инструкцию 6502 (fetch опкода, чтение/запись операнда,
// стек, флаги). В логе [GAME] profil это видно как `cpu ~7 мс/кадр` — при
// ~10 000 инструкций за кадр это ~175 тактов на инструкцию, хотя сама работа
// тянет тактов на 50-60.
// ПРИЧИНА: тела были обычными функциями, а GCC в огромном switch() внутри
// step() перестаёт встраивать их после первых сотен мест. Каждое обращение к
// памяти превращалось в НАСТОЯЩИЙ вызов функции: на Xtensa это переключение
// регистрового окна (call8/retw) плюс промах предсказания — ~20..35 тактов,
// и таких вызовов 3..6 на инструкцию.
// ЛЕЧЕНИЕ: DENDY_INLINE = `inline __attribute__((always_inline))` (DendyFast.h) —
// тело встраивается ГАРАНТИРОВАННО, ровно как у readPrg()/readChr() (см. пояснение
// в NesMapper.h, там та же история и та же болезнь была вылечена так же).
// Определения при этом остаются в этом файле: снаружи их никто не вызывает
// (проверено: других пользователей rd/wr/am*/push/pull/setZN нет).
DENDY_INLINE uint8_t NesCpu::rd(Nes* nes, uint16_t addr)  { return nesRead(nes, addr); }
DENDY_INLINE void    NesCpu::wr(Nes* nes, uint16_t addr, uint8_t value) { nesWrite(nes, addr, value); }
DENDY_INLINE uint8_t NesCpu::fetch8(Nes* nes)  { return rd(nes, pc++); }
DENDY_INLINE uint16_t NesCpu::fetch16(Nes* nes) {
  const uint16_t lo = rd(nes, pc++);
  return (uint16_t)(lo | (rd(nes, pc++) << 8));
}

DENDY_INLINE uint16_t NesCpu::amZp(Nes* nes)  { return fetch8(nes); }
DENDY_INLINE uint16_t NesCpu::amZpX(Nes* nes) { return (uint16_t)((fetch8(nes) + x) & 0xFF); }
DENDY_INLINE uint16_t NesCpu::amZpY(Nes* nes) { return (uint16_t)((fetch8(nes) + y) & 0xFF); }
DENDY_INLINE uint16_t NesCpu::amAbs(Nes* nes) { return fetch16(nes); }
DENDY_INLINE uint16_t NesCpu::amAbsX(Nes* nes) {
  const uint16_t b = fetch16(nes); const uint16_t r = (uint16_t)(b + x);
  pageCross = ((b ^ r) & 0xFF00) != 0; return r;
}
DENDY_INLINE uint16_t NesCpu::amAbsY(Nes* nes) {
  const uint16_t b = fetch16(nes); const uint16_t r = (uint16_t)(b + y);
  pageCross = ((b ^ r) & 0xFF00) != 0; return r;
}
DENDY_INLINE uint16_t NesCpu::amIndX(Nes* nes) {
  const uint8_t z = (uint8_t)(fetch8(nes) + x);
  return (uint16_t)(rd(nes, z) | (rd(nes, (uint8_t)(z + 1)) << 8));
}
DENDY_INLINE uint16_t NesCpu::amIndY(Nes* nes) {
  const uint8_t z = fetch8(nes);
  const uint16_t b = (uint16_t)(rd(nes, z) | (rd(nes, (uint8_t)(z + 1)) << 8));
  const uint16_t r = (uint16_t)(b + y);
  pageCross = ((b ^ r) & 0xFF00) != 0; return r;
}

DENDY_INLINE void    NesCpu::push(Nes* nes, uint8_t value) { wr(nes, (uint16_t)(0x0100 + sp--), value); }
DENDY_INLINE uint8_t NesCpu::pull(Nes* nes) { return rd(nes, (uint16_t)(0x0100 + (++sp))); }
DENDY_INLINE void    NesCpu::push16(Nes* nes, uint16_t value) {
  push(nes, (uint8_t)(value >> 8)); push(nes, (uint8_t)value);
}
DENDY_INLINE uint16_t NesCpu::pull16(Nes* nes) {
  const uint8_t lo = pull(nes);
  return (uint16_t)(lo | (pull(nes) << 8));
}
// Флаги: setZN() вызывается на КАЖДОЙ инструкции (≈10 000 раз за кадр), setFlag() —
// ещё 1..3 раза. Обычная функция здесь стоила бы дороже самой операции, поэтому
// тела встраиваются принудительно (см. пояснение у rd()/wr() выше).
DENDY_INLINE void NesCpu::setZN(uint8_t value) {
  p = (uint8_t)((p & ~(F_Z | F_N)) | (value == 0 ? F_Z : 0) | (value & F_N));
}
DENDY_INLINE void NesCpu::setFlag(uint8_t mask, bool value) {
  if (value) p |= mask; else p &= (uint8_t)~mask;
}

// ================================= Сброс =====================================
void NesCpu::reset(Nes* nes)
{
  a = x = y = 0;
  sp = 0xFD;
  p  = F_U | F_I;
  irqLine = false;
  nmiPending = false;
  irqCount = nmiCount = 0;
  instrCount = 0;
  pcTraceIdx = 0;
  for (int i = 0; i < TRACE_LEN; i++) pcTrace[i] = 0;
  pc = (uint16_t)(rd(nes, 0xFFFC) | (rd(nes, 0xFFFD) << 8));   // вектор сброса
}

// ============================ Прерывания =====================================
void NesCpu::doNmi(Nes* nes)
{
  push16(nes, pc);
  push(nes, (uint8_t)((p & ~F_B) | F_U));
  setFlag(F_I, true);
  pc = (uint16_t)(rd(nes, 0xFFFA) | (rd(nes, 0xFFFB) << 8));
  cycles += 7;
}

void NesCpu::doIrq(Nes* nes)
{
  push16(nes, pc);
  push(nes, (uint8_t)((p & ~F_B) | F_U));
  setFlag(F_I, true);
  pc = (uint16_t)(rd(nes, 0xFFFE) | (rd(nes, 0xFFFF) << 8));
  cycles += 7;
}

// ---------------------- Шаблоны адресации для инструкций ---------------------
// op(v) — операция над прочитанным байтом v; такты учитываются точно.
#define RD_IMM(op)   { const uint8_t v = fetch8(nes);         op(v); cycles += 2; break; }
#define RD_ZP(op)    { const uint8_t v = rd(nes, amZp(nes));  op(v); cycles += 3; break; }
#define RD_ZPX(op)   { const uint8_t v = rd(nes, amZpX(nes)); op(v); cycles += 4; break; }
#define RD_ZPY(op)   { const uint8_t v = rd(nes, amZpY(nes)); op(v); cycles += 4; break; }
#define RD_ABS(op)   { const uint8_t v = rd(nes, amAbs(nes)); op(v); cycles += 4; break; }
#define RD_ABSX(op)  { const uint16_t q = amAbsX(nes); const uint8_t v = rd(nes, q); op(v); \
                       cycles += 4 + (pageCross ? 1 : 0); break; }
#define RD_ABSY(op)  { const uint16_t q = amAbsY(nes); const uint8_t v = rd(nes, q); op(v); \
                       cycles += 4 + (pageCross ? 1 : 0); break; }
#define RD_INDX(op)  { const uint8_t v = rd(nes, amIndX(nes)); op(v); cycles += 6; break; }
#define RD_INDY(op)  { const uint16_t q = amIndY(nes); const uint8_t v = rd(nes, q); op(v); \
                       cycles += 5 + (pageCross ? 1 : 0); break; }

#define WR_ZP(val)   { wr(nes, amZp(nes),   (val)); cycles += 3; break; }
#define WR_ZPX(val)  { wr(nes, amZpX(nes),  (val)); cycles += 4; break; }
#define WR_ZPY(val)  { wr(nes, amZpY(nes),  (val)); cycles += 4; break; }
#define WR_ABS(val)  { wr(nes, amAbs(nes),  (val)); cycles += 4; break; }
#define WR_ABSX(val) { wr(nes, amAbsX(nes), (val)); cycles += 5; break; }
#define WR_ABSY(val) { wr(nes, amAbsY(nes), (val)); cycles += 5; break; }
#define WR_INDX(val) { wr(nes, amIndX(nes), (val)); cycles += 6; break; }
#define WR_INDY(val) { wr(nes, amIndY(nes), (val)); cycles += 6; break; }

#define RMW_ZP(op)   { ea = amZp(nes);  uint8_t v = rd(nes, ea); op(v); wr(nes, ea, v); cycles += 5; break; }
#define RMW_ZPX(op)  { ea = amZpX(nes); uint8_t v = rd(nes, ea); op(v); wr(nes, ea, v); cycles += 6; break; }
#define RMW_ABS(op)  { ea = amAbs(nes); uint8_t v = rd(nes, ea); op(v); wr(nes, ea, v); cycles += 6; break; }
#define RMW_ABSX(op) { ea = amAbsX(nes);uint8_t v = rd(nes, ea); op(v); wr(nes, ea, v); cycles += 7; break; }

#define DO_LDA(val)  { a = (val); setZN(a); }
#define DO_LDX(val)  { x = (val); setZN(x); }
#define DO_LDY(val)  { y = (val); setZN(y); }
#define DO_CMPA(val) DO_CMP(a, val)
#define DO_CMPX(val) DO_CMP(x, val)
#define DO_CMPY(val) DO_CMP(y, val)
#define DO_BIT(val)  { setFlag(F_Z, ((a & (val)) == 0)); setFlag(F_N, ((val) & 0x80) != 0); \
                       setFlag(F_V, ((val) & 0x40) != 0); }

// ============================== Основной шаг =================================
DENDY_FAST_ATTR void NesCpu::step(Nes* nes)
{
  // --- Трасса адресов инструкций (диагностика, только DENDY_VIDEO_TRACE) -------
  // Кольцо трассы (последние адреса инструкций) пишется только под DENDY_VIDEO_TRACE:
  // через step() проходит ~10 000 инструкций за кадр, и лишние записи в кольцо
  // молча отнимали время у эмуляции.
  // СЧЁТЧИК ЖЕ инструкций ведём ВСЕГДА: это одна инкрементация на инструкцию, а в
  // строке [GAME] profil по нему видно `instrukcij` за кадр — то есть сколько
  // тактов реально уходит на одну инструкцию 6502 (норма — 20..40). Без него
  // большой `cpu` невозможно объяснить: интерпретатор это или что-то забирает
  // время внутри step().
  ++instrCount;
#if DENDY_VIDEO_TRACE
  pcTrace[pcTraceIdx & (TRACE_LEN - 1)] = pc;
  pcTraceIdx = (uint8_t)((pcTraceIdx + 1) & (TRACE_LEN - 1));
#endif

  // --- Обработка прерываний перед инструкцией ---
  if (nmiPending) { nmiPending = false; ++nmiCount; doNmi(nes); }
  else if (irqLine && ((p & F_I) == 0)) { ++irqCount; doIrq(nes); }

  const uint8_t op = fetch8(nes);
  pageCross = false;

  switch (op) {
    // ---------------------------- 0x00 - 0x1F ----------------------------
    case 0x00:                                                   // BRK
      pc++;
      push16(nes, pc);
      push(nes, (uint8_t)(p | F_B | F_U));
      setFlag(F_I, true);
      pc = (uint16_t)(rd(nes, 0xFFFE) | (rd(nes, 0xFFFF) << 8));
      cycles += 7; break;
    case 0x01: RD_INDX(DO_ORA);
    case 0x05: RD_ZP(DO_ORA);
    case 0x06: RMW_ZP(DO_ASL);
    case 0x08: push(nes, (uint8_t)(p | F_B | F_U)); cycles += 3; break;      // PHP
    case 0x09: RD_IMM(DO_ORA);
    case 0x0A: DO_ASL(a); cycles += 2; break;                                // ASL A
    case 0x0D: RD_ABS(DO_ORA);
    case 0x0E: RMW_ABS(DO_ASL);
    case 0x10: BRANCH(!(p & F_N));                                           // BPL
    case 0x11: RD_INDY(DO_ORA);
    case 0x15: RD_ZPX(DO_ORA);
    case 0x16: RMW_ZPX(DO_ASL);
    case 0x18: setFlag(F_C, false); cycles += 2; break;                      // CLC
    case 0x19: RD_ABSY(DO_ORA);
    case 0x1D: RD_ABSX(DO_ORA);
    case 0x1E: RMW_ABSX(DO_ASL);

    // ---------------------------- 0x20 - 0x3F ----------------------------
    case 0x20:                                                   // JSR
      { const uint16_t target = fetch16(nes);
        push16(nes, (uint16_t)(pc - 1));
        pc = target; cycles += 6; } break;
    case 0x21: RD_INDX(DO_AND);
    case 0x24: RD_ZP(DO_BIT);
    case 0x25: RD_ZP(DO_AND);
    case 0x26: RMW_ZP(DO_ROL);
    case 0x28: p = (uint8_t)((pull(nes) & ~F_B) | F_U); cycles += 4; break;  // PLP
    case 0x29: RD_IMM(DO_AND);
    case 0x2A: DO_ROL(a); cycles += 2; break;                                // ROL A
    case 0x2C: RD_ABS(DO_BIT);
    case 0x2D: RD_ABS(DO_AND);
    case 0x2E: RMW_ABS(DO_ROL);
    case 0x30: BRANCH(p & F_N);                                              // BMI
    case 0x31: RD_INDY(DO_AND);
    case 0x35: RD_ZPX(DO_AND);
    case 0x36: RMW_ZPX(DO_ROL);
    case 0x38: setFlag(F_C, true); cycles += 2; break;                       // SEC
    case 0x39: RD_ABSY(DO_AND);
    case 0x3D: RD_ABSX(DO_AND);
    case 0x3E: RMW_ABSX(DO_ROL);

    // ---------------------------- 0x40 - 0x5F ----------------------------
    case 0x40:                                                   // RTI
      p = (uint8_t)((pull(nes) & ~F_B) | F_U);
      pc = pull16(nes); cycles += 6; break;
    case 0x41: RD_INDX(DO_EOR);
    case 0x45: RD_ZP(DO_EOR);
    case 0x46: RMW_ZP(DO_LSR);
    case 0x48: push(nes, a); cycles += 3; break;                             // PHA
    case 0x49: RD_IMM(DO_EOR);
    case 0x4A: DO_LSR(a); cycles += 2; break;                                // LSR A
    case 0x4C: pc = fetch16(nes); cycles += 3; break;                        // JMP abs
    case 0x4D: RD_ABS(DO_EOR);
    case 0x4E: RMW_ABS(DO_LSR);
    case 0x50: BRANCH(!(p & F_V));                                           // BVC
    case 0x51: RD_INDY(DO_EOR);
    case 0x55: RD_ZPX(DO_EOR);
    case 0x56: RMW_ZPX(DO_LSR);
    case 0x58: setFlag(F_I, false); cycles += 2; break;                      // CLI
    case 0x59: RD_ABSY(DO_EOR);
    case 0x5D: RD_ABSX(DO_EOR);
    case 0x5E: RMW_ABSX(DO_LSR);

    // ---------------------------- 0x60 - 0x7F ----------------------------
    case 0x60: pc = (uint16_t)(pull16(nes) + 1); cycles += 6; break;         // RTS
    case 0x61: RD_INDX(DO_ADC);
    case 0x65: RD_ZP(DO_ADC);
    case 0x66: RMW_ZP(DO_ROR);
    case 0x68: a = pull(nes); setZN(a); cycles += 4; break;                  // PLA
    case 0x69: RD_IMM(DO_ADC);
    case 0x6A: DO_ROR(a); cycles += 2; break;                                // ROR A
    case 0x6C:                                                   // JMP (ind) — с багом страницы
      { const uint16_t ptr = fetch16(nes);
        const uint16_t lo = rd(nes, ptr);
        const uint16_t hi = rd(nes, (uint16_t)((ptr & 0xFF00) | ((ptr + 1) & 0x00FF)));
        pc = (uint16_t)(lo | (hi << 8)); cycles += 5; } break;
    case 0x6D: RD_ABS(DO_ADC);
    case 0x6E: RMW_ABS(DO_ROR);
    case 0x70: BRANCH(p & F_V);                                              // BVS
    case 0x71: RD_INDY(DO_ADC);
    case 0x75: RD_ZPX(DO_ADC);
    case 0x76: RMW_ZPX(DO_ROR);
    case 0x78: setFlag(F_I, true); cycles += 2; break;                       // SEI
    case 0x79: RD_ABSY(DO_ADC);
    case 0x7D: RD_ABSX(DO_ADC);
    case 0x7E: RMW_ABSX(DO_ROR);

    // ---------------------------- 0x80 - 0x9F ----------------------------
    case 0x81: WR_INDX(a);
    case 0x84: WR_ZP(y);
    case 0x85: WR_ZP(a);
    case 0x86: WR_ZP(x);
    case 0x88: DO_DEC(y); cycles += 2; break;                                // DEY
    case 0x8A: a = x; setZN(a); cycles += 2; break;                          // TXA
    case 0x8C: WR_ABS(y);
    case 0x8D: WR_ABS(a);
    case 0x8E: WR_ABS(x);
    case 0x90: BRANCH(!(p & F_C));                                           // BCC
    case 0x91: WR_INDY(a);
    case 0x94: WR_ZPX(y);
    case 0x95: WR_ZPX(a);
    case 0x96: WR_ZPY(x);
    case 0x98: a = y; setZN(a); cycles += 2; break;                          // TYA
    case 0x99: WR_ABSY(a);
    case 0x9A: sp = x; cycles += 2; break;                                   // TXS
    case 0x9D: WR_ABSX(a);

    // ---------------------------- 0xA0 - 0xBF ----------------------------
    case 0xA0: RD_IMM(DO_LDY);
    case 0xA1: RD_INDX(DO_LDA);
    case 0xA2: RD_IMM(DO_LDX);
    case 0xA4: RD_ZP(DO_LDY);
    case 0xA5: RD_ZP(DO_LDA);
    case 0xA6: RD_ZP(DO_LDX);
    case 0xA8: y = a; setZN(y); cycles += 2; break;                          // TAY
    case 0xA9: RD_IMM(DO_LDA);
    case 0xAA: x = a; setZN(x); cycles += 2; break;                          // TAX
    case 0xAC: RD_ABS(DO_LDY);
    case 0xAD: RD_ABS(DO_LDA);
    case 0xAE: RD_ABS(DO_LDX);
    case 0xB0: BRANCH(p & F_C);                                              // BCS
    case 0xB1: RD_INDY(DO_LDA);
    case 0xB4: RD_ZPX(DO_LDY);
    case 0xB5: RD_ZPX(DO_LDA);
    case 0xB6: RD_ZPY(DO_LDX);
    case 0xB8: setFlag(F_V, false); cycles += 2; break;                      // CLV
    case 0xB9: RD_ABSY(DO_LDA);
    case 0xBA: x = sp; setZN(x); cycles += 2; break;                         // TSX
    case 0xBC: RD_ABSX(DO_LDY);
    case 0xBD: RD_ABSX(DO_LDA);
    case 0xBE: RD_ABSY(DO_LDX);

    // ---------------------------- 0xC0 - 0xDF ----------------------------
    case 0xC0: RD_IMM(DO_CMPY);
    case 0xC1: RD_INDX(DO_CMPA);
    case 0xC4: RD_ZP(DO_CMPY);
    case 0xC5: RD_ZP(DO_CMPA);
    case 0xC6: RMW_ZP(DO_DEC);
    case 0xC8: DO_INC(y); cycles += 2; break;                                // INY
    case 0xC9: RD_IMM(DO_CMPA);
    case 0xCA: DO_DEC(x); cycles += 2; break;                                // DEX
    case 0xCC: RD_ABS(DO_CMPY);
    case 0xCD: RD_ABS(DO_CMPA);
    case 0xCE: RMW_ABS(DO_DEC);
    case 0xD0: BRANCH(!(p & F_Z));                                           // BNE
    case 0xD1: RD_INDY(DO_CMPA);
    case 0xD5: RD_ZPX(DO_CMPA);
    case 0xD6: RMW_ZPX(DO_DEC);
    case 0xD8: setFlag(F_D, false); cycles += 2; break;                      // CLD
    case 0xD9: RD_ABSY(DO_CMPA);
    case 0xDD: RD_ABSX(DO_CMPA);
    case 0xDE: RMW_ABSX(DO_DEC);

    // ---------------------------- 0xE0 - 0xFF ----------------------------
    case 0xE0: RD_IMM(DO_CMPX);
    case 0xE1: RD_INDX(DO_SBC);
    case 0xE4: RD_ZP(DO_CMPX);
    case 0xE5: RD_ZP(DO_SBC);
    case 0xE6: RMW_ZP(DO_INC);
    case 0xE8: DO_INC(x); cycles += 2; break;                                // INX
    case 0xE9: RD_IMM(DO_SBC);
    case 0xEA: cycles += 2; break;                                           // NOP
    case 0xEC: RD_ABS(DO_CMPX);
    case 0xED: RD_ABS(DO_SBC);
    case 0xEE: RMW_ABS(DO_INC);
    case 0xF0: BRANCH(p & F_Z);                                              // BEQ
    case 0xF1: RD_INDY(DO_SBC);
    case 0xF5: RD_ZPX(DO_SBC);
    case 0xF6: RMW_ZPX(DO_INC);
    case 0xF8: setFlag(F_D, true); cycles += 2; break;                       // SED
    case 0xF9: RD_ABSY(DO_SBC);
    case 0xFD: RD_ABSX(DO_SBC);
    case 0xFE: RMW_ABSX(DO_INC);

    // ------------- Неофициальные NOP: важна только длина -------------
    case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA:        // 1 байт
      cycles += 2; break;
    case 0x04: case 0x44: case 0x64:                                         // zp
      fetch8(nes); cycles += 3; break;
    case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4:        // zp,X
      fetch8(nes); cycles += 4; break;
    case 0x0C: case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC:  // abs,X
      fetch16(nes); cycles += 4; break;
    case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2:                   // imm
      fetch8(nes); cycles += 2; break;

    default:                                                                 // прочее -> NOP
      cycles += 2; break;
  }
}
