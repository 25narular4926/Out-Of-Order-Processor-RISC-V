/*
 * RV32IM arithmetic smoke test -- compiled C exercising the ALU and the multi-cycle MULDIV unit.
 *
 * Verifies that gcc-generated code for mul / div / rem actually round-trips through the
 * out-of-order issue queue, the iterative divider FSM, and the CDB wakeup path.
 *
 * Expected result: 0x600D  (see the accumulation below)
 */

#include <stdint.h>

static volatile int32_t sink; /* defeat constant folding */

static int32_t mul_(int32_t a, int32_t b) { sink = a; return sink * b; }
static int32_t div_(int32_t a, int32_t b) { sink = a; return sink / b; }
static int32_t rem_(int32_t a, int32_t b) { sink = a; return sink % b; }

int main(void)
{
    int32_t acc = 0;

    if (mul_(6, 7) != 42)        return 0xBAD1; /* MUL   */
    if (mul_(-3, 5) != -15)      return 0xBAD2; /* MUL, signed */
    if (div_(-20, 3) != -6)      return 0xBAD3; /* DIV, truncate toward zero */
    if (div_(20, 3) != 6)        return 0xBAD4; /* DIV   */
    if (rem_(-20, 3) != -2)      return 0xBAD5; /* REM takes sign of dividend */

    /* a loop, to exercise branches + the ROB retiring many ops in order */
    for (int32_t i = 1; i <= 10; i++)
        acc += i * i;            /* 1+4+9+...+100 = 385 */

    if (acc != 385)              return 0xBAD6;

    return 0x600D;
}
