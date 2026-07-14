/*
 * Hardware timer + debug console test.
 *
 * This de-risks the single most likely way Doom hangs on this core. doomgeneric's RISC-V port
 * implements DG_SleepMs as a busy-wait on the microsecond timer at 0x08000004:
 *
 *     void DG_SleepMs(uint32_t ms) {
 *         uint32_t start = *((uint32_t*)0x8000004) / 1000;
 *         uint32_t now   = start;
 *         while (now - start < ms) { now = *((uint32_t*)0x8000004) / 1000; }
 *     }
 *
 * If the timer never advances, that loop NEVER exits and Doom hangs on its first frame. So we
 * reproduce the exact pattern here.
 *
 * It also exercises the two debug MMIO ports, which is how we will watch Doom boot: writing to
 * them makes the Chisel `printf` in Memory.scala emit to the simulation console.
 *
 * Expected result: 0x900D
 */

#include <stdint.h>

#define TIMER_US  ((volatile uint32_t *)0x08000004)
#define DBG_CHAR  ((volatile uint32_t *)0x70000000)
#define DBG_NUM   ((volatile uint32_t *)0x70000008)

static void dbg_puts(const char *s)
{
    while (*s) {
        *DBG_CHAR = (uint32_t)*s;
        s++;
    }
}

int main(void)
{
    dbg_puts("timer test\n");

    /* 1. The timer must actually advance. */
    uint32_t t0 = *TIMER_US;
    for (volatile int i = 0; i < 500; i++) { /* burn cycles */ }
    uint32_t t1 = *TIMER_US;

    if (t1 == t0) {
        dbg_puts("FAIL: timer is frozen\n");
        return 0xDEAD;
    }

    /* 2. It must move forwards. */
    if (t1 < t0) {
        dbg_puts("FAIL: timer went backwards\n");
        return 0xDEAD;
    }

    /* 3. The exact DG_SleepMs busy-wait must TERMINATE. This is the Doom hang scenario. */
    uint32_t start = *TIMER_US / 1000;
    uint32_t now = start;
    while (now - start < 2) { /* sleep 2 ms */
        now = *TIMER_US / 1000;
    }

    dbg_puts("elapsed_us=");
    *DBG_NUM = *TIMER_US;
    dbg_puts("\nok\n");

    return 0x900D;
}
