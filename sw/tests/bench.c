/*
 * Branch-heavy benchmark: count the primes below 500 by trial division.
 *
 * This exists to measure the BRANCH PREDICTOR, so it is written to be un-foldable by the
 * compiler (unlike arith.c, where gcc constant-folds the loop away and turns `/3` into a
 * multiply-shift, leaving almost no branches to predict).
 *
 * What it exercises:
 *   - a hot, highly-predictable inner loop (the trial-division loop) -- a good predictor should
 *     learn its backward branch is almost always taken,
 *   - a data-dependent branch (`n % d == 0`) that is genuinely hard to predict,
 *   - real hardware REM operations, which take 32 cycles on the iterative divider and so give
 *     the out-of-order engine something to hide latency behind,
 *   - a call/return per outer iteration.
 *
 * Expected result: pi(500) = 95 primes below 500.
 */

#include <stdint.h>

static volatile int32_t sink; /* keep the result observable so nothing is optimised out */

static int is_prime(int32_t n)
{
    if (n < 2) {
        return 0;
    }
    for (int32_t d = 2; d * d <= n; d++) {
        if (n % d == 0) {
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    int32_t count = 0;

    for (int32_t n = 0; n < 500; n++) {
        if (is_prime(n)) {
            count++;
        }
    }

    sink = count;
    return count; /* 95 */
}
