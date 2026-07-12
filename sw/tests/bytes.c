/*
 * Byte/halfword store test -- compiled C exercising the LSU's sub-word store path.
 *
 * This is the regression that matters for running any real C program (and eventually Doom):
 * the compiler emits `sb` / `sh` constantly (strings, byte buffers, 8-bit framebuffers), and
 * the LSU previously spliced sub-word stores over a ZERO base word, destroying the surrounding
 * bytes. With per-byte write enables in Memory, each store must touch only its own lanes.
 *
 * Expected result: 0xEEBB11DD
 *   start 0xAABBCCDD  (lane0=DD lane1=CC lane2=BB lane3=AA, little-endian)
 *   sb 0x11 -> lane 1     => 0xAABB11DD
 *   sb 0xEE -> lane 3     => 0xEEBB11DD
 * If sub-word stores clobbered their neighbours, the surrounding bytes would come back zero.
 */

#include <stdint.h>

/* volatile so the compiler actually emits the loads/stores instead of folding them away */
static volatile uint32_t word;

int main(void)
{
    volatile uint8_t *b = (volatile uint8_t *)&word;

    word = 0xAABBCCDDu; /* sw */
    b[1] = 0x11;        /* sb -- must preserve lanes 0, 2, 3 */
    b[3] = 0xEE;        /* sb -- must preserve lanes 0, 1, 2 */

    return (int)word;   /* lw -> 0xEEBB11DD */
}
