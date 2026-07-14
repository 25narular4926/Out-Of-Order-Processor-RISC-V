/*
 * Custom riscv-tests environment for the OoO RISC-V core.
 *
 * The official environment (riscv-tests/env/p) boots through machine-mode CSRs and exits via
 * `ecall` into an HTIF tohost word -- it uses csrr/csrw/csrwi/ecall/mret in ~36 places. This
 * core implements NONE of that: there are no CSRs, no ecall, and no trap handling.
 *
 * The TEST BODIES, however, are pure RV32I/RV32M and are perfectly runnable. Only the
 * boot/exit scaffolding needs replacing, and the tests reach for just six macros. So we supply
 * our own:
 *
 *   - boot:  the test itself IS _start, linked at address 0 (the core's reset PC).
 *   - exit:  store a result word to the tohost mailbox at 0x70000010 and spin. That is a
 *            dedicated MMIO word (see OoOParams), well above RAM, so Memory routes it out of
 *            the RAM array to the testbench (same trick crt0.S uses).
 *
 * RESULT PROTOCOL (matches riscv-tests convention):
 *   pass -> 1
 *   fail -> (TESTNUM << 1) | 1     ... so the failing test number is recoverable as (x >> 1)
 * Any other value, or no write at all, means the core hung.
 */

#ifndef _ENV_OOO_RISCV_TEST_H
#define _ENV_OOO_RISCV_TEST_H

/* riscv-tests keeps the current sub-test index in gp (x3). */
#define TESTNUM gp

/* The core is always in "machine mode" (there is only one mode), so the ISA/priv-level
 * selector macros are no-ops. The tests #undef/redefine RVTEST_RV64U -> RVTEST_RV32U. */
#define RVTEST_RV32U
#define RVTEST_RV64U
#define RVTEST_RV32M
#define RVTEST_RV64M
#define RVTEST_RV32S
#define RVTEST_RV64S

/* tohost mailbox: byte 0x70000010 == OoOParams.tohostByte */
#define OOO_TOHOST 0x70000010

#define RVTEST_CODE_BEGIN                                                     \
        .section .text.init;                                                  \
        .align 6;                                                             \
        .globl _start;                                                        \
_start:

#define RVTEST_CODE_END

/* pass: write 1 */
#define RVTEST_PASS                                                           \
        li   a0, 1;                                                           \
        li   a1, OOO_TOHOST;                                                  \
        sw   a0, 0(a1);                                                       \
9998:   j 9998b;

/* fail: write (TESTNUM << 1) | 1 */
#define RVTEST_FAIL                                                           \
        slli a0, TESTNUM, 1;                                                  \
        ori  a0, a0, 1;                                                       \
        li   a1, OOO_TOHOST;                                                  \
        sw   a0, 0(a1);                                                       \
9999:   j 9999b;

#define RVTEST_DATA_BEGIN                                                     \
        .data;                                                                \
        .align 4;                                                             \
        .global begin_signature;                                              \
begin_signature:

#define RVTEST_DATA_END                                                       \
        .align 4;                                                             \
        .global end_signature;                                                \
end_signature:

#endif /* _ENV_OOO_RISCV_TEST_H */
