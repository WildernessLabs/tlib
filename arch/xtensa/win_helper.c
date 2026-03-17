/*
 * Copyright (c) 2011 - 2019, Max Filippov, Open Source and Linux Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Open Source and Linux Lab nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu.h"
#include "osdep.h"
#include "tb-helper.h"

static void copy_window_from_phys(CPUState *env, uint32_t window, uint32_t phys, uint32_t n)
{
    if(phys >= env->config->nareg) {
        tlib_printf(LOG_LEVEL_ERROR,
                    "WIN_ASSERT: copy_window_from_phys phys=%u >= nareg=%u "
                    "(window=%u, n=%u, WB=%u, PC=0x%08x)\n",
                    phys, env->config->nareg, window, n,
                    env->sregs[WINDOW_BASE], env->pc);
        /* Mask to valid range to avoid crash */
        phys = phys % env->config->nareg;
    }
    if(phys + n <= env->config->nareg) {
        memcpy(env->regs + window, env->phys_regs + phys, n * sizeof(uint32_t));
    } else {
        uint32_t n1 = env->config->nareg - phys;
        memcpy(env->regs + window, env->phys_regs + phys, n1 * sizeof(uint32_t));
        memcpy(env->regs + window + n1, env->phys_regs, (n - n1) * sizeof(uint32_t));
    }
}

static void copy_phys_from_window(CPUState *env, uint32_t phys, uint32_t window, uint32_t n)
{
    if(phys >= env->config->nareg) {
        tlib_printf(LOG_LEVEL_ERROR,
                    "WIN_ASSERT: copy_phys_from_window phys=%u >= nareg=%u "
                    "(window=%u, n=%u, WB=%u, PC=0x%08x)\n",
                    phys, env->config->nareg, window, n,
                    env->sregs[WINDOW_BASE], env->pc);
        phys = phys % env->config->nareg;
    }
    if(phys + n <= env->config->nareg) {
        memcpy(env->phys_regs + phys, env->regs + window, n * sizeof(uint32_t));
    } else {
        uint32_t n1 = env->config->nareg - phys;
        memcpy(env->phys_regs + phys, env->regs + window, n1 * sizeof(uint32_t));
        memcpy(env->phys_regs, env->regs + window + n1, (n - n1) * sizeof(uint32_t));
    }
}

static inline unsigned windowbase_bound(unsigned a, const CPUState *env)
{
    return a & (env->config->nareg / 4 - 1);
}

static inline unsigned windowstart_bit(unsigned a, const CPUState *env)
{
    return 1 << windowbase_bound(a, env);
}

void xtensa_sync_window_from_phys(CPUState *env)
{
    copy_window_from_phys(env, 0, env->sregs[WINDOW_BASE] * 4, 16);
}

void xtensa_sync_phys_from_window(CPUState *env)
{
    copy_phys_from_window(env, env->sregs[WINDOW_BASE] * 4, 0, 16);
}

static void xtensa_rotate_window_abs(CPUState *env, uint32_t position)
{
    xtensa_sync_phys_from_window(env);
    env->sregs[WINDOW_BASE] = windowbase_bound(position, env);
    xtensa_sync_window_from_phys(env);
}

void xtensa_rotate_window(CPUState *env, uint32_t delta)
{
    xtensa_rotate_window_abs(env, env->sregs[WINDOW_BASE] + delta);
}

void HELPER(sync_windowbase)(CPUState *env)
{
    uint32_t old_wb = env->sregs[WINDOW_BASE];
    uint32_t new_wb = windowbase_bound(env->windowbase_next, env);

    /* Skip if the rotation was already done (e.g., by HELPER(entry)). */
    if(old_wb == new_wb) {
        return;
    }

    xtensa_rotate_window_abs(env, env->windowbase_next);
}

void HELPER(entry)(CPUState *env, uint32_t pc, uint32_t s, uint32_t imm)
{
    int callinc = (env->sregs[PS] & PS_CALLINC) >> PS_CALLINC_SHIFT;

    env->regs[(callinc << 2) | (s & 3)] = env->regs[s] - imm;
    env->windowbase_next = env->sregs[WINDOW_BASE] + callinc;
    env->sregs[WINDOW_START] |= windowstart_bit(env->windowbase_next, env);


    /* Perform the window rotation here rather than in a separate
     * sync_windowbase helper call.  When sync_windowbase was a separate
     * helper, gen_exit_tb would write stale pre-rotation TCG global values
     * (cached in host registers from the TB start) over the post-rotation
     * values that sync_windowbase set in env->regs.  By doing the rotation
     * in the same helper as the register setup, all env->regs modifications
     * are complete before TCG globals are synced back, and gen_exit_tb
     * won't overwrite them. */
    xtensa_rotate_window_abs(env, env->windowbase_next);
}

void HELPER(window_check)(CPUState *env, uint32_t pc, uint32_t w)
{
    uint32_t windowbase = windowbase_bound(env->sregs[WINDOW_BASE], env);
    uint32_t windowstart = xtensa_replicate_windowstart(env) >> (env->sregs[WINDOW_BASE] + 1);
    uint32_t n = ctz32(windowstart) + 1;

    if(n > w) {
        /* Runtime WINDOWSTART state shows no overflow is needed (the nearest
         * occupied window is farther than w positions away).  This can happen
         * if a cached TB's static window check is stale.  Do NOT clamp n and
         * proceed — that would trigger a spurious overflow that clears a
         * WINDOWSTART bit for an in-use window, corrupting register state.
         * Instead, restart the CPU loop so a fresh TB with correct flags is
         * generated. */
        tlib_printf(LOG_LEVEL_WARNING,
                    "window_check: n=%u > w=%u, no overflow needed "
                    "(WB=%u, WS=0x%08x, PC=0x%08x). Restarting TB lookup.\n",
                    n, w, env->sregs[WINDOW_BASE],
                    env->sregs[WINDOW_START], pc);
        env->pc = pc;
        cpu_loop_exit(env);
    }

    xtensa_rotate_window(env, n);

    env->sregs[PS] = (env->sregs[PS] & ~PS_OWB) | (windowbase << PS_OWB_SHIFT) | PS_EXCM;
    env->sregs[EPC1] = env->pc = pc;

    switch(ctz32(windowstart >> n)) {
        case 0:
            HELPER(exception)(env, EXC_WINDOW_OVERFLOW4);
            break;
        case 1:
            HELPER(exception)(env, EXC_WINDOW_OVERFLOW8);
            break;
        default:
            HELPER(exception)(env, EXC_WINDOW_OVERFLOW12);
            break;
    }
}

void HELPER(test_ill_retw)(CPUState *env, uint32_t pc)
{
    int n = (env->regs[0] >> 30) & 0x3;
    int m = 0;
    uint32_t windowbase = windowbase_bound(env->sregs[WINDOW_BASE], env);
    uint32_t windowstart = env->sregs[WINDOW_START];

    if(windowstart & windowstart_bit(windowbase - 1, env)) {
        m = 1;
    } else if(windowstart & windowstart_bit(windowbase - 2, env)) {
        m = 2;
    } else if(windowstart & windowstart_bit(windowbase - 3, env)) {
        m = 3;
    }

    if(n == 0 && m != 0) {
        /* a0 bits 30-31 are zero, but WINDOWSTART shows a caller at distance m.
         * This happens when WindowOverflow8/12 uses a0 as scratch (l32e a0, a1, -12)
         * and the stack base save area was uninitialized (zero).  On real hardware
         * the stack would have a valid caller-SP with bits 30-31 encoding the call
         * type.  Rather than crash, fix up a0 to use the correct n derived from m,
         * allowing the underflow handler to restore the real register values. */
        tlib_printf(LOG_LEVEL_WARNING,
                    "retw: a0=0x%08x has n=0 but m=%d (WB=%u, WS=0x%08x, PC=0x%08x). "
                    "Fixing a0 bits 30-31 to match m.\n",
                    env->regs[0], m, windowbase, windowstart, pc);
        env->regs[0] = (env->regs[0] & 0x3FFFFFFF) | ((uint32_t)m << 30);
    } else if(n == 0 || (m != 0 && m != n)) {
        tlib_printf(LOG_LEVEL_ERROR,
                    "Illegal retw instruction(pc = %08x), "
                    "PS = %08x, m = %d, n = %d\n",
                    pc, env->sregs[PS], m, n);
        HELPER(exception_cause)(env, pc, ILLEGAL_INSTRUCTION_CAUSE);
    }
}

void HELPER(test_underflow_retw)(CPUState *env, uint32_t pc)
{
    int n = (env->regs[0] >> 30) & 0x3;

    if(!(env->sregs[WINDOW_START] & windowstart_bit(env->sregs[WINDOW_BASE] - n, env))) {
        uint32_t windowbase = windowbase_bound(env->sregs[WINDOW_BASE], env);

        xtensa_rotate_window(env, -n);
        /* window underflow */
        env->sregs[PS] = (env->sregs[PS] & ~PS_OWB) | (windowbase << PS_OWB_SHIFT) | PS_EXCM;
        env->sregs[EPC1] = env->pc = pc;

        if(n == 1) {
            HELPER(exception)(env, EXC_WINDOW_UNDERFLOW4);
        } else if(n == 2) {
            HELPER(exception)(env, EXC_WINDOW_UNDERFLOW8);
        } else if(n == 3) {
            HELPER(exception)(env, EXC_WINDOW_UNDERFLOW12);
        }
    }
}

void HELPER(retw)(CPUState *env, uint32_t a0)
{
    int n = (a0 >> 30) & 0x3;
    xtensa_rotate_window(env, -n);
}

void xtensa_restore_owb(CPUState *env)
{
    uint32_t owb = (env->sregs[PS] & PS_OWB) >> PS_OWB_SHIFT;
    xtensa_rotate_window_abs(env, owb);
}

void HELPER(restore_owb)(CPUState *env)
{
    xtensa_restore_owb(env);
}

void HELPER(restore_owb_no_phys_sync)(CPUState *env)
{
    /* Used by rfwo (window overflow return).  Skip sync_phys_from_window so
     * the overflow handler's scratch register modifications (e.g., WOVF8's
     * l32e a0, a1, -12 which uses a0 as scratch) do NOT propagate to
     * phys_regs.  This preserves the caller's register values in phys_regs
     * — the overflow handler saved the original values to memory via s32e,
     * and rfwo clears the WS bit marking the overflow window as not-live.
     * The underflow handler will restore from memory when needed.
     *
     * Without this, the scratch value overwrites phys_regs at a position
     * shared with the caller's return address (due to the circular register
     * file), corrupting the caller's registers. */
    uint32_t owb = (env->sregs[PS] & PS_OWB) >> PS_OWB_SHIFT;
    env->sregs[WINDOW_BASE] = windowbase_bound(owb, env);
    xtensa_sync_window_from_phys(env);
}

void HELPER(movsp)(CPUState *env, uint32_t pc)
{
    if((env->sregs[WINDOW_START] &
        (windowstart_bit(env->sregs[WINDOW_BASE] - 3, env) | windowstart_bit(env->sregs[WINDOW_BASE] - 2, env) |
         windowstart_bit(env->sregs[WINDOW_BASE] - 1, env))) == 0) {
        HELPER(exception_cause)(env, pc, ALLOCA_CAUSE);
    }
}
