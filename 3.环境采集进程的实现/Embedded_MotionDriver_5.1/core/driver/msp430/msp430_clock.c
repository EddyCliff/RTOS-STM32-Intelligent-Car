/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_clock.c $
 *****************************************************************************/
/**
 *  @defgroup MSP430-SL
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_clock.h
 *      @brief      Functions to configure the MSP430 system clock to settings
 *                  required for eMPL.
 *      @details    ACLK, MCLK, and SMCLK are all sourced directly by the DCO.
 *                  The DCO frequency is set by multiplying the internal
 *                  32.768kHz oscillator, which may vary in performance between
 *                  multiple chips. This may be an issue for time-critical
 *                  tasks such as providing a baud rate reference.
 */

#include "stdio.h"

#include "msp430.h"
#include "msp430_clock.h"
#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"

#if !defined __MSP430F5528__ && !defined __MSP430F5529__ && \
    !defined __MSP430F5229__
#error  The system clock driver does not support this MSP430.
#endif

struct msp430_clock_s {
    volatile uint32_t timestamp;
    unsigned long mclk;
    unsigned long smclk;
    unsigned long aclk;
    unsigned short ms_per_interrupt;
    unsigned char enabled;
    unsigned short ticks_per_interrupt;
    unsigned long timer_remaining_ms;
    void (*timer_cb)(void);
};
static struct msp430_clock_s clock = {
    .enabled = 0,
    .timer_remaining_ms = 0,
    .timer_cb = NULL
};

#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR (void)
{
    TBCCR0 += clock.ticks_per_interrupt;
    clock.timestamp += clock.ms_per_interrupt;
    if (clock.timer_remaining_ms) {
        clock.timer_remaining_ms -= clock.ms_per_interrupt;
        if (!clock.timer_remaining_ms)
            clock.timer_cb();
    }
    __bic_SR_register_on_exit(LPM0_bits);
}

int msp430_clock_enable(void)
{
    if (clock.enabled)
        return 0;

    /* Set ACLK to use REF0CLK.
     * Set SMCLK to use the DCO.
     */
    UCSCTL4 &= ~SELA_7;
    UCSCTL4 |= SELA_2;

    UCSCTL8 |= SMCLKREQEN | MCLKREQEN | ACLKREQEN;

    /* Enable interrupt for TBCCR0. */
    TBCCTL0 = CCIE;

    /* Number of ticks per millisecond. */
    clock.aclk = 32768;
    clock.ticks_per_interrupt = clock.aclk / 1000;
    TBCCR0 = clock.ticks_per_interrupt;

    /* Use ACLK, set timer to up-count mode, and start timer at zero. */
    TBCTL = TBSSEL_1 | MC_2 | TBCLR;

    clock.ms_per_interrupt = 1;
    clock.timer_remaining_ms += (clock.timer_remaining_ms %
        clock.ms_per_interrupt);
    clock.enabled = 1;

    /* Enable interrupts. */
    __bis_SR_register(GIE);

    return 0;
}

int msp430_clock_disable(void)
{
    if (!clock.enabled)
        return 0;

    /* Set ACLK to use VL0. */
    UCSCTL4 &= ~SELA_7;
    UCSCTL4 |= SELA_1;

    /* Previously, we would switch to VL0 and continue to increment the
     * millisecond clock. However, we don't need it anymore and can disable the
     * timer completely. VL0 isn't reliable at all anyway.
     */
    TBCTL &= ~MC_3;
    TBCCTL0 &= ~CCIE;
    TBCTL &= ~TBIFG;

    UCSCTL8 &= ~SMCLKREQEN & ~MCLKREQEN & ~ACLKREQEN;
    clock.enabled = 0;
    return 0;
}

int msp430_clock_init(unsigned long mclk, unsigned char xt)
{
    /* Enable XT pins. */
    if (xt & 0x01)
        P5SEL |= 0x30;
    if (xt & 0x02)
        P5SEL |= 0x0C;

    /* Select REF0 for FLL reference. */
    UCSCTL3 &= ~SELREF_7;
    UCSCTL3 |= SELREF_2;

    /* Set ACLK to use REF0CLK.
     * Set SMCLK to use the DCO.
     */
    UCSCTL4 &= ~SELA_7 & ~SELS_7 & ~SELM_7;
    UCSCTL4 |= SELA_2 | SELS_4 | SELM_4;

    /* Initialize FLL. */
    Init_FLL_Settle(mclk/1000L, mclk/32768);

    if (xt & 0x01)
        XT1_Start(XT1DRIVE_0);
    else
        UCSCTL6 &= ~XT1DRIVE_3;
    if (xt & 0x02)
        XT2_Start(XT2DRIVE_0);
    else
        UCSCTL6 &= ~XT2DRIVE_3;
    clock.mclk = mclk;
    clock.smclk = mclk;
    clock.aclk = 32768;

    /* Start the millisecond clock. */
    msp430_clock_enable();

    /* Start timestamp at zero. */
    clock.timestamp = 0;
    clock.timer_cb = NULL;
    clock.timer_remaining_ms = 0;
    return 0;
}

int msp430_get_mclk_freq(unsigned long *mclk)
{
    mclk[0] = clock.mclk;
    return 0;
}

int msp430_get_smclk_freq(unsigned long *smclk)
{
    smclk[0] = clock.smclk;
    return 0;
}

int msp430_get_aclk_freq(unsigned long *aclk)
{
    aclk[0] = clock.aclk;
    return 0;
}

int msp430_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = clock.timestamp;
    return 0;
}

int msp430_delay_ms(unsigned long num_ms)
{
    uint32_t start_time = clock.timestamp;
    if (!clock.enabled)
        return -1;
    while (clock.timestamp - start_time < num_ms)
        __bis_SR_register(LPM0_bits + GIE);
    return 0;
}

int msp430_slow_timer(unsigned char slow)
{
    if (slow) {
        clock.ticks_per_interrupt = 0xFFFF;
        TBCCR0 = clock.ticks_per_interrupt;
        /* Integer rounding happens here. If that's a problem, TBCCR0 can be
         * set to a multiple of clock.mclk / 1000 instead.
         */
        clock.ms_per_interrupt = 0xFFFF * 1000L / clock.aclk;
        clock.timer_remaining_ms += (clock.timer_remaining_ms %
            clock.ms_per_interrupt);
    } else {
        clock.ticks_per_interrupt = clock.aclk/1000;
        TBCCR0 = clock.ticks_per_interrupt;
        clock.ms_per_interrupt = 1;
    }
    return 0;
}
int msp430_register_timer_cb(void (*timer_cb)(void), unsigned long num_ms)
{
    if (!timer_cb || !num_ms) {
        clock.timer_cb = NULL;
        clock.timer_remaining_ms = 0;
        return 0;
    }

    /* Timer count needs to be evenly divisible by clock.ms_per_interrupt to
     * avoid overflow.
     */
    clock.timer_remaining_ms = num_ms + (clock.timer_remaining_ms %
        clock.ms_per_interrupt);
    clock.timer_cb = timer_cb;
    return 0;
}
