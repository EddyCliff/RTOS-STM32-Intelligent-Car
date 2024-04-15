/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_interrupt.c $
 *****************************************************************************/
/**
 *  @defgroup MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_interrupt.c
 *      @brief      Supports common interrupt vectors using callbacks.
 *      @details    The following MSP430s are supported:
 *
 *                  MSP430F5528
 *                  MSP430F5529
 */

#include <stdio.h>
#include <string.h>
#include "msp430.h"
#include "msp430_interrupt.h"
#include "descriptors.h"
#include "USB_API/USB_Common/usb.h"
#if !defined __MSP430F5528__ && !defined __MSP430F5529__ && \
    !defined __MSP430F5229__
#error  The interrupt driver does not support this MSP430.
#endif

#define PINS_PER_PORT   (8)
struct msp430_int_s {
    void (*p1_cbs[PINS_PER_PORT])(void);
    void (*p2_cbs[PINS_PER_PORT])(void);
    unsigned char p1_exit[PINS_PER_PORT];
    unsigned char p2_exit[PINS_PER_PORT];
    /* Masks for each GPIO pin. */
    unsigned char p1_active_low;
    unsigned char p2_active_low;
};
static struct msp430_int_s msp_int = {0};

static inline unsigned char int_pin_to_index(unsigned short pin)
{
    /* Remove INT_PORT_Px from pin; index = pin/2 - 1. */
    return ((pin & 0x1F) >> 1) - 1;
}

/* Why does this even deserve its own function? */
static inline unsigned char index_to_bit(unsigned char index)
{
    return 1 << index;
}

#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
            UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG);
            SFRIFG1 &= ~OFIFG;
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            SYSBERRIV = 0;
#ifdef CONFIG_INTERFACE_USB
            USB_disable();
#endif
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void)
{
    unsigned char index, bit;
    index = int_pin_to_index(P1IV);
    bit = index_to_bit(index);

    if (msp_int.p1_cbs[index])
        msp_int.p1_cbs[index]();
    P1IFG &= ~bit;

    switch (msp_int.p1_exit[index]) {
    case INT_EXIT_NONE:
        break;
    case INT_EXIT_LPM0:
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case INT_EXIT_LPM1:
        __bic_SR_register_on_exit(LPM1_bits);
        break;
    case INT_EXIT_LPM2:
        __bic_SR_register_on_exit(LPM2_bits);
        break;
    case INT_EXIT_LPM3:
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case INT_EXIT_LPM4:
        __bic_SR_register_on_exit(LPM4_bits);
        break;
    default:
        break;
    }
}

#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void)
{
    unsigned char index, bit;
    index = int_pin_to_index(P2IV);
    bit = index_to_bit(index);

    if (msp_int.p2_cbs[index])
        msp_int.p2_cbs[index]();
    P2IFG &= ~bit;

    switch (msp_int.p2_exit[index]) {
    case INT_EXIT_NONE:
        break;
    case INT_EXIT_LPM0:
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case INT_EXIT_LPM1:
        __bic_SR_register_on_exit(LPM1_bits);
        break;
    case INT_EXIT_LPM2:
        __bic_SR_register_on_exit(LPM2_bits);
        break;
    case INT_EXIT_LPM3:
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case INT_EXIT_LPM4:
        __bic_SR_register_on_exit(LPM4_bits);
        break;
    default:
        break;
    }
}

int msp430_int_init(void)
{
    memset(&msp_int, 0, sizeof(msp_int));
    msp430_int_enable();
    return 0;
}

int msp430_int_enable(void)
{
    unsigned char index;

    P1IFG &= ~0xFF;
    P1IE &= ~0xFF;
    P2IFG &= ~0xFF;
    P2IE &= ~0xFF;

    for (index = 0; index < 8; index++) {
        unsigned char bit = index_to_bit(index);
        if (msp_int.p1_cbs[index]) {
            P1IFG |= bit;
            P1IE |= bit;
        }
        if (msp_int.p2_cbs[index]) {
            P2IFG |= bit;
            P2IE |= bit;
        }
    }

    return 0;
}

int msp430_int_disable(void)
{
    P1IFG &= ~0xFF;
    P1IE &= ~0xFF;
    P2IFG &= ~0xFF;
    P2IE &= ~0xFF;
    return 0;
}

int msp430_reg_int_cb(void (*cb)(void), unsigned short pin,
    unsigned char lp_exit, unsigned char active_low)
{
    unsigned char index, bit;

    index = int_pin_to_index(pin);
    bit = index_to_bit(index);

    if (pin & INT_PORT_P1) {
        msp_int.p1_cbs[index] = cb;
        if (active_low) {
            /* If interrupt enabled, use pullup resistor.
             * If interrupt disabled, output high.
             */
            P1OUT |= bit;
            /* Flag set on falling edge. */
            P1IES |= bit;
            msp_int.p1_active_low |= bit;
        } else {
            P1OUT &= ~bit;
            msp_int.p1_active_low &= ~bit;
        }

        if (!cb) {
            /* No callback registered. Disable this interrupt if necessary. */
            /* Set to output mode. */
            P1DIR |= bit;
            /* Disable pullup/pulldown resistor. */
            P1REN &= ~bit;
            /* Disable interrupt, clear flag. */
            P1IFG &= ~bit;
            P1IE &= ~bit;
            msp_int.p1_exit[index] = INT_EXIT_NONE;
        } else {
            /* Set to input mode. */
            P1DIR &= ~bit;
            /* Enable pullup/pulldown resistor. */
            P1REN |= bit;
            /* Enable interrupt, clear flag. */
            P1IFG &= ~bit;
            P1IE |= bit;
            msp_int.p1_exit[index] = lp_exit;
        }
    } else if (pin & INT_PORT_P2) {
        msp_int.p2_cbs[index] = cb;
        if (active_low) {
            /* If interrupt enabled, use pullup resistor.
             * If interrupt disabled, output high.
             */
            P2OUT |= bit;
            /* Flag set on falling edge. */
            P2IES |= bit;
            msp_int.p2_active_low |= bit;
        } else {
            P2OUT &= ~bit;
            msp_int.p2_active_low &= ~bit;
        }

        if (!cb) {
            /* No callback registered. Disable this interrupt if necessary. */
            /* Set to output mode. */
            P2DIR |= bit;
            /* Disable pullup/pulldown resistor. */
            P2REN &= ~bit;
            /* Disable interrupt, clear flag. */
            P2IFG &= ~bit;
            P2IE &= ~bit;
            msp_int.p2_exit[index] = INT_EXIT_NONE;
        } else {
            /* Set to input mode. */
            P2DIR &= ~bit;
            /* Enable pullup/pulldown resistor. */
            P2REN |= bit;
            /* Enable interrupt, clear flag. */
            P2IFG &= ~bit;
            P2IE |= bit;
            msp_int.p2_exit[index] = lp_exit;
        }
    }
    return 0;
}
