/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_sysclock.h $
 *****************************************************************************/
/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
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
#ifndef _MSP430_CLOCK_H_
#define _MSP430_CLOCK_H_

#include <stdint.h>

/**
 *  @brief      Set the frequency of MCLK, SMCLK, and ACLK.
 *  @param[in]  mclk    Frequency of master clock.
 *  @param[in]  xt      1 if XT1 is present, 2 if XT2 is present, 0 otherwise.
 *  @return     0 if successful.
 */
int msp430_clock_init(unsigned long mclk, unsigned char xt);
/**
 *  @brief  Enable the millisecond timer.
 *  This function is automatically called by @e msp430_clock_init. It should be
 *  used to re-enable the timer after @e msp430_clock_disable is called.
 *  @return 0 if successful.
 */
int msp430_clock_enable(void);
/**
 *  @brief  Disable the millisecond timer.
 *  This function should be used prior to entering a low-power mode.
 *  @return 0 if successful.
 */
int msp430_clock_disable(void);
/**
 *  @brief      Get current clock count.
 *  Timer overflow will occur after 2^32 milliseconds.
 *  @param[out] count   Timer count in milliseconds.
 *  @return      0 if successful.
 */
int msp430_get_clock_ms(unsigned long *count);
/**
 *  @brief      Perform a blocking delay.
 *  @param[in]  num_ms  Number of milliseconds to delay.
 *  @return     0 if successful.
 */
int msp430_delay_ms(unsigned long num_ms);
/**
 *  @brief      Get frequency of SMCLK.
 *  Currently, the sub-master clock and the master clock are the same frequency.
 *  @param[out] smclk   SMCLK frequency.
 *  @return     0 if successful.
 */
int msp430_get_smclk_freq(unsigned long *smclk);
/**
 *  @brief      Slow down the timer.
 *  By default, a millisecond timer is used for timing/scheduling purposes.
 *  This API can be used to slow down the interval at which this clock is
 *  updated, saving power by reducing interrupts.
 *  @param[in]  slow    1 to slow down timer.
 *  @return     0 if successful.
 */
int msp430_slow_timer(unsigned char slow);
/**
 *  @brief      Register a one-time timer event.
 *  Only one event can be registered. If this function is called before the
 *  current timer ends, the new event will be registered and the current one
 *  will be discarded.
 *  @param[in]  timer_cb    Function called when timer is expired.
 *  @param[in]  num_ms      Number of milliseconds before function is called.
 *  @return     0 if successful.
 */
int msp430_register_timer_cb(void (*timer_cb)(void), unsigned long num_ms);
#endif  /* _MSP430_CLOCK_H_ */

/**
 * @}
 */
