#include "timeTask.h"

#include <avr/io.h>         // AVR IO ports
#include <avr/interrupt.h>  // AVR Interrupts
#include <util/atomic.h>
#include <stdbool.h>


/**
 * An array carrying the last and current uptime in milliseconds as
 * uint16_t. The array needs to be indexed by #timeTask_idx.
 * Two uptimes are used due to concurrency reasons as the array is written
 * asynchronously in interrupt context.
 *
 * This array is internally used by the #TIMETASK macro.
 */
volatile uint16_t timeTask_uptime[] = { 0, 1 };

/**
 * Index in array #timeTask_uptime to find the current uptime in milliseconds.
 * timeTask_idx is updated in interrupt context, however the update is atomic
 * since it is an uint8_t.
 *
 * This variable is internally used by the #TIMETASK macro.
 */
volatile uint8_t timeTask_idx = 1;

// count uptime in milliseconds for use of execution time measurement
// this value is incremented by timer 5 compare match interrupt running at 1kHz
volatile uint16_t timeTask_time_ms = 0;

// storage for the captured start time with separate milliseconds and
// microseconds where range of microseconds is from 0 to 999
static uint16_t timeTask_start_time_ms = 0;
static uint16_t timeTask_start_time_us = 0;

// storage for the captured stop time with separate milliseconds and
// microseconds where range of microseconds is from 0 to 999
static uint16_t timeTask_stop_time_ms = 0;
static uint16_t timeTask_stop_time_us = 0;

// records if timeTask_init() has already been called in order to prevent
// multiple initializations
static bool timeTask_initialized = false;


// initialize timer 5 to fire interrupt every millisecond at an 8MHz clock
void timeTask_init(void) {
    // prevent multiple initializations
    if (! timeTask_initialized) {
        timeTask_initialized = true;

        // disable power reduction of timer 5
        PRR1 &= ~_BV(PRTIM5);

        TCCR5B = 0x00;  // stop timer 5 by selecting no clock

        // set counter value to zero
        TCNT5 = 0;

        // set timer 5 to CTC mode (clear timer on compare match with OCR5A)
        // use prescaler of 8 => timer frequency of 1MHz
        // TOP = max value of counter is defined by OCR5A
        // set OCR5A to 1000 = 0x03E8, produces a compare match interrupt frequency of 1kHz = 1ms
        OCR5A = 0x03E8;
        TCCR5C = 0x00; // no force output compare
        TCCR5A = 0x00; // WGM51=0, WGM50=0, COM5A0/1=0, COM5B0/1=0, COM5C0/1=0 (no compare output mode)
        TIMSK5 = _BV(OCIE5A); // timer 5 output compare A match interrupt enable
        TCCR5B = _BV(WGM52) | _BV(CS51); // CTC mode (TOP=OCR5A), Clock Select 1 => Prescaler 8, Timer frequency: 1MHz, start timer
    }
}


void timeTask_captureStartTime(void) {
    register uint16_t
        time_us,
        time_ms;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        time_us = TCNT5;
        time_ms = timeTask_time_ms;
    }

    timeTask_start_time_ms = time_ms;
    timeTask_start_time_us = time_us;
}


uint32_t timeTask_getStartTime(void) {
    return (uint32_t)timeTask_start_time_ms * 1000 + timeTask_start_time_us;
}


void timeTask_captureStopTime(void) {
    register uint16_t
        time_us,
        time_ms;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        time_us = TCNT5;
        time_ms = timeTask_time_ms;
    }

    timeTask_stop_time_ms = time_ms;
    timeTask_stop_time_us = time_us;
}


uint32_t timeTask_getStopTime(void) {
    return (uint32_t)timeTask_stop_time_ms * 1000 + timeTask_stop_time_us;
}


uint32_t timeTask_getDuration(void) {
    uint16_t
        diff_us,
        diff_ms;
    uint8_t
        neg_carry = 0;

    if (timeTask_stop_time_us >= timeTask_start_time_us)
        diff_us = timeTask_stop_time_us - timeTask_start_time_us;
    else {
        diff_us = 1000 - timeTask_start_time_us + timeTask_stop_time_us;
        neg_carry = 1;
    }

    // case start_time_ms > stop_time_ms is automatically covered by overflow
    diff_ms = timeTask_stop_time_ms - timeTask_start_time_ms - neg_carry;

    return (uint32_t)diff_ms * 1000 + diff_us;
}
