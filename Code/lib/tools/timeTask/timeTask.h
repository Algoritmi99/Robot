/**
 * @file timeTask.h
 * @ingroup tools
 *
 * Functions for timed execution of code and measurement of execution time.
 *
 * Timer 5 is used for measuring the uptime. Its output compare match ISR
 * <code>TIMER5_COMPA_vect</code> is fired every millisecond.
 *
 * Note that the ISR <code>TIMER5_COMPA_vect</code> also has the responsibility
 * of starting AD conversions if the ADC library has been initialized. See
 * io/adc/adc.h for further details.
 */

#ifndef TIMETASK_H_
#define TIMETASK_H_

#include <stdint.h>


/**
 * Execution of body block every 'cycles' loop cycles.
 * Needs NO initialization via timeTask_init() and hence no timer.
 *
 * <b>Usage:</b>
 * @code
 * for(;;) { // main loop
 *
 *     CYCLETASK(T1, 1000) { // execute every 1000 loop cycles
 *         // do some stuff
 *         static int16_t j = 0;
 *         j++;
 *         communication_log(LEVEL_INFO, "T1: %d\n", j);
 *      }
 *
 *      CYCLETASK(T2, 333) { // execute every 333 loop cycles
 *          // do some other stuff
 *          static int16_t j = 0;
 *          j++;
 *          communication_log(LEVEL_INFO, "T2: %d\n", j);
 *      }
 *  }
 *  @endcode
 */
#define CYCLETASK(name, cycles) static uint32_t name = 0; if (! (name = (name > cycles) ? 0 : (name + 1)))


/**
 * Initialization for the #TIMETASK macro and for execution time measurement.
 * Uses timer 5 to fire output compare match interrupt every millisecond.
 * Global interrupts must be enabled manually after calling this function.
 *
 * Note that timeTask_init() is also called by ADC_init() to ensure that AD
 * conversions are started by <code>TIMER5_COMPA_vect</code>.
 */
void timeTask_init(void);


/**
 * Execution of body block with a minimum delay of 'interval_ms' milliseconds in
 * between.
 * Requires initialization via timeTask_init(). The function is robust concerning
 * overflow of uptime variable, since comparison with interval_ms is done as
 * unsigned using the full range of the variable.
 * Note that the actual interval may be larger and varying due to execution time of
 * other code in main loop as well as frequency and duration of interrupt service
 * routines.
 *
 * The maximum interval is 65535ms!
 *
 * <b>Usage:</b>
 * @code
 * for(;;) { // main loop
 *
 *     TIMETASK(T1, 100) { // execute block approximately every 100ms
 *         // do some stuff
 *         static int16_t j = 0;
 *         j++;
 *         communication_log(LEVEL_INFO, "T1: %d\n", j);
 *      }
 *
 *      TIMETASK(T2, 200) { // execute block approximately every 200ms
 *          // do some other stuff
 *          static int16_t j = 0;
 *          j++;
 *          communication_log(LEVEL_INFO, "T2: %d\n", j);
 *      }
 *  }
 *  @endcode
 */
#define TIMETASK(name, interval_ms)                                      \
    static uint16_t name = 0;                                            \
    register uint16_t tt_uptime_##name = timeTask_getUptime();           \
    register uint8_t tt_ex_##name = 0;                                   \
    if ((tt_uptime_##name - name) >= (uint16_t)interval_ms) {            \
        name = tt_uptime_##name;                                         \
        tt_ex_##name = 1;                                                \
    }                                                                    \
    if (tt_ex_##name)


/**
 * Get current uptime in milliseconds.
 * This function is internally used by the #TIMETASK macro when checking
 * if the time task needs to be executed.
 *
 * @return  current uptime in milliseconds
 */
static inline uint16_t __attribute__((always_inline)) timeTask_getUptime(void) {
	extern volatile uint16_t timeTask_uptime[2];
	extern volatile uint8_t timeTask_idx;
	return timeTask_uptime[timeTask_idx];
}

/**
 * Capture start time for execution time measurement.
 *
 * This functions captures the time with a resolution of 1 microsecond and marks
 * the beginning of an execution time measurement. The maximum duration which
 * can be captured is 65535999 microseconds (about 65.5 seconds).
 *
 * Use this function in conjunction with timeTask_captureStopTime() and
 * timeTask_getDuration();
 */
void timeTask_captureStartTime(void);


/**
 * Get start time which was captured by timeTask_captureStartTime().
 *
 * @return  captured start time in microseconds (max. value is 65535999)
 */
uint32_t timeTask_getStartTime(void);


/**
 * Capture stop time for execution time measurement.
 *
 * This functions captures the time with a resolution of 1 microsecond and marks
 * the end of an execution time measurement. The maximum duration which
 * can be captured is 65535999 microseconds (about 65.5 seconds).
 *
 * Use this function in conjunction with timeTask_captureStartTime() and
 * timeTask_getDuration();
 */
void timeTask_captureStopTime(void);


/**
 * Get stop time which was captured by timeTask_captureStopTime().
 *
 * @return  captured stop time in microseconds (max. value is 65535999)
 */
uint32_t timeTask_getStopTime(void);


/**
 * Calculate the duration between captured start and stop time.
 *
 * @return  duration in microseconds (max. value is 65535999)
 */
uint32_t timeTask_getDuration(void);



#endif /* TIMETASK_H_ */
