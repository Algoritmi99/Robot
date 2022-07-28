/**
 * @file adc.h
 * @ingroup io
 *
 * Library for interfacing with AD converter.
 *
 * The AD converter library supports a moving average filter for each sampled
 * channel. Configuration of the AD converter library is included in
 * src/cfg/io/adc/adc_cfg.h where size of the moving average filter
 * (#ADC_FILTER_SIZE), number of channels to be sampled (#ADC_CHANNEL_COUNT) and
 * specification of channels (#ADC_CHANNEL_INIT) is given.
 *
 * After calling ADC_init() the ADC is setup to sample all given channels
 * consecutively using the ADC complete interrupt. The ADC clock is set to a
 * frequency of 62.5kHz which is the lowest possible frequency resulting in the
 * least noise of sampled values. The first conversion takes 25 ADC clock cycles
 * (400us). All following conversions take 13 ADC clock cycles each (208us).
 *
 * Starting a conversion is triggered through timer 5 compare match ISR
 * <code>TIMER5_COMPA_vect</code> (see lib/tools/timeTask/timeTask_isr.S) every
 * two milliseconds. As soon as a conversion is finished (after 208us or 400us),
 * the ADC complete ISR <code>ADC_vect</code> (see lib/io/adc/adc_isr.S) is
 * executed which reads  the converted value and stores it in a data structure
 * which contains the sampled values for each channel and their average value.
 * The ISR then sets up the ADC for the conversion of the next channel which
 * will be triggered by run of <code>TIMER5_COMPA_vect</code> after the next.
 * So every two milliseconds, a single channel is sampled. This means that the
 * frequency of sampling each channel depends on the number of overall channels
 * to be sampled. Thus, each channel is sampled every #ADC_CHANNEL_COUNT * 2ms.
 *
 * The latest sampled raw value for a given channel can be requested with
 * ADC_getLastValue(). Note that the age of this value is in the
 * worst case about #ADC_CHANNEL_COUNT * 2ms.
 *
 * The filtered sample can be read with ADC_getFilteredValue().
 * This is a result of applying a moving average filter to the latest
 * #ADC_FILTER_SIZE samples. Note that the first meaningful value can be
 * obtained after the ADC has sampled a channel at least #ADC_FILTER_SIZE times.
 * Thus, measured from the time when ADC_init() was called and interrupts were
 * globally enabled, the first readout of the filtered value should occur no
 * earlier than (#ADC_FILTER_SIZE * #ADC_CHANNEL_COUNT * 2ms) + 2ms.
 *
 * Note that filtering introduces a time delay. The group delay of the moving
 * average filter is exactly (#ADC_FILTER_SIZE - 1) / 2 samples or
 * (#ADC_FILTER_SIZE - 1) / 2 * #ADC_CHANNEL_COUNT * 2ms.
 *
 * The JTAG interface is enabled by default up on a power cycle. When enabled,
 * the pins PF4 to PF7 (TCK, TMS, TDO, TDI) which also have the alternate
 * functions ADC4 to ADC7 can only be used for the JTAG interface. In this
 * situation all these pins are pulled high by the hardware and cannot be used
 * by the ADC.
 *
 * In order to allow their usage by the ADC, the JTAG can be switched off by
 * calling ADC_init() with the argument true. The JTD bit of the register MCUCR
 * is then used to turn off the JTAG interface.
 */

#ifndef ADC_H_
#define ADC_H_

// include user configuration of AD converter library
#include <cfg/io/adc/adc_cfg.h>

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>


/**
 * Initialize the AD converter library.
 *
 * Global interrupts must be enabled manually after calling this function.
 *
 * See src/cfg/io/adc/adc_cfg.h for configuration of the AD converter library.
 *
 * The parameter disableJTAG allows disabling the JTAG interface, so that pins
 * PF4 to PF7 (ADC4 to ADC7) can be used for AD conversions.
 *
 * Note that the JTAG interface is enabled by default up on a power cycle. When
 * enabled, the pins PF4 to PF7 (TCK, TMS, TDO, TDI) which also have the
 * alternate functions ADC4 to ADC7 can only be used for the JTAG interface.
 * In this situation all these pins are pulled high by the hardware.
 *
 * In order to allow their usage by the ADC, the JTAG needs to be switched off
 * using the JTD bit of the register MCUCR. A timed sequence is necessary in
 * which the new value of MCUCR must be written twice within four clock cycles.
 * Thus, interrupts will be disabled before and re-enabled afterwards if they
 * were enabled before.
 *
 * @param   disableJTAG  true to disable JTAG and allow pins ADC4 to ADC7 to be
 *                       used by the ADC, false to keep JTAG enabled
 */
void ADC_init(const bool disableJTAG);


/**
 * Read the filtered value from the AD converter for a given channel.
 *
 * For the configuration of the moving average filter, see the user
 * configuration in src/cfg/io/adc/adc_cfg.h.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 *
 * @param   channel   the virtual channel number (see #ADC_CHANNEL_INIT)
 * @return  the filtered digital value (10 bits)
 */
uint16_t ADC_getFilteredValue(const uint8_t channel);


/**
 * Read the latest raw value (unfiltered) from the AD converter for a given
 * channel.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 *
 * @param   channel   the virtual channel number (see #ADC_CHANNEL_INIT)
 * @return  the latest raw digital value (10 bits)
 */
uint16_t ADC_getLastValue(const uint8_t channel);


#endif /* ADC_H_ */
