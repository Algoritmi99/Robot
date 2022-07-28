#include "adc.h"
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>

#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>


// data structure for moving average filter (for a single channel)
typedef struct {
    volatile uint8_t pos;                       // index in history array with the latest sample
    uint8_t admux;                              // value for the ADMUX register for sampling this channel
    uint8_t adcsrb;                             // value for the ADCSRB register for sampling this channel
    uint8_t channel;                            // physical ADC channel, values: 0 => ADC0, 1 => ADC1, ..., 15 => ADC15
    volatile uint32_t sum;                      // sum of the latest HISTORYLENGTH samples
    volatile uint16_t history[ADC_FILTER_SIZE]; // storage for the latest ADC_FILTER_SIZE samples
} ADCChannel;

// moving average filter for all channels
ADCChannel channels[ADC_CHANNEL_COUNT];

// pointer into channels array with currently sampled channel
ADCChannel* currentChannel = &channels[0];

// Status of the ADC library. This value is used by TIMER5_COMPA_vect
// (see tools/timeTask/timeTask_isr.S) to check if an ADC conversion needs to
// be started.
bool ADC_enabled = false;


static inline void disable_JTAG(void) {
    // atomically disable the JTAG interface
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // read MCUCR and set bit for disabling JTAG
        uint8_t mcucr = MCUCR | _BV(JTD);

        // New value for MCUCR must be written twice within four clock cycles
        // in order to have JTAG disabled
        MCUCR = mcucr;
        MCUCR = mcucr;
    }
}


void ADC_init(const bool disableJTAG) {
    if (disableJTAG)
        disable_JTAG();

    // disable power reduction of ADC
    PRR0 &= ~_BV(PRADC);

    // disable ADC for now
    ADCSRA = 0x00;
    // switch off analog comparator
    ACSR |= _BV(ACD);

    #if (ADC_FILTER_SIZE > 256)
        #error ADC_FILTER_SIZE exceeds limit of 256 bytes
    #endif
    #if (ADC_FILTER_SIZE & (ADC_FILTER_SIZE - 1))
        #error ADC_FILTER_SIZE is not a power of 2
    #endif

    // initialize data structure
    memset((void*)(channels), 0, sizeof(channels));

    // set mapping between virtual and physical ADC channels in data structure
    ADC_CHANNEL_INIT

    // Configure channels[i].admux and channels[i].adcsrb for all channels.
    // Also disable digital input buffers for used ADC channels in order to save
    // some power, especially if analog voltage is close to Vcc/2
    uint8_t
        didr0 = 0,
        didr2 = 0;
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; ++i) {
        register uint8_t
            physicalChannel = channels[i].channel,
            physicalChannelAND0x07 = physicalChannel & 0x07;

        // voltage reference for ADC is AVCC pin (connected to 3.3V)
        channels[i].admux = _BV(REFS0) | physicalChannelAND0x07;

        if (physicalChannel & 0x08) {
            didr2 |= _BV(physicalChannelAND0x07);
            // if channel > 7 MUX5 (ADCSRB bit 3) has to be set, otherwise cleared
            channels[i].adcsrb = _BV(MUX5);
        } else {
            didr0 |= _BV(physicalChannelAND0x07);
            // note: channels[i].adcsrb = 0 per initialization
        }
    }
    DIDR0 = didr0;
    DIDR2 = didr2;

    // configure ADC to sample the first channel
    ADMUX = channels[0].admux;
    ADCSRB = channels[0].adcsrb;

    // ADC is now fully setup and the next run of TIMER5_COMPA_vect can start
    // the conversion
    ADC_enabled = true;

    // initialize timeTask since starting of ADC conversion is triggered
    // by compare match interrupt of timer 5
    timeTask_init();
}


uint16_t ADC_getFilteredValue(const uint8_t channel) {
    // perform atomic read on channels[channel].sum

    ADCChannel *c = &channels[channel];

    // Tell the compiler that we want to use variable c now, although we don't
    // do this. This memory barrier allows the placement of the code for
    // accessing channels[channel] before interrupts are disabled in the atomic
    // block. Otherwise the compiler would place this code after cli. This
    // ensures that interrupts are disabled for the smallest possible time.
    asm volatile ("" : : "r" (c) : "memory");

    uint32_t sum;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sum = c->sum;
    }

    // calculate average
    return (uint16_t)(sum / ADC_FILTER_SIZE);
}


uint16_t ADC_getLastValue(const uint8_t channel) {
    // perform atomic read on channels[channel].history[pos]
    ADCChannel *c = &channels[channel];

    // Tell the compiler that we want to use variable c now, although we don't
    // do this. This memory barrier allows the placement of the code for
    // accessing channels[channel] before interrupts are disabled in the atomic
    // block. Otherwise the compiler would place this code after cli. This
    // ensures that interrupts are disabled for the smallest possible time.
    asm volatile ("" : : "r" (c) : "memory");

    uint16_t value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = c->history[c->pos];
    }

    return value;
}
