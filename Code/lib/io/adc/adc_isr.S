// include ADC configuration
#include <cfg/io/adc/adc_cfg.h>

#include <avr/io.h>         // AVR IO ports

.extern currentChannel;
.extern channels;


/*
ISR(ADC_vect) {
    // ADC conversion result is available

    uint16_t value = ADC; // read 10 bit ADC value

    // move position index in history array to next element
    uint8_t pos = (currentChannel->pos + 1) & (ADC_FILTER_SIZE - 1);
    currentChannel->pos = pos;

    // update sum of samples and history element
    currentChannel->sum = currentChannel->sum - currentChannel->history[pos] + value;
    currentChannel->history[pos] = value;

    // select next channel to convert
    if (++currentChannel == channels + ADC_CHANNEL_COUNT)
        currentChannel = &channels[0];

    // configure ADC to sample the required channel
    ADMUX = currentChannel->admux;
    ADCSRB = currentChannel->adcsrb;
}
*/


.global ADC_vect
ADC_vect:
    push r2
    in r2, _SFR_IO_ADDR(SREG)
    push r23
    push r24
    push r25
    push YL
    push YH
    push ZL
    push ZH

    // load currentChannel
    lds YL, currentChannel
    lds YH, currentChannel + 1
    // make a copy of currentChannel for later use
    movw ZL, YL

    // if (++currentChannel != channels + CHANNELCOUNT)
    subi ZL, lo8(-(8 + 2 * ADC_FILTER_SIZE))
    sbci ZH, hi8(-(8 + 2 * ADC_FILTER_SIZE))
    ldi r24, hi8(channels + ADC_CHANNEL_COUNT * (8 + 2 * ADC_FILTER_SIZE))
    cpi ZL, lo8(channels + ADC_CHANNEL_COUNT * (8 + 2 * ADC_FILTER_SIZE))
    cpc ZH, r24
    breq ADC_RESET_CURRENT_CHANNEL

ADC_SET_CHANNEL:
    // ADMUX  = currentChannel->admux;
    // ADMUX is set for the next conversion as early as possible to give the
    // voltage some time to stabilize
    ldd r24, Z+1
    sts _SFR_MEM_ADDR(ADMUX), r24

    // ADCSRB  = currentChannel->adcsrb;
    ldd r24, Z+2
    sts _SFR_MEM_ADDR(ADCSRB), r24

    sts currentChannel + 1, ZH
    sts currentChannel, ZL

    // Note: YH:YL contains "old" value of currentChannel to which result
    // of conversion corresponds to.

    // move position index in history array to next element
    // uint8_t pos = (currentChannel->pos + 1) & (ADC_FILTER_SIZE - 1);
    ld ZL, Y
    inc ZL
    andi ZL, ADC_FILTER_SIZE-1

    // currentChannel->pos = pos;
    st  Y, ZL

    // update sum of samples and history element
    // currentChannel->sum = currentChannel->sum - currentChannel->history[pos] + value;

    // calculate address of (currentChannel->history[pos]) - 8 in ZH:ZL
    eor ZH, ZH
    add ZL, ZL // ZH:ZL = pos*2
    adc ZH, ZH
    add ZL, YL // add address of currentChannel to ZH:ZL
    adc ZH, YH

    // interleaved execution of:
    //   load currentChannel->history[pos] (into r23,r24)
    //   uint16_t value = ADC; // read 10 bit ADC value (into r25, r25)
    //   currentChannel->history[pos] = value;
    //   subtract currentChannel->history[pos] - value
    ldd r23, Z+8
    lds r25, _SFR_MEM_ADDR(ADCL)
    std Z+8, r25
    sub r23, r25

    ldd r24, Z+9
    lds r25, _SFR_MEM_ADDR(ADCH)
    std Z+9, r25
    sbc r24, r25
    // result of subtraction as int16_t in r24:r23

    // convert int16_t to 24bit, result in r25:r24:r23
    mov r25, r24
    add r25, r25
    sbc r25, r25

    // interleaved execution of:
    //   load currentChannel->sum (24 bits only)
    //   subtract difference (history[pos]-value) from currentChannel->sum
    //   store updated value to currentChannel->sum
    ldd ZL, Y+4
    sub ZL, r23
    std Y+4, ZL

    ldd ZL, Y+5
    sbc ZL, r24
    std Y+5, ZL

    ldd ZL, Y+6
    sbc ZL, r25
    std Y+6, ZL

    // starting a conversion is done in TIMER5_COMPA_vect
    // (see tools/timeTask/timeTask_isr.S)

    pop ZH
    pop ZL
    pop YH
    pop YL
    pop r25
    pop r24
    pop r23
    out _SFR_IO_ADDR(SREG), r2
    pop r2
    reti

ADC_RESET_CURRENT_CHANNEL:
    // currentChannel = &channels[0];
    ldi ZL, lo8(channels)
    ldi ZH, hi8(channels)
    rjmp ADC_SET_CHANNEL
