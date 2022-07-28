// include ADC configuration
#include <cfg/io/adc/adc_cfg.h>

#include <avr/io.h>         // AVR IO ports

.extern timeTask_idx;
.extern timeTask_uptime;
.extern timeTask_time_ms;
.extern ADC_enabled;


/*
ISR(TIMER5_COMPA_vect) {
    // timer 5 has reached the TOP value defined by OCR5A, i.e. 1ms has passed
    ++timeTask_time_ms;
    uint8_t idx = (timeTask_idx + 1) & 1;
    timeTask_uptime[idx] += 2;
    timeTask_idx = idx;

    // if ADC is enabled, start a conversion every 2 milliseconds
    if (((timeTask_uptime[idx] % 2) == 0) && ADC_enabled) {
        // start ADC conversion of the next channel using the configuration described
        // by ADC_ADCSRA_CONFIG (see io/adc/adc.h for details)
        ADCSRA = ADC_ADCSRA_CONFIG;
    }
}
*/


.global TIMER5_COMPA_vect
TIMER5_COMPA_vect:
    push r2
    in r2, _SFR_IO_ADDR(SREG)
    push YL
    push YH
    push ZL
    push ZH

    // timer 5 has reached the TOP value defined by OCR5A, i.e. 1ms has passed

    // ++timeTask_time_ms;
    lds ZL, timeTask_time_ms
    lds ZH, timeTask_time_ms + 1
    // increment ZH:ZL by one
    adiw ZL, 1
    sts timeTask_time_ms + 1, ZH
    sts timeTask_time_ms, ZL

    // uint8_t idx = (timeTask_idx + 1) & 1;
    lds ZL, timeTask_idx
    inc ZL
    andi ZL, 0x01

    // timeTask_idx = idx;
    sts timeTask_idx, ZL

    // timeTask_uptime[idx] += 2;
    // load ZH:ZL with idx*2
    ldi ZH, 0x00
    add ZL, ZL
    // calculate address of timeTask_uptime[idx] in ZH:ZL
    subi ZL, lo8(-(timeTask_uptime))
    sbci ZH, hi8(-(timeTask_uptime))
    // load uint16_t at timeTask_uptime[idx]
    ld YL, Z
    ldd YH, Z+1
    // increment YH:YL by 2
    adiw YL, 2
    // store back
    std Z+1, YH
    st Z, YL

    // if updated value of timeTask_uptime[idx] is odd, jump to end of this ISR
    // otherwise, start a conversion, i.e. every 2 milliseconds
    sbrc YL, 0
    rjmp TIMER5_COMPA_FINISH

    // if ADC is disabled, jump to end of this ISR
    lds ZH, ADC_enabled
    cpi ZH, 0
    breq TIMER5_COMPA_FINISH

    // start ADC conversion of the next channel using the configuration described
    // by ADC_ADCSRA_CONFIG (see io/adc/adc.h for details)
    ldi ZL, ADC_ADCSRA_CONFIG
    sts _SFR_MEM_ADDR(ADCSRA), ZL

TIMER5_COMPA_FINISH:
    pop ZH
    pop ZL
    pop YH
    pop YL
    out _SFR_IO_ADDR(SREG), r2
    pop r2
    reti
