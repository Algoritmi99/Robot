#include "remoteDataProcessing.h"
#include <communication/communication.h>

#include <stdio.h>
#include <stdarg.h>


void remoteDataProcessing_command(const bool execute, const char* format, ...) {
    va_list argp;
    va_start(argp, format);

    char buff[514];
    buff[0] = execute ? 0x02 : 0x01;

    int size = vsnprintf(&buff[1], 513, format, argp);
    va_end(argp);

    communication_writePacket(CH_OUT_RDP, (uint8_t*)buff, size > 512 ? 513 : (size+1));
}


void remoteDataProcessing_clear(void) {
    char buff = 0x00;
    communication_writePacket(CH_OUT_RDP, (uint8_t*)&buff, 1);
}
