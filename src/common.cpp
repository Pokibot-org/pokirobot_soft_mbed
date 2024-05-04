//
// Created by CopperBot on 04/05/2024.
//

#include "common.h"

FileHandle *mbed::mbed_override_console(int fd) {
    return &terminal;
}

// custom printf
void terminal_printf(const char *fmt, ...)
{
    static char terminal_buff[MAX_PRINTF_LENGTH];
    va_list args;
    va_start(args, fmt);
    int length = vsprintf(terminal_buff, fmt, args);
    va_end(args);
    terminal.write(terminal_buff, length);
}

void terminal_debug(const char *fmt, ...)
{
#if PRINTF_DEBUG_ENABLE
    static char terminal_buff[MAX_PRINTF_LENGTH];
    va_list args;
    va_start(args, fmt);
    int length = vsprintf(terminal_buff, fmt, args);
    va_end(args);
    terminal.write(terminal_buff, length);
#endif
}
