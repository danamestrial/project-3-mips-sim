#include <stdio.h>
#include "shell.h"

static int i = 0;
void process_instruction()
{
    uint32_t something = mem_read_32(CURRENT_STATE.PC);
    /* execute one instruction here. You should use CURRENT_STATE and modify
     * values in NEXT_STATE. You can call mem_read_32() and mem_write_32() to
     * access memory. */

    uint8_t opcode = (uint8_t) (something >> 26);
    uint8_t special = (uint8_t) (something & 0x3F);

    printf("Last 6 bits: %u\n", opcode);
    printf("First 6 bits: %u\n\n", special);


    // Check for SysCall
    if (opcode == 0 && special == 12)
    {
        RUN_BIT = 0;
    }

    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
}
