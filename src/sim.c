#include <stdio.h>
#include "shell.h"

//////////////////////////////
//                          //
//    NORMAL INSTRUCTION    //
//                          //
//////////////////////////////

// Jumps
#define J 0b000010
#define JAL 0b000011

// Immediate Instructions
#define ADDI 0b001000
#define ADDIU 0b001001
#define XORI 0b001110
#define LUI 0b001111

// Load/Store Instructions
#define LHU 0b100101
#define SB 0b101000

// SPECIAL
#define SPECIAL 0b000000

// REGIMM
#define REGIMM 0b000001

//////////////////////////////
//                          //
//    REGIMM INSTRUCTION    //
//                          //
//////////////////////////////

// Branch Instructions
#define BLTZAL 0b10000
#define BGEZAL 0b10001

///////////////////////////////
//                           //
//    SPECIAL INSTRUCTION    //
//                           //
///////////////////////////////

// Shift Instructions
#define SRLV 0b000110
#define SRAV 0b000111
#define SLLV 0b000100

// Arithmetic Instructions
#define SUB 0b100010
#define SUBU 0b100011
#define SLT 0b101010
#define SLTU 0b101011
#define MULTU 0b011001

// Logical Instructions
#define AND 0b100100
#define OR 0b100101

// Multiply/Divide Instructions
#define MULT 0b011000
#define MFHI 0b010000

// Jump Instructions
#define JR 0b001000
#define JALR 0b001001

// System Call
#define SYSCALL 0b001100

// Shift Instructions
#define SLL 0b000000
#define SRL 0b000010

// Miscellaneous Instructions
#define MFLO 0b010010
#define MTHI 0b010001
#define NOR 0b100111
#define XOR 0b100110
#define ADD 0b100000
#define ADDU 0b100001

#define cast(T, V) ((T) V)
#define MASK(n) cast(uint32_t, (~(0xFFFFFFFF << n)))

#define rs(bits) (get_bits_between(bits, 21, 5))
#define rt(bits) (get_bits_between(bits, 16, 5))
#define rd(bits) (get_bits_between(bits, 11, 5))
#define imm(bits) (get_bits_between(bits, 0, 16))
#define sa(bits) (get_bits_between(bits, 6, 5))

uint32_t get_bits_between(uint32_t bits, int start, int size)
{
    return (bits >> start) & MASK(size);
}

void process_special(uint32_t bits)
{
    uint8_t specialOpCode = (uint8_t) (bits & 0x3F);
    uint8_t rs;
    uint8_t rt;
    uint8_t rd;
    uint8_t sa;

    printf("SPECIAL\n");

    switch (specialOpCode)
    {
    case SRLV:
        break;

    case SRAV:
        break;

    case SLLV:
        break;

    case SUB:
    case SUBU:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] - CURRENT_STATE.REGS[rt(bits)];
        break;

    case SLT:
        break;

    case SLTU:
        break;

    case MULTU:
        break;

    case AND:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] & CURRENT_STATE.REGS[rt(bits)];
        break;

    case OR:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] | CURRENT_STATE.REGS[rt(bits)];
        break;

    case MULT:
        break;

    case MFHI:
        break;

    case JR:
        break;

    case JALR:
        break;

    case SYSCALL:
        RUN_BIT = 0;
        break;

    case SLL:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rt(bits)] << sa(bits);
        break;

    case SRL:
        break;

    case MFLO:
        break;

    case MTHI:
        break;

    case NOR:
        break;

    case XOR:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] ^ CURRENT_STATE.REGS[rt(bits)];
        break;

    case ADD:
        break;

    case ADDU:
        break;

    default:
        break;
    }
}

void process_instruction()
{
    uint32_t bits = mem_read_32(CURRENT_STATE.PC);
    /* execute one instruction here. You should use CURRENT_STATE and modify
     * values in NEXT_STATE. You can call mem_read_32() and mem_write_32() to
     * access memory. */

    uint8_t opcode = (uint8_t) (bits >> 26);

    printf("Last 6 bits: %u\n", opcode);

    switch(opcode)
    {
        case J:
            break;
        case JAL:
            break;

        case ADDI:
        case ADDIU:
            printf("ADD TO %u, using %u + %u\n", rt(bits), rs(bits), imm(bits));
            NEXT_STATE.REGS[rt(bits)] = CURRENT_STATE.REGS[rs(bits)] + imm(bits);
            printf("RESULT AT %u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
            break;

        case XORI:
            NEXT_STATE.REGS[rt(bits)] = CURRENT_STATE.REGS[rs(bits)] ^ imm(bits);
            break;
        case LUI:
            break;
        case LHU:
            break;
        case SB:
            break;
        case REGIMM:
            break;
        case SPECIAL:
            process_special(bits);
    }

    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
}
