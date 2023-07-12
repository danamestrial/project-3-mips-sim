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

// Branchs
#define BNE 0b000101
#define BEQ 0b000100
#define BGTZ 0b000111
#define BLEZ 0b000110

// Immediate Instructions
#define ADDI 0b001000
#define ADDIU 0b001001
#define ANDI 0b001100
#define ORI 0b001101
#define XORI 0b001110
#define LUI 0b001111

// Load/Store Instructions
#define LHU 0b100101
#define SB 0b101000
#define SW 0b101011
#define LW 0b100011

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
#define BGEZ 0b00001

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
#define SRA 0b000011

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
#define sa(bits) (get_bits_between(bits, 6, 5))
#define imm(bits) (get_bits_between(bits, 0, 16))
#define base(bits) (get_bits_between(bits, 21, 5))
#define target(bits) (get_bits_between(bits, 0, 26))

uint32_t get_bits_between(uint32_t bits, int start, int size)
{
    return (bits >> start) & MASK(size);
}

uint32_t convert_to_32(int16_t immediate) {
    if ((immediate >> 15) == 1) {
        return (uint32_t) (immediate | 0xFFFF0000);
    }
    else {
        return (uint32_t) immediate;
    }
}

void process_special(uint32_t bits)
{
    uint8_t specialOpCode = (uint8_t) (bits & 0x3F);

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
        printf("SUB TO @%u, using @%u - @%u\n", rd(bits), rs(bits), rt(bits));
        printf("RESULT @%u = %d\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
        break;

    case SLT:
        break;

    case SLTU:
        break;

    case MULTU:
        break;

    case AND:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] & CURRENT_STATE.REGS[rt(bits)];
        printf("AND TO @%u, using @%u & @%u\n", rd(bits), rs(bits), rt(bits));
        printf("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
        break;

    case OR:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] | CURRENT_STATE.REGS[rt(bits)];
        printf("OR TO @%u, using @%u + @%u\n", rd(bits), rs(bits), rt(bits));
        printf("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
        break;

    case MULT:
        break;

    case MFHI:
        break;

    case JR:
        printf("JUMP %x\n\n", CURRENT_STATE.REGS[rs(bits)]);
        CURRENT_STATE.PC = CURRENT_STATE.REGS[rs(bits)] - 4;
        break;

    case JALR:
        break;

    case SYSCALL:
        RUN_BIT = 0;
        break;

    case SLL:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rt(bits)] << sa(bits);
        printf("SHIFT LEFT TO @%u, using @%u << %u\n", rd(bits), rt(bits), sa(bits));
        printf("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
        break;

    case SRA:
    case SRL:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rt(bits)] >> sa(bits);
        printf("SHIFT RIGHT TO @%u, using @%u >> %u\n", rd(bits), rt(bits), sa(bits));
        printf("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
        break;

    case MFLO:
        break;

    case MTHI:
        break;

    case NOR:
        break;

    case XOR:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] ^ CURRENT_STATE.REGS[rt(bits)];
        printf("XOR TO @%u, using @%u ^ @%u\n", rd(bits), rs(bits), rt(bits));
        printf("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
        break;

    case ADD:
    case ADDU:
        NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] + CURRENT_STATE.REGS[rt(bits)];
        printf("ADD TO @%u, using @%u + @%u\n", rd(bits), rs(bits), rt(bits));
        printf("RESULT @%u = %d\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
        break;

    default:
        break;
    }
}

void process_regimm(uint32_t bits)
{
    uint8_t regimmOpCode = (uint8_t) get_bits_between(bits, 16, 5);

    printf("Regimm\n");

    switch (regimmOpCode)
    {
    case BGEZ:
        printf("COMPARING %u >= 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
        if ((CURRENT_STATE.REGS[rs(bits)] >> 31) == 0)
        {
            CURRENT_STATE.PC = CURRENT_STATE.PC + convert_to_32(imm(bits << 2)) - 4;
        }
        break;

    case BLTZAL:
        printf("COMPARING %u < 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
        NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
        if ((CURRENT_STATE.REGS[rs(bits)] >> 31) != 0)
        {
            CURRENT_STATE.PC = CURRENT_STATE.PC + convert_to_32(imm(bits << 2)) - 4;
        }
        break;

    case BGEZAL:
        printf("COMPARING %u >= 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
        NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
        if ((CURRENT_STATE.REGS[rs(bits)] >> 31) == 0)
        {
            CURRENT_STATE.PC = CURRENT_STATE.PC + convert_to_32(imm(bits << 2)) - 4;
        }
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
    printf("%x\n", CURRENT_STATE.PC);
    uint8_t opcode = (uint8_t) (bits >> 26);

    // printf("Last 6 bits: %u\n", opcode);

    switch(opcode)
    {
        case J:
            CURRENT_STATE.PC = (target(bits) << 2) - 4;
            break;

        case JAL:
            NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
            CURRENT_STATE.PC = (target(bits) << 2) - 4;
            break;

        case ADDI:
        case ADDIU:
            NEXT_STATE.REGS[rt(bits)] = CURRENT_STATE.REGS[rs(bits)] + convert_to_32(imm(bits));
            printf("ADDI/U TO @%u, using @%u + %u\n", rt(bits), rs(bits), imm(bits));
            printf("RESULT @%u = %d\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
            break;

        case XORI:
            NEXT_STATE.REGS[rt(bits)] = CURRENT_STATE.REGS[rs(bits)] ^ imm(bits);
            printf("XORI TO @%u, using @%u ^ %u\n", rt(bits), rs(bits), imm(bits));
            printf("RESULT @%u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
            break;

        case ANDI:
            NEXT_STATE.REGS[rt(bits)] = CURRENT_STATE.REGS[rs(bits)] & imm(bits);
            printf("ANDI TO @%u, using @%u & %u\n", rt(bits), rs(bits), imm(bits));
            printf("RESULT @%u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
            break;

        case LUI:
            NEXT_STATE.REGS[rt(bits)] = imm(bits) << 16;
            printf("LUI TO @%u, using %u << 16\n", rt(bits), imm(bits));
            printf("RESULT @%u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
            break;

        case ORI:
            NEXT_STATE.REGS[rt(bits)] = imm(bits) | CURRENT_STATE.REGS[rs(bits)];
            printf("ORI TO @%u, using %u | %u\n", rt(bits), imm(bits), CURRENT_STATE.REGS[rs(bits)]);
            printf("RESULT @%u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
            break;

        case BNE:
            printf("COMPARING %u != %u\n\n", CURRENT_STATE.REGS[rs(bits)], CURRENT_STATE.REGS[rt(bits)]);
            if (CURRENT_STATE.REGS[rs(bits)] != CURRENT_STATE.REGS[rt(bits)])
            {
                CURRENT_STATE.PC = CURRENT_STATE.PC + convert_to_32(imm(bits) << 2) - 4;
            }
            break;

        case BEQ:
            printf("COMPARING %u == %u\n\n", CURRENT_STATE.REGS[rs(bits)], CURRENT_STATE.REGS[rt(bits)]);
            if (CURRENT_STATE.REGS[rs(bits)] == CURRENT_STATE.REGS[rt(bits)])
            {
                CURRENT_STATE.PC = CURRENT_STATE.PC + convert_to_32(imm(bits << 2)) - 4;
            }
            break;

        case BGTZ:
            printf("COMPARING %u > 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
            if ((CURRENT_STATE.REGS[rs(bits)] >> 31) == 0 && CURRENT_STATE.REGS[rs(bits)] != 0)
            {
                CURRENT_STATE.PC = CURRENT_STATE.PC + convert_to_32(imm(bits << 2)) - 4;
            }
            break;

        case BLEZ:
            printf("COMPARING %u < 0 or = 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
            if (((CURRENT_STATE.REGS[rs(bits)] >> 31) == 1) || (CURRENT_STATE.REGS[rs(bits)] == 0))
            {
                CURRENT_STATE.PC = CURRENT_STATE.PC + convert_to_32(imm(bits << 2)) - 4;
            }
            break;

        case LHU:
            break;

        case SB:
            break;

        case SW:
            mem_write_32(convert_to_32(imm(bits)) + base(bits), CURRENT_STATE.REGS[rt(bits)]);
            break;

        case LW:
            NEXT_STATE.REGS[rt(bits)] = (convert_to_32(imm(bits)));
            break;

        case REGIMM:
            process_regimm(bits);
            break;

        case SPECIAL:
            process_special(bits);
    }

    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
}
