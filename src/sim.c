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

// Arithmetic Instructions
#define ADDI 0b001000
#define ADDIU 0b001001
#define ANDI 0b001100
#define ORI 0b001101
#define XORI 0b001110
#define LUI 0b001111

// Load/Store Instructions
#define LHU 0b100101
#define LH 0b100001
#define SB 0b101000
#define SW 0b101011
#define SH 0b101001
#define LW 0b100011
#define LBU 0b100100
#define LB 0b100000

// Set Instructions
#define SLTI 0b001010
#define SLTIU 0b001011

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
#define BLTZ 0b00000
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
#define ADD 0b100000
#define ADDU 0b100001
#define SUB 0b100010
#define SUBU 0b100011
#define SLT 0b101010
#define SLTU 0b101011
#define MULTU 0b011001

// Logical Instructions
#define AND 0b100100
#define OR 0b100101
#define NOR 0b100111
#define XOR 0b100110

// Multiply/Divide Instructions
#define MULT 0b011000
#define MFHI 0b010000
#define DIV 0b011010
#define DIVU 0b011011

// Jump Instructions
#define JR 0b001000
#define JALR 0b001001

// Shift Instructions
#define SLL 0b000000
#define SRL 0b000010
#define SRA 0b000011

// Move Instructions
#define MFLO 0b010010
#define MTHI 0b010001
#define MTLO 0b010011

// System Call
#define SYSCALL 0b001100





/////////////////////////
//                     //
//    HELPER MACROS    //
//                     //
/////////////////////////

typedef uint32_t u32;
typedef int32_t i32;

#define cast(T, V) ((T) V)
#define MASK(n) cast(u32, (~(0xFFFFFFFF << n)))

#define rs(bits) (get_bits_between(bits, 21, 5))
#define rt(bits) (get_bits_between(bits, 16, 5))
#define rd(bits) (get_bits_between(bits, 11, 5))
#define sa(bits) (get_bits_between(bits, 6, 5))
#define imm(bits) (get_bits_between(bits, 0, 16))
#define base(bits) (get_bits_between(bits, 21, 5))
#define target(bits) (get_bits_between(bits, 0, 26))

// #define DEBUG_FLAG
#ifdef DEBUG_FLAG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

////////////////////////////
//                        //
//    HELPER FUNCTIONS    //
//                        //
////////////////////////////

u32 get_bits_between(u32 bits, int start, int size)
{
    return (bits >> start) & MASK(size);
}

u32 convert_to_32(int16_t immediate, int bitLength) {
    if ((immediate >> (bitLength - 1)) == 1) {
        return (u32) (immediate | (0xFFFFFFFF << bitLength));
    }
    else {
        return (u32) immediate;
    }
}





////////////////////////////////////////
//                                    //
//    SPECIAL INSTRUCTION FUNCTIONS   //
//                                    //
////////////////////////////////////////

void process_SRLV(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = get_bits_between(CURRENT_STATE.REGS[rt(bits)] >> get_bits_between(rs(bits), 0, 5), 0, (32 - get_bits_between(rs(bits), 0, 5)));
}

void process_SRAV(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rt(bits)] >> get_bits_between(CURRENT_STATE.REGS[rs(bits)], 0, 5);
}

void process_SLLV(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rt(bits)] << get_bits_between(CURRENT_STATE.REGS[rs(bits)], 0, 5);
}

void process_SUB(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = (i32) CURRENT_STATE.REGS[rs(bits)] - (i32) CURRENT_STATE.REGS[rt(bits)];
    DEBUG_PRINT("SUB TO @%u, using @%u - @%u\n", rd(bits), rs(bits), rt(bits));
    DEBUG_PRINT("RESULT @%u = %d\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_SUBU(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] - CURRENT_STATE.REGS[rt(bits)];
    DEBUG_PRINT("SUB TO @%u, using @%u - @%u\n", rd(bits), rs(bits), rt(bits));
    DEBUG_PRINT("RESULT @%u = %d\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_SLT(u32 bits)
{
    if ((i32) CURRENT_STATE.REGS[rs(bits)] < (i32) CURRENT_STATE.REGS[rt(bits)]) {
        NEXT_STATE.REGS[rd(bits)] = 0;
    } else {
        NEXT_STATE.REGS[rd(bits)] = 1;
    }
}

void process_SLTU(u32 bits)
{
    if (CURRENT_STATE.REGS[rs(bits)] < CURRENT_STATE.REGS[rt(bits)]) {
        NEXT_STATE.REGS[rd(bits)] = 0;
    } else {
        NEXT_STATE.REGS[rd(bits)] = 1;
    }
}

void process_MULT(u32 bits)
{
    int64_t multiplied = ((i32) CURRENT_STATE.REGS[rs(bits)]) * ((i32) CURRENT_STATE.REGS[rt(bits)]);
    NEXT_STATE.LO = multiplied & 0xffffffff;
    NEXT_STATE.HI = (multiplied >> 32) & 0xffffffff;
}

void process_MULTU(u32 bits)
{
    uint64_t multiplied = CURRENT_STATE.REGS[rs(bits)] * CURRENT_STATE.REGS[rt(bits)];
    NEXT_STATE.LO = multiplied & 0xffffffff;
    NEXT_STATE.HI = (multiplied >> 32) & 0xffffffff;
}

void process_DIV(u32 bits)
{
    if (CURRENT_STATE.REGS[rt(bits)] != 0) {
        NEXT_STATE.LO = ((i32) CURRENT_STATE.REGS[rs(bits)]) / ((i32) CURRENT_STATE.REGS[rt(bits)]);
        NEXT_STATE.HI = ((i32) CURRENT_STATE.REGS[rs(bits)]) % ((i32) CURRENT_STATE.REGS[rt(bits)]);
    }
}

void process_DIVU(u32 bits)
{
    if (CURRENT_STATE.REGS[rt(bits)] != 0) {
        NEXT_STATE.LO = CURRENT_STATE.REGS[rs(bits)] / CURRENT_STATE.REGS[rt(bits)];
        NEXT_STATE.HI = CURRENT_STATE.REGS[rs(bits)] % CURRENT_STATE.REGS[rt(bits)];
    }
}

void process_MFHI(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.HI;
}

void process_MTLO(u32 bits)
{
    NEXT_STATE.LO = CURRENT_STATE.REGS[rs(bits)];
}

void process_AND(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] & CURRENT_STATE.REGS[rt(bits)];
    DEBUG_PRINT("AND TO @%u, using @%u & @%u\n", rd(bits), rs(bits), rt(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_OR(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] | CURRENT_STATE.REGS[rt(bits)];
    DEBUG_PRINT("OR TO @%u, using @%u + @%u\n", rd(bits), rs(bits), rt(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_JR(u32 bits)
{
    CURRENT_STATE.PC = CURRENT_STATE.REGS[rs(bits)] - 4;
    DEBUG_PRINT("JUMP %x\n\n", CURRENT_STATE.REGS[rs(bits)]);
}

void process_JALR(u32 bits)
{
    CURRENT_STATE.PC = CURRENT_STATE.REGS[rs(bits)] - 4;
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.PC + 4;
}

void process_SLL(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rt(bits)] << sa(bits);
    DEBUG_PRINT("SHIFT LEFT TO @%u, using @%u << %u\n", rd(bits), rt(bits), sa(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_SRA(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rt(bits)] >> sa(bits);
    DEBUG_PRINT("SHIFT RIGHT TO @%u, using @%u >> %u\n", rd(bits), rt(bits), sa(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_SRL(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = get_bits_between(CURRENT_STATE.REGS[rt(bits)] >> sa(bits), 0, 32-sa(bits));
    DEBUG_PRINT("SHIFT RIGHT TO @%u, using @%u >> %u\n", rd(bits), rt(bits), sa(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_MFLO(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.LO;
}

void process_MTHI(u32 bits)
{
    NEXT_STATE.HI = CURRENT_STATE.REGS[rs(bits)];
}

void process_NOR(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] ^ CURRENT_STATE.REGS[rt(bits)];
}

void process_XOR(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] ^ CURRENT_STATE.REGS[rt(bits)];
    DEBUG_PRINT("XOR TO @%u, using @%u ^ @%u\n", rd(bits), rs(bits), rt(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_ADD(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = (i32) CURRENT_STATE.REGS[rs(bits)] + (i32) CURRENT_STATE.REGS[rt(bits)];
    DEBUG_PRINT("ADD TO @%u, using @%u + @%u\n", rd(bits), rs(bits), rt(bits));
    DEBUG_PRINT("RESULT @%u = %d\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_ADDU(u32 bits)
{
    NEXT_STATE.REGS[rd(bits)] = CURRENT_STATE.REGS[rs(bits)] + CURRENT_STATE.REGS[rt(bits)];
    DEBUG_PRINT("ADD TO @%u, using @%u + @%u\n", rd(bits), rs(bits), rt(bits));
    DEBUG_PRINT("RESULT @%u = %d\n\n", rd(bits), NEXT_STATE.REGS[rd(bits)]);
}

void process_SYSCALL(u32 bits)
{
    if (CURRENT_STATE.REGS[2] == 10)
    {
        RUN_BIT = 0;
    }
}

/*

SPECIAL PROCESS SWITCH CASE STARTS HERE

*/

void process_special(u32 bits)
{
    uint8_t specialOpCode = (uint8_t) (bits & 0x3F);

    DEBUG_PRINT("SPECIAL\n");

    switch (specialOpCode)
    {
    case SRLV:
        process_SRLV(bits);
        break;

    case SRAV:
        process_SRAV(bits);
        break;

    case SLLV:
        process_SLLV(bits);
        break;

    case SUB:
        process_SUB(bits);
        break;

    case SUBU:
        process_SUBU(bits);
        break;

    case SLT:
        process_SLT(bits);
        break;

    case SLTU:
        process_SLTU(bits);
        break;

    case MULT:
        process_MULT(bits);
        break;

    case MULTU:
        process_MULTU(bits);
        break;

    case DIV:
        process_DIV(bits);
        break;

    case DIVU:
        process_DIVU(bits);
        break;

    case MFHI:
        process_MFHI(bits);
        break;

    case MTLO:
        process_MTLO(bits);
        break;

    case AND:
        process_AND(bits);
        break;

    case OR:
        process_OR(bits);
        break;

    case JR:
        process_JR(bits);
        break;

    case JALR:
        process_JALR(bits);
        break;

    case SLL:
        process_SLL(bits);
        break;

    case SRA:
        process_SRA(bits);

    case SRL:
        process_SRL(bits);
        break;

    case MFLO:
        process_MFLO(bits);
        break;

    case MTHI:
        process_MTHI(bits);
        break;

    case NOR:
        process_NOR(bits);
        break;

    case XOR:
        process_XOR(bits);
        break;

    case ADD:
        process_ADD(bits);
        break;

    case ADDU:
        process_ADDU(bits);
        break;

    case SYSCALL:
        process_SYSCALL(bits);
        break;
    }
}




///////////////////////////////////////
//                                   //
//    REGIMM INSTRUCTION FUNCTIONS   //
//                                   //
///////////////////////////////////////

void process_BGEZ(u32 bits) {
    DEBUG_PRINT("COMPARING %u >= 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
    if ((CURRENT_STATE.REGS[rs(bits)] >> 31) == 0) {
        CURRENT_STATE.PC = CURRENT_STATE.PC + ((convert_to_32(imm(bits), 16) << 2) - 4);
    }
}

void process_BGEZAL(u32 bits) {
    DEBUG_PRINT("COMPARING %u >= 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
    NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
    if ((CURRENT_STATE.REGS[rs(bits)] >> 31) == 0) {
        CURRENT_STATE.PC = CURRENT_STATE.PC + ((convert_to_32(imm(bits), 16) << 2) - 4);
    }
}

void process_BLTZ(u32 bits) {
    DEBUG_PRINT("COMPARING %u < 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
    if ((CURRENT_STATE.REGS[rs(bits)] >> 31) != 0) {
        CURRENT_STATE.PC = CURRENT_STATE.PC + ((convert_to_32(imm(bits), 16) << 2) - 4);
    }
}

void process_BLTZAL(u32 bits) {
    DEBUG_PRINT("COMPARING %u < 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
    NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
    if ((CURRENT_STATE.REGS[rs(bits)] >> 31) != 0) {
        CURRENT_STATE.PC = CURRENT_STATE.PC + ((convert_to_32(imm(bits), 16) << 2) - 4);
    }
}

/*

REGIMM INSTRUCTIONS SWITCH CASE HERE

*/

void process_regimm(u32 bits)
{
    uint8_t regimmOpCode = (uint8_t) get_bits_between(bits, 16, 5);

    DEBUG_PRINT("Regimm\n");

    switch (regimmOpCode)
    {
    case BGEZ:
        process_BGEZ(bits);
        break;

    case BGEZAL:
        process_BGEZAL(bits);
        break;

    case BLTZ:
        process_BLTZ(bits);
        break;

    case BLTZAL:
        process_BLTZAL(bits);
        break;

    default:
        break;
    }
}




///////////////////////////////////////
//                                   //
//    NORMAL INSTRUCTION FUNCTIONS   //
//                                   //
///////////////////////////////////////

void process_J(u32 bits)
{
    CURRENT_STATE.PC = ((target(bits)) << 2) - 4;
}

void process_JAL(u32 bits)
{
    NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
    CURRENT_STATE.PC = ((target(bits)) << 2) - 4;
}

void process_ADDI(u32 bits)
{
    NEXT_STATE.REGS[rt(bits)] = (i32) convert_to_32(imm(bits), 16) + (i32) CURRENT_STATE.REGS[rs(bits)];
}

void process_ADDIU(u32 bits)
{
    NEXT_STATE.REGS[rt(bits)] = convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[rs(bits)];

    DEBUG_PRINT("ADDI/U TO @%u, using @%u + %u\n", rt(bits), rs(bits), imm(bits));
    DEBUG_PRINT("RESULT @%u = %d\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
}

void process_XORI(u32 bits)
{
    NEXT_STATE.REGS[rt(bits)] = CURRENT_STATE.REGS[rs(bits)] ^ imm(bits);

    DEBUG_PRINT("XORI TO @%u, using @%u ^ %u\n", rt(bits), rs(bits), imm(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
}

void process_ANDI(u32 bits)
{
    NEXT_STATE.REGS[rt(bits)] = CURRENT_STATE.REGS[rs(bits)] & imm(bits);

    DEBUG_PRINT("ANDI TO @%u, using @%u & %u\n", rt(bits), rs(bits), imm(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
}

void process_LUI(u32 bits)
{
    NEXT_STATE.REGS[rt(bits)] = imm(bits) << 16;

    DEBUG_PRINT("LUI TO @%u, using %u << 16\n", rt(bits), imm(bits));
    DEBUG_PRINT("RESULT @%u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
}

void process_ORI(u32 bits)
{
    NEXT_STATE.REGS[rt(bits)] = imm(bits) | CURRENT_STATE.REGS[rs(bits)];

    DEBUG_PRINT("ORI TO @%u, using %u | %u\n", rt(bits), imm(bits), CURRENT_STATE.REGS[rs(bits)]);
    DEBUG_PRINT("RESULT @%u = %u\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
}

void process_BNE(u32 bits)
{
    if (CURRENT_STATE.REGS[rs(bits)] != CURRENT_STATE.REGS[rt(bits)])
    {
        CURRENT_STATE.PC = CURRENT_STATE.PC + ((convert_to_32(imm(bits), 16) << 2) - 4);
    }

    DEBUG_PRINT("COMPARING %u != %u\n\n", CURRENT_STATE.REGS[rs(bits)], CURRENT_STATE.REGS[rt(bits)]);
}

void process_BEQ(u32 bits)
{
    if (CURRENT_STATE.REGS[rs(bits)] == CURRENT_STATE.REGS[rt(bits)])
    {
        CURRENT_STATE.PC = CURRENT_STATE.PC + ((convert_to_32(imm(bits), 16) << 2) - 4);
    }

    DEBUG_PRINT("COMPARING %u == %u\n\n", CURRENT_STATE.REGS[rs(bits)], CURRENT_STATE.REGS[rt(bits)]);
}

void process_BGTZ(u32 bits)
{
    if ((CURRENT_STATE.REGS[rs(bits)] >> 31) == 0 && CURRENT_STATE.REGS[rs(bits)] != 0)
    {
        CURRENT_STATE.PC = CURRENT_STATE.PC + ((convert_to_32(imm(bits), 16) << 2) - 4);
    }

    DEBUG_PRINT("COMPARING %u > 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
}

void process_BLEZ(u32 bits)
{
    if (((CURRENT_STATE.REGS[rs(bits)] >> 31) == 1) || (CURRENT_STATE.REGS[rs(bits)] == 0))
    {
        CURRENT_STATE.PC = CURRENT_STATE.PC + ((convert_to_32(imm(bits), 16) << 2) - 4);
    }

    DEBUG_PRINT("COMPARING %u < 0 or = 0\n\n", CURRENT_STATE.REGS[rs(bits)]);
}

void process_LH(u32 bits)
{
    u32 virtualAddress = mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]);
    NEXT_STATE.REGS[rt(bits)] = convert_to_32(get_bits_between(virtualAddress, 0, 16), 16);

    DEBUG_PRINT("READING %d, to @%d\n\n", mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]), NEXT_STATE.REGS[rt(bits)]);
}

void process_LHU(u32 bits)
{
    u32 virtualAddress = mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]);
    NEXT_STATE.REGS[rt(bits)] = get_bits_between(virtualAddress, 0, 16);

    DEBUG_PRINT("READING %d, to @%d\n\n", mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]), NEXT_STATE.REGS[rt(bits)]);
}

void process_LB(u32 bits)
{
    u32 virtualAddress = mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]);
    NEXT_STATE.REGS[rt(bits)] = convert_to_32(get_bits_between(virtualAddress, 0, 8), 8);

    DEBUG_PRINT("READING %d, to @%d\n\n", mem_read_32(convert_to_32(imm(bits), 8) + CURRENT_STATE.REGS[base(bits)]), NEXT_STATE.REGS[rt(bits)]);
}

void process_LBU(u32 bits)
{
    u32 virtualAddress = mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]);
    NEXT_STATE.REGS[rt(bits)] = get_bits_between(virtualAddress, 0, 8);

    DEBUG_PRINT("READING %d, to @%d\n\n", mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]), NEXT_STATE.REGS[rt(bits)]);
}

void process_SB(u32 bits)
{
    mem_write_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)], get_bits_between(CURRENT_STATE.REGS[rt(bits)], 0, 8));

    DEBUG_PRINT("WRITING %d, at the Addr %d\n\n", get_bits_between(CURRENT_STATE.REGS[rt(bits)], 0, 8), convert_to_32(imm(bits), 16) + base(bits));
}

void process_SW(u32 bits)
{
    mem_write_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)], CURRENT_STATE.REGS[rt(bits)]);

    DEBUG_PRINT("WRITING %d, at the Addr %d\n", CURRENT_STATE.REGS[rt(bits)], convert_to_32(imm(bits), 16) + base(bits));
    DEBUG_PRINT("WROTE %d, to %d\n\n", mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]), convert_to_32(imm(bits), 16) + base(bits));
}

void process_SH(u32 bits)
{
    mem_write_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)], get_bits_between(CURRENT_STATE.REGS[rt(bits)], 0, 16));

    DEBUG_PRINT("WRITING %d, at the Addr %d\n\n", get_bits_between(CURRENT_STATE.REGS[rt(bits)], 0, 16), convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]);
}

void process_LW(u32 bits)
{
    NEXT_STATE.REGS[rt(bits)] = mem_read_32(convert_to_32(imm(bits), 16) + CURRENT_STATE.REGS[base(bits)]);

    DEBUG_PRINT("READING %d, to @%d\n", convert_to_32(imm(bits), 16) + base(bits), rt(bits));
    DEBUG_PRINT("CHECK @%d : %d\n\n", rt(bits), NEXT_STATE.REGS[rt(bits)]);
}

void process_SLTI(u32 bits)
{
    CURRENT_STATE.REGS[rs(bits)] -= (i32) convert_to_32(imm(bits), 16);
    if ((i32) CURRENT_STATE.REGS[rs(bits)] < (i32) convert_to_32(imm(bits), 16))
    {
        NEXT_STATE.REGS[rt(bits)] = 1;
    }
    else
    {
        NEXT_STATE.REGS[rt(bits)] = 0;
    }
}

void process_SLTIU(u32 bits)
{
    CURRENT_STATE.REGS[rs(bits)] -= convert_to_32(imm(bits), 16);
    if (CURRENT_STATE.REGS[rs(bits)] < convert_to_32(imm(bits), 16))
    {
        NEXT_STATE.REGS[rt(bits)] = 1;
    }
    else
    {
        NEXT_STATE.REGS[rt(bits)] = 0;
    }
}

/*

NORMAL INSTRUCTIONS SWITCH CASE HERE ( process_instruction() )

*/

void process_instruction()
{
    /* execute one instruction here. You should use CURRENT_STATE and modify
     * values in NEXT_STATE. You can call mem_read_32() and mem_write_32() to
     * access memory. */
    u32 bits = mem_read_32(CURRENT_STATE.PC);
    DEBUG_PRINT("%x\n", CURRENT_STATE.PC);

    uint8_t opcode = (uint8_t) (bits >> 26);
    // DEBUG_PRINT("Last 6 bits: %u\n", opcode);

    switch(opcode)
    {
        case J:
            process_J(bits);
            break;

        case JAL:
            process_JAL(bits);
            break;

        case ADDI:
            process_ADDI(bits);
            break;

        case ADDIU:
            process_ADDIU(bits);
            break;

        case XORI:
            process_XORI(bits);
            break;

        case ANDI:
            process_ANDI(bits);
            break;

        case LUI:
            process_LUI(bits);
            break;

        case ORI:
            process_ORI(bits);
            break;

        case BNE:
            process_BNE(bits);
            break;

        case BEQ:
            process_BEQ(bits);
            break;

        case BGTZ:
            process_BGTZ(bits);
            break;

        case BLEZ:
            process_BLEZ(bits);
            break;

        case LH:
            process_LH(bits);
            break;

        case LHU:
            process_LHU(bits);
            break;

        case LB:
            process_LB(bits);
            break;

        case LBU:
            process_LBU(bits);
            break;

        case SB:
            process_SB(bits);
            break;

        case SW:
            process_SW(bits);
            break;

        case SH:
            process_SH(bits);
            break;

        case LW:
            process_LW(bits);
            break;

        case SLTI:
            process_SLTI(bits);
            break;

        case SLTIU:
            process_SLTIU(bits);
            break;

        case REGIMM:
            process_regimm(bits);
            break;

        case SPECIAL:
            process_special(bits);
    }

    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
}
