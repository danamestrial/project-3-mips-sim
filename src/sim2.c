#include <stdio.h>
#include "shell.h"

///////////////////////////////
//                           //
//    SPECIAL INSTRUCTION    //
//                           //
/////////////////////////////// with RegWrite

// Shift Instructions
#define SRLV 0b000110
#define SRAV 0b000111
#define SLLV 0b000100

// Shift Instructions
#define SLL 0b000000
#define SRL 0b000010
#define SRA 0b000011

// Multiply/Divide Instructions
#define MULT 0b011000
#define MULTU 0b011001
#define MFHI 0b010000
#define DIV 0b011010
#define DIVU 0b011011

// Arithmetic Instructions
#define ADD 0b100000
#define ADDU 0b100001
#define SUB 0b100010
#define SUBU 0b100011

// Logical Instructions
#define AND 0b100100
#define OR 0b100101
#define XOR 0b100110
#define NOR 0b100111
#define SLT 0b101010
#define SLTU 0b101011

///////////////////////////////
//                           //
//    SPECIAL INSTRUCTION    //
//                           //
/////////////////////////////// w/o RegWrite

// Jump Instructions
#define JR 0b001000
#define JALR 0b001001

// Move Instructions
#define MFLO 0b010010
#define MTHI 0b010001
#define MTLO 0b010011

// System Call
#define SYSCALL 0b001100

#define cast(T, V) ((T) V)
#define MASK(n) cast(u32, (~(0xFFFFFFFF << n)))

// R-Type [op, rs, rt, rd, shamt, func]
// I-Type [op, rs, rt, imm]
// J-Type [op, target]
#define op(bits) (get_bits_between(bits, 26, 6))
#define rs(bits) (get_bits_between(bits, 21, 5))
#define rt(bits) (get_bits_between(bits, 16, 5))
#define imm(bits) (get_bits_between(bits, 0, 16))
#define target(bits) (get_bits_between(bits, 0, 26))
#define rd(bits) (get_bits_between(bits, 11, 5))
#define shamt(bits) (get_bits_between(bits, 6, 5))
#define funct(bits) (get_bits_between(bits, 0, 6))
// #define sa(bits) (get_bits_between(bits, 6, 5))
// #define base(bits) (get_bits_between(bits, 21, 5))

typedef uint32_t u32;
typedef int32_t i32;

typedef uint16_t u16;
typedef int16_t i16;

typedef uint8_t u8;
typedef int8_t i8;

typedef enum Signal Signal;

enum Signal
{
    LOW,
    HIGH
};

struct IFID_PIPELINE_REG
{
    u32 IR; // the instruction
    u32 PCPLUS4; // PC + 4
};

struct IFID_PIPELINE_REG IFID_REG = {0, 0};

struct IDEX_PILELINE_REG
{
    u32 RSDATA;
    u32 RTDATA;
    u32 RD;
    u32 EXTENDEDIMM;
    u32 FUNCT;
    enum Signal RegDst;
    enum Signal Jump;
    enum Signal Branch;
    enum Signal MemRead;
    enum Signal MemToReg;
    enum Signal ALUOp;
    enum Signal MemWrite;
    enum Signal ALUSrc;
    enum Signal RegWrite;
};

struct IDEX_PILELINE_REG IDEX_REG = {0, 0, 0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

struct CONTROL_UNIT
{
    enum Signal RegDst;
    enum Signal Jump;
    enum Signal Branch;
    enum Signal MemRead;
    enum Signal MemToReg;
    enum Signal ALUOp;
    enum Signal MemWrite;
    enum Signal ALUSrc;
    enum Signal RegWrite;
};

struct CONTROL_UNIT CONTROL_UNIT = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};


u32 get_bits_between(u32 bits, int start, int size)
{
    return (bits >> start) & MASK(size);
}

u32 instruction_memory(u32 pc)
{
    return mem_read_32(pc);
}

void decode()
{
    // Load Data from IFID register
    u32 IR = IFID_REG.IR;
    u32 PCPLUS4 = IFID_REG.PCPLUS4;

    u32 op = op(IR);

    // If R-Type RegDst set to High, ALUOp to High, RegWrite to High
    // This simulate opcode into control unit
    if (op)
    {
        uint8_t specialOpCode = (uint8_t) (IR & 0x3F);

        switch(specialOpCode) // This op code needs RegWrite
        {
            case ADDU:
                CONTROL_UNIT.RegDst = HIGH;
                CONTROL_UNIT.ALUOp = HIGH;
                CONTROL_UNIT.RegWrite = HIGH;
                break;
        }
    }

    // Stalling should be here
    // Pipe everything into IDEX register pipeline

    IDEX_REG.RSDATA = CURRENT_STATE.REGS[rs(IR)];
    IDEX_REG.RTDATA = CURRENT_STATE.REGS[rt(IR)];
    IDEX_REG.RD = rd(IR);
    IDEX_REG.FUNCT = funct(IR);
    IDEX_REG.RegDst = CONTROL_UNIT.RegDst;
    IDEX_REG.ALUOp = CONTROL_UNIT.ALUOp;
    IDEX_REG.RegWrite = CONTROL_UNIT.RegWrite;
}

void fetch()
{
    u32 IR = instruction_memory(CURRENT_STATE.PC);
    u32 PCPLUS4 = CURRENT_STATE.PC + 4;

    IFID_REG.IR = IR;
    IFID_REG.PCPLUS4 = PCPLUS4;
}

/*

NORMAL INSTRUCTIONS SWITCH CASE HERE ( process_instruction() )

*/

void process_instruction()
{
    /* execute one instruction here. You should use CURRENT_STATE and modify
     * values in NEXT_STATE. You can call mem_read_32() and mem_write_32() to
     * access memory. */

    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
}