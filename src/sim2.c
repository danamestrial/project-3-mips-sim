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
    u32 OP;
    u32 RSDATA;
    u32 RTDATA;
    u32 RD;
    u32 TARGET;
    u32 EXTENDEDIMM;
    u32 FUNCT;
    u32 PCPLUS4; // PC + 4
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

struct IDEX_PILELINE_REG IDEX_REG = {0, 0, 0, 0, 0, 0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

struct EXMEM_PIPELINE_REG
{
    // no extendImm, RSDATA/RTDATA, FUNCT
    u32 ALURESULT;
    u32 JUMPADDRESS;
    u32 RD;
    // no ALUOp & ALUSrc
    enum Signal RegDst;
    enum Signal Jump;
    enum Signal Branch;
    enum Signal BranchGate;
    enum Signal MemRead;
    enum Signal MemToReg;
    enum Signal MemWrite;
    enum Signal RegWrite;
};

struct EXMEM_PIPELINE_REG EXMEM_REG = {0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

struct MEMWB_PIPELINE_REG
{
    // no extendImm, RSDATA/RTDATA, FUNCT
    u32 ALURESULT;
    u32 JUMPADDRESS;
    u32 RD;
    // no ALUOp & ALUSrc & MemRead/Write
    enum Signal RegDst;
    enum Signal Jump;
    enum Signal Branch;
    enum Signal BranchGate;
    enum Signal MemToReg;
    enum Signal RegWrite;
};

struct MEMWB_PIPELINE_REG MEMWB_REG = {0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW};

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

u32 convert_to_32(int16_t immediate, int bitLength) {
    if ((immediate >> (bitLength - 1)) == 1) {
        return (u32) (immediate | (0xFFFFFFFF << bitLength));
    }
    else {
        return (u32) immediate;
    }
}

u32 instruction_memory(u32 pc)
{
    return mem_read_32(pc);
}

void writeback()
{
    u32 RD = MEMWB_REG.RD;
    u32 ALURESULT = MEMWB_REG.ALURESULT;

    if (MEMWB_REG.RegWrite == HIGH)
    {
        //write to register
        NEXT_STATE.REGS[RD] = ALURESULT;
        printf("writing: %u into %u\n",ALURESULT, RD);
    }

    if (MEMWB_REG.Jump == HIGH)
    {
        printf("jumped\n");
        NEXT_STATE.PC = MEMWB_REG.JUMPADDRESS;
    }
    else if (MEMWB_REG.BranchGate == HIGH)
    {
        printf("branched\n");
        NEXT_STATE.PC = MEMWB_REG.JUMPADDRESS;
    }
    else
    {
        NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    }


    // RESET MEMWB
    // // no extendImm, RSDATA/RTDATA, FUNCT
    // u32 ALURESULT;
    // u32 RD;
    // // no ALUOp & ALUSrc & MemRead/Write
    // enum Signal RegDst;
    // enum Signal Jump;
    // enum Signal Branch;
    // enum Signal MemToReg;
    // enum Signal RegWrite;
    MEMWB_REG.JUMPADDRESS = 0;
    MEMWB_REG.ALURESULT = 0;
    MEMWB_REG.RD = 0;
    MEMWB_REG.RegDst = LOW;
    MEMWB_REG.Jump = LOW;
    MEMWB_REG.Branch = LOW;
    MEMWB_REG.BranchGate = LOW;
    MEMWB_REG.MemToReg = LOW;
    MEMWB_REG.RegWrite = LOW;
}

void memory()
{
    if (EXMEM_REG.MemWrite == HIGH)
    {
        printf("No Mem\n");
    }
    else
    {
        printf("Memory down\n");
        MEMWB_REG.Jump = EXMEM_REG.Jump;
        MEMWB_REG.BranchGate = EXMEM_REG.BranchGate;
        MEMWB_REG.JUMPADDRESS = EXMEM_REG.JUMPADDRESS;
        MEMWB_REG.RD = EXMEM_REG.RD;
        MEMWB_REG.ALURESULT = EXMEM_REG.ALURESULT;
        MEMWB_REG.RegDst = EXMEM_REG.RegDst;
        MEMWB_REG.RegWrite = EXMEM_REG.RegWrite;
        MEMWB_REG.MemToReg = EXMEM_REG.MemToReg;
    }

    // Reset EXMEM
    EXMEM_REG.JUMPADDRESS = 0;
    EXMEM_REG.ALURESULT = 0;
    EXMEM_REG.RD = 0;
    EXMEM_REG.RegDst = LOW;
    EXMEM_REG.Jump = LOW;
    EXMEM_REG.Branch = LOW;
    EXMEM_REG.BranchGate = LOW;
    EXMEM_REG.MemRead = LOW;
    EXMEM_REG.MemToReg = LOW;
    EXMEM_REG.MemWrite = LOW;
    EXMEM_REG.RegWrite = LOW;
}

void execute()
{
    // Load Data from IDEX register
    u32 ALUDATA1 = IDEX_REG.RSDATA;
    // Simulate MUX if ALUSRC high then use extendImm if not RsData
    u32 ALUDATA2 = (IDEX_REG.ALUSrc == HIGH) ? IDEX_REG.EXTENDEDIMM : IDEX_REG.RTDATA;
    u32 op = IDEX_REG.OP;

    // if using alu (ALUOp=High) then find op from FUNCT
    if (IDEX_REG.ALUOp == HIGH)
    {
        if (op == SPECIAL) {
            switch(IDEX_REG.FUNCT)
            {
                case ADDU:
                    EXMEM_REG.ALURESULT = ALUDATA1 + ALUDATA2;
                    break;
                case SYSCALL:
                    break;
            }
        } else {
            switch (op)
            {
                case ADDIU:
                    EXMEM_REG.ALURESULT = ALUDATA1 + ALUDATA2;
                    break;
                case BEQ:
                    EXMEM_REG.JUMPADDRESS = IDEX_REG.PCPLUS4 + (IDEX_REG.EXTENDEDIMM << 2);
                    // CURRENT_STATE.REGS[rs(bits)] == CURRENT_STATE.REGS[rt(bits)]
                    // Built in GATE
                    EXMEM_REG.BranchGate = (ALUDATA1 == ALUDATA2) && (IDEX_REG.Branch == HIGH) ? HIGH : LOW;
                    break;
            }
        }
    }
    else
    {
        if (IDEX_REG.Jump == HIGH)
        {
            switch (op)
            {
            case J:
                EXMEM_REG.JUMPADDRESS = (IDEX_REG.TARGET << 2);
                break;
            case JR:
                printf("jump to %u\n", IDEX_REG.RSDATA);
                EXMEM_REG.JUMPADDRESS = IDEX_REG.RSDATA;
                break;
            }
        }
    }

    // enum Signal Jump;
    // enum Signal Branch;
    EXMEM_REG.RD = IDEX_REG.RD;
    EXMEM_REG.Jump = IDEX_REG.Jump;
    EXMEM_REG.RegDst = IDEX_REG.RegDst;
    EXMEM_REG.MemRead = IDEX_REG.MemRead;
    EXMEM_REG.MemToReg = IDEX_REG.MemToReg;
    EXMEM_REG.MemWrite = IDEX_REG.MemWrite;
    EXMEM_REG.RegWrite = IDEX_REG.RegWrite;

    // Reset ID/EX
    // u32 OP;
    // u32 RSDATA;
    // u32 RTDATA;
    // u32 RD;
    // u32 EXTENDEDIMM;
    // u32 FUNCT;
    // enum Signal RegDst;
    // enum Signal Jump;
    // enum Signal Branch;
    // enum Signal MemRead;
    // enum Signal MemToReg;
    // enum Signal ALUOp;
    // enum Signal MemWrite;
    // enum Signal ALUSrc;
    // enum Signal RegWrite;
    IDEX_REG.OP = 0;
    IDEX_REG.RSDATA = 0;
    IDEX_REG.RTDATA = 0;
    IDEX_REG.RD = 0;
    IDEX_REG.TARGET = 0;
    IDEX_REG.EXTENDEDIMM = 0;
    IDEX_REG.FUNCT = 0;
    IDEX_REG.PCPLUS4 = 0;
    IDEX_REG.RegDst = LOW;
    IDEX_REG.Jump = LOW;
    IDEX_REG.Branch = LOW;
    IDEX_REG.MemRead = LOW;
    IDEX_REG.MemToReg = LOW;
    IDEX_REG.ALUOp = LOW;
    IDEX_REG.MemWrite = LOW;
    IDEX_REG.ALUSrc = LOW;
    IDEX_REG.RegWrite = LOW;
}

void decode()
{
    // Load Data from IFID register
    u32 IR = IFID_REG.IR;
    u32 PCPLUS4 = IFID_REG.PCPLUS4;

    u32 op = op(IR);

    // If R-Type RegDst set to High, ALUOp to High, RegWrite to High
    // This simulate opcode into control unit
    uint8_t specialOpCode = (uint8_t) (IR & 0x3F);

    switch(op)
    {
        case J: // J-Type
            printf("Jump\n");
            CONTROL_UNIT.Jump = HIGH;
            IDEX_REG.TARGET = target(IR);
            break;
        case BEQ: // I-Type (but branch)
            CONTROL_UNIT.Branch = HIGH;
            CONTROL_UNIT.ALUOp = HIGH;
            IDEX_REG.EXTENDEDIMM = convert_to_32(imm(IR), 16);
            break;
        case ADDIU: // I-Type
            printf("I-Type\n");
            CONTROL_UNIT.RegDst = HIGH;
            CONTROL_UNIT.ALUOp = HIGH;
            CONTROL_UNIT.RegWrite = HIGH;
            CONTROL_UNIT.ALUSrc = HIGH;
            IDEX_REG.RD = rt(IR);
            printf("CHECK: %u\n", IDEX_REG.RD);
            IDEX_REG.EXTENDEDIMM = convert_to_32(imm(IR), 16);
            break;
        case SPECIAL: // R-Type
            switch(specialOpCode) // This op code needs RegWrite
            {
                case JR:
                    CONTROL_UNIT.Jump = HIGH;
                    break;
                case ADDU:
                    CONTROL_UNIT.RegDst = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    CONTROL_UNIT.RegWrite = HIGH;
                    IDEX_REG.RD = rd(IR);
                    IDEX_REG.FUNCT = funct(IR);
                    break;
                case SYSCALL:
                    RUN_BIT = 0;
                    break;
            }
    }

    // Stalling should be here
    // Pipe everything into IDEX register pipeline

    IDEX_REG.OP = op;
    IDEX_REG.RSDATA = CURRENT_STATE.REGS[rs(IR)];
    IDEX_REG.RTDATA = CURRENT_STATE.REGS[rt(IR)];
    IDEX_REG.Jump = CONTROL_UNIT.Jump;
    IDEX_REG.Branch = CONTROL_UNIT.Branch;
    IDEX_REG.RegDst = CONTROL_UNIT.RegDst;
    IDEX_REG.ALUOp = CONTROL_UNIT.ALUOp;
    IDEX_REG.RegWrite = CONTROL_UNIT.RegWrite;
    IDEX_REG.ALUSrc = CONTROL_UNIT.ALUSrc;

    // RESET IFID
    IFID_REG.IR = 0;
    IFID_REG.PCPLUS4 = 0;

    // RESET CONTROL UNIT
    CONTROL_UNIT.RegDst = LOW;
    CONTROL_UNIT.Jump = LOW;
    CONTROL_UNIT.Branch = LOW;
    CONTROL_UNIT.MemRead = LOW;
    CONTROL_UNIT.MemToReg = LOW;
    CONTROL_UNIT.ALUOp = LOW;
    CONTROL_UNIT.MemWrite = LOW;
    CONTROL_UNIT.ALUSrc = LOW;
    CONTROL_UNIT.RegWrite = LOW;
}

void fetch()
{
    printf("Entered Fetch\n");
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
    fetch();
    printf("f\n");

    decode();
    printf("d\n");

    execute();
    printf("e\n");

    memory();
    printf("m\b");

    writeback();
    printf("wb\n");

    /* execute one instruction here. You should use CURRENT_STATE and modify
     * values in NEXT_STATE. You can call mem_read_32() and mem_write_32() to
     * access memory. */
}