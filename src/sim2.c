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

void reset_IFID_pipeline()
{
    IFID_REG.IR = 0;
    IFID_REG.PCPLUS4 = 0;
}

struct IDEX_PILELINE_REG
{
    u32 OP;
    u32 RSDATA;
    u32 RS;
    u32 RTDATA;
    u32 RT;
    u32 SA;
    u32 RD;
    u32 TARGET;
    u32 EXTENDEDIMM;
    u32 FUNCT;
    u32 PCPLUS4; // PC + 4
    u32 HI;
    u32 LO;
    enum Signal RegDst;
    enum Signal Jump;
    enum Signal Branch;
    enum Signal MemRead;
    enum Signal MemToReg;
    enum Signal ALUOp;
    enum Signal MemWrite;
    enum Signal ALUSrc;
    enum Signal RegWrite;
    enum Signal SpecialRegHi;
    enum Signal SpecialRegLo;
};

struct IDEX_PILELINE_REG IDEX_REG = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

void reset_IDEX_pipeline()
{
    IDEX_REG.OP = 0;
    IDEX_REG.RSDATA = 0;
    IDEX_REG.RS = 0;
    IDEX_REG.RTDATA = 0;
    IDEX_REG.RT = 0;
    IDEX_REG.SA = 0;
    IDEX_REG.RD = 0;
    IDEX_REG.TARGET = 0;
    IDEX_REG.EXTENDEDIMM = 0;
    IDEX_REG.FUNCT = 0;
    IDEX_REG.PCPLUS4 = 0;
    IDEX_REG.HI = 0;
    IDEX_REG.LO = 0;
    IDEX_REG.RegDst = LOW;
    IDEX_REG.Jump = LOW;
    IDEX_REG.Branch = LOW;
    IDEX_REG.MemRead = LOW;
    IDEX_REG.MemToReg = LOW;
    IDEX_REG.ALUOp = LOW;
    IDEX_REG.MemWrite = LOW;
    IDEX_REG.ALUSrc = LOW;
    IDEX_REG.RegWrite = LOW;
    IDEX_REG.SpecialRegHi = LOW;
    IDEX_REG.SpecialRegLo = LOW;
}

struct EXMEM_PIPELINE_REG
{
    // no extendImm, RSDATA/RTDATA, FUNCT
    u32 ALURESULT;
    u32 ALURESULT2;
    u32 JUMPADDRESS;
    u32 EXTENDEDIMM;
    u32 RSDATA;
    u32 RTDATA;
    u32 OP;
    u32 RD;
    // no ALUOp & ALUSrc
    enum Signal RegDst;
    enum Signal Jump;
    enum Signal BranchGate;
    enum Signal MemRead;
    enum Signal MemToReg;
    enum Signal MemWrite;
    enum Signal RegWrite;
    enum Signal SpecialRegHi;
    enum Signal SpecialRegLo;
};

struct EXMEM_PIPELINE_REG EXMEM_REG = {0, 0, 0, 0, 0, 0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

void reset_EXMEM_pipeline()
{
    EXMEM_REG.JUMPADDRESS = 0;
    EXMEM_REG.ALURESULT = 0;
    EXMEM_REG.ALURESULT2 = 0;
    EXMEM_REG.EXTENDEDIMM = 0;
    EXMEM_REG.OP = 0;
    EXMEM_REG.RSDATA = 0;
    EXMEM_REG.RTDATA = 0;
    EXMEM_REG.RD = 0;
    EXMEM_REG.RegDst = LOW;
    EXMEM_REG.Jump = LOW;
    EXMEM_REG.BranchGate = LOW;
    EXMEM_REG.MemRead = LOW;
    EXMEM_REG.MemToReg = LOW;
    EXMEM_REG.MemWrite = LOW;
    EXMEM_REG.RegWrite = LOW;
    EXMEM_REG.SpecialRegHi = LOW;
    EXMEM_REG.SpecialRegLo = LOW;
}

struct MEMWB_PIPELINE_REG
{
    // no extendImm, RSDATA/RTDATA, FUNCT
    u32 ALURESULT;
    u32 ALURESULT2;
    u32 JUMPADDRESS;
    u32 RD;
    // no ALUOp & ALUSrc & MemRead/Write
    enum Signal RegDst;
    enum Signal Jump;
    enum Signal BranchGate;
    enum Signal MemToReg;
    enum Signal RegWrite;
    enum Signal SpecialRegHi;
    enum Signal SpecialRegLo;
};

struct MEMWB_PIPELINE_REG MEMWB_REG = {0, 0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

void reset_MEMWB_pipeline()
{
    MEMWB_REG.JUMPADDRESS = 0;
    MEMWB_REG.ALURESULT = 0;
    MEMWB_REG.ALURESULT2 = 0;
    MEMWB_REG.RD = 0;
    MEMWB_REG.RegDst = LOW;
    MEMWB_REG.Jump = LOW;
    MEMWB_REG.BranchGate = LOW;
    MEMWB_REG.MemToReg = LOW;
    MEMWB_REG.RegWrite = LOW;
    MEMWB_REG.SpecialRegHi = LOW;
    MEMWB_REG.SpecialRegLo = LOW;
}

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

void reset_control_unit()
{
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

struct STATE
{
    enum Signal Fetch;
    enum Signal Decode;
};

struct STATE STATE = {LOW, LOW};

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

/////////////////////////////
//                         //
//    PIPELINE FUNCTION    //
//                         //
/////////////////////////////

void pipe_to_IDEX()
{
    IDEX_REG.OP = op(IFID_REG.IR);
    IDEX_REG.PCPLUS4 = IFID_REG.PCPLUS4;
    IDEX_REG.RSDATA = CURRENT_STATE.REGS[rs(IFID_REG.IR)];
    IDEX_REG.RS = rs(IFID_REG.IR);
    IDEX_REG.RTDATA = CURRENT_STATE.REGS[rt(IFID_REG.IR)];
    IDEX_REG.RT = rt(IFID_REG.IR);
    IDEX_REG.SA = shamt(IFID_REG.IR);
    IDEX_REG.HI = CURRENT_STATE.HI;
    IDEX_REG.LO = CURRENT_STATE.LO;
    IDEX_REG.Jump = CONTROL_UNIT.Jump;
    IDEX_REG.Branch = CONTROL_UNIT.Branch;
    IDEX_REG.RegDst = CONTROL_UNIT.RegDst;
    IDEX_REG.ALUOp = CONTROL_UNIT.ALUOp;
    IDEX_REG.RegWrite = CONTROL_UNIT.RegWrite;
    IDEX_REG.ALUSrc = CONTROL_UNIT.ALUSrc;
}

void pipe_to_EXMEM()
{
    EXMEM_REG.RSDATA = IDEX_REG.RSDATA;
    EXMEM_REG.RTDATA = IDEX_REG.RTDATA;
    EXMEM_REG.OP = IDEX_REG.OP;
    EXMEM_REG.RD = IDEX_REG.RD;
    EXMEM_REG.EXTENDEDIMM = IDEX_REG.EXTENDEDIMM;
    EXMEM_REG.Jump = IDEX_REG.Jump;
    EXMEM_REG.RegDst = IDEX_REG.RegDst;
    EXMEM_REG.MemRead = IDEX_REG.MemRead;
    EXMEM_REG.MemToReg = IDEX_REG.MemToReg;
    EXMEM_REG.MemWrite = IDEX_REG.MemWrite;
    EXMEM_REG.RegWrite = IDEX_REG.RegWrite;
    EXMEM_REG.SpecialRegHi = IDEX_REG.SpecialRegHi;
    EXMEM_REG.SpecialRegLo = IDEX_REG.SpecialRegLo;
}

void pipe_to_MEMWB()
{
    MEMWB_REG.Jump = EXMEM_REG.Jump;
    MEMWB_REG.BranchGate = EXMEM_REG.BranchGate;
    MEMWB_REG.JUMPADDRESS = EXMEM_REG.JUMPADDRESS;
    MEMWB_REG.RD = EXMEM_REG.RD;
    MEMWB_REG.ALURESULT = EXMEM_REG.ALURESULT;
    MEMWB_REG.ALURESULT2 = EXMEM_REG.ALURESULT2;
    MEMWB_REG.RegDst = EXMEM_REG.RegDst;
    MEMWB_REG.RegWrite = EXMEM_REG.RegWrite;
    MEMWB_REG.MemToReg = EXMEM_REG.MemToReg;
    MEMWB_REG.SpecialRegHi = EXMEM_REG.SpecialRegHi;
    MEMWB_REG.SpecialRegLo = EXMEM_REG.SpecialRegLo;
}

//////////////////////////
//                      //
//    MAIN FUNCTIONS    //
//                      //
//////////////////////////

void writeback()
{
    u32 RD = MEMWB_REG.RD;
    u32 ALURESULT = MEMWB_REG.ALURESULT;
    u32 ALURESULT2 = MEMWB_REG.ALURESULT2;

    if (MEMWB_REG.SpecialRegHi == HIGH && MEMWB_REG.SpecialRegLo == HIGH)
    {
        CURRENT_STATE.HI = ALURESULT;
        CURRENT_STATE.LO = ALURESULT2;
    }
    else if (MEMWB_REG.SpecialRegHi == HIGH)
    {
        CURRENT_STATE.HI = ALURESULT;
    }
    else if (MEMWB_REG.SpecialRegLo == HIGH)
    {
        CURRENT_STATE.LO = ALURESULT2;
    }
    else if (MEMWB_REG.RegWrite == HIGH)
    {
        //write to register
        CURRENT_STATE.REGS[RD] = ALURESULT;
    }

    if (MEMWB_REG.Jump == HIGH)
    {
        printf("jumped\n");
        CURRENT_STATE.PC = MEMWB_REG.JUMPADDRESS;
    }
    else if (MEMWB_REG.BranchGate == HIGH)
    {
        printf("branched\n");
        CURRENT_STATE.PC = MEMWB_REG.JUMPADDRESS;
    }
    else
    {
        CURRENT_STATE.PC = CURRENT_STATE.PC + 4;
    }

    // RESET MEMWB
    reset_MEMWB_pipeline();

}

void memory()
{
    if (EXMEM_REG.MemWrite == HIGH)
    {
        switch(EXMEM_REG.OP)
        {
            case SB:
                mem_write_32(EXMEM_REG.EXTENDEDIMM + EXMEM_REG.RSDATA, get_bits_between(EXMEM_REG.RTDATA,0 ,8));
                break;
            case SH:
                mem_write_32(EXMEM_REG.EXTENDEDIMM + EXMEM_REG.RSDATA, get_bits_between(EXMEM_REG.RTDATA, 0, 16));
                break;
            case SW:
                mem_write_32(EXMEM_REG.EXTENDEDIMM + EXMEM_REG.RSDATA, EXMEM_REG.RTDATA);
                break;
        }
    }
    else if (EXMEM_REG.MemRead == HIGH)
    {
        switch(EXMEM_REG.OP)
        {
            case LBU:
                {u32 virtualAddress = mem_read_32(EXMEM_REG.EXTENDEDIMM + EXMEM_REG.RSDATA);
                EXMEM_REG.ALURESULT = get_bits_between(virtualAddress, 0, 8);}
                break;
            case LHU:
                {u32 virtualAddress = mem_read_32(EXMEM_REG.EXTENDEDIMM + EXMEM_REG.RSDATA);
                EXMEM_REG.ALURESULT = get_bits_between(virtualAddress, 0, 16);}
                break;
            case LH:
                {u32 virtualAddress = mem_read_32(EXMEM_REG.EXTENDEDIMM + EXMEM_REG.RSDATA);
                EXMEM_REG.ALURESULT = convert_to_32(get_bits_between(virtualAddress, 0, 16), 16);}
                break;
            case LB:
                {u32 virtualAddress = mem_read_32(EXMEM_REG.EXTENDEDIMM + EXMEM_REG.RSDATA);
                EXMEM_REG.ALURESULT = convert_to_32(get_bits_between(virtualAddress, 0, 8), 8);}
                break;
            case LW:
                EXMEM_REG.ALURESULT = mem_read_32(EXMEM_REG.EXTENDEDIMM + EXMEM_REG.RSDATA);
                break;
        }
    }

    // Pipe to MEMWB
    pipe_to_MEMWB();

    // Reset EXMEM
    reset_EXMEM_pipeline();
}

void execute()
{
    // Load Data from IDEX register
    u32 ALUDATA1 = IDEX_REG.RSDATA;
    // Simulate MUX if ALUSRC high then use extendImm if not RsData
    u32 ALUDATA2 = (IDEX_REG.ALUSrc == HIGH) ? IDEX_REG.EXTENDEDIMM : IDEX_REG.RTDATA;
    u32 op = IDEX_REG.OP;
    u32 rs = IDEX_REG.RSDATA;
    u32 rt = IDEX_REG.RTDATA;

    // if using alu (ALUOp=High) then find op from FUNCT
    if (IDEX_REG.ALUOp == HIGH)
    {
        switch(op)
        {
            case XORI:
                EXMEM_REG.ALURESULT = rs ^ IDEX_REG.EXTENDEDIMM;
                break;
            case ANDI:
                EXMEM_REG.ALURESULT = rs & IDEX_REG.EXTENDEDIMM;
                break;
            case SLTIU:
                EXMEM_REG.ALURESULT = (rs - IDEX_REG.EXTENDEDIMM) < IDEX_REG.EXTENDEDIMM ? 1 : 0;
                break;
            case SLTI:
                EXMEM_REG.ALURESULT = ((i32) rs - (i32) IDEX_REG.EXTENDEDIMM) < (i32) IDEX_REG.EXTENDEDIMM ? 1 : 0;
                break;
            case LUI:
                EXMEM_REG.ALURESULT = ALUDATA2 << 16;
                break;
            case ORI:
                EXMEM_REG.ALURESULT = ALUDATA2 & 0xFFFF | ALUDATA1;
                break;
            case ADDI:
                EXMEM_REG.ALURESULT = (i32) ALUDATA1 + (i32) ALUDATA2;
                break;
            case ADDIU:
                EXMEM_REG.ALURESULT = ALUDATA1 + ALUDATA2;
                break;
            case BNE:
                EXMEM_REG.JUMPADDRESS = (IDEX_REG.PCPLUS4 - 4) + (IDEX_REG.EXTENDEDIMM << 2);
                // Built in GATE
                EXMEM_REG.BranchGate = (ALUDATA1 != ALUDATA2) && (IDEX_REG.Branch == HIGH) ? HIGH : LOW;
                break;
            case BEQ:
                EXMEM_REG.JUMPADDRESS = (IDEX_REG.PCPLUS4 - 4) + (IDEX_REG.EXTENDEDIMM << 2);
                // Built in GATE
                EXMEM_REG.BranchGate = (ALUDATA1 == ALUDATA2) && (IDEX_REG.Branch == HIGH) ? HIGH : LOW;
                break;
            case BLEZ:
                EXMEM_REG.JUMPADDRESS = (IDEX_REG.PCPLUS4 - 4) + (IDEX_REG.EXTENDEDIMM << 2);
                EXMEM_REG.BranchGate = (((rs >> 31) == 1) || (rs == 0)) && (IDEX_REG.Branch == HIGH) ? HIGH : LOW;
                break;
            case BGTZ:
                EXMEM_REG.JUMPADDRESS = (IDEX_REG.PCPLUS4 - 4) + (IDEX_REG.EXTENDEDIMM << 2);
                EXMEM_REG.BranchGate = (((rs >> 31) == 0) && (rs != 0)) && (IDEX_REG.Branch == HIGH) ? HIGH : LOW;
                break;
            case SPECIAL:
                switch(IDEX_REG.FUNCT)
                {
                case MULT:
                    {
                    int64_t multiplied = (i32) rs * (i32) rt;
                    EXMEM_REG.ALURESULT = (multiplied >> 32) & 0xffffffff;
                    EXMEM_REG.ALURESULT2 = multiplied & 0xffffffff;
                    }
                    break;
                case MULTU:
                    {
                    uint64_t multiplied = (uint64_t) rs * (uint64_t) rt;
                    EXMEM_REG.ALURESULT = (multiplied >> 32) & 0xffffffff;
                    EXMEM_REG.ALURESULT2 = multiplied & 0xffffffff;
                    }
                    break;
                case DIV:
                    if (rt != 0)
                    {
                    EXMEM_REG.ALURESULT = (i32) rs % (i32) rt;
                    EXMEM_REG.ALURESULT2 = (i32) rs / (i32) rt;
                    }
                    break;
                case DIVU:
                    if (rt != 0)
                    {
                    EXMEM_REG.ALURESULT = rs % rt;
                    EXMEM_REG.ALURESULT2 = rs / rt;
                    }
                    break;
                case SLL:
                    EXMEM_REG.ALURESULT = rt << IDEX_REG.SA;
                    break;
                case SRL:
                    EXMEM_REG.ALURESULT = get_bits_between(rt >> IDEX_REG.SA, 0, 32 - IDEX_REG.SA);
                    break;
                case SRA:
                    EXMEM_REG.ALURESULT = rt >> IDEX_REG.SA;
                    break;
                case SLLV:
                    EXMEM_REG.ALURESULT = rt << get_bits_between(rs, 0, 5);
                    break;
                case SRLV:
                    EXMEM_REG.ALURESULT = get_bits_between(rt >> get_bits_between(rs,0, 5), 0, (32 - get_bits_between(rs, 0, 5)));
                    break;
                case SRAV:
                    EXMEM_REG.ALURESULT = (i32) rt >> get_bits_between(rs, 0, 5);
                    break;
                case ADD:
                    EXMEM_REG.ALURESULT = (i32) rs + (i32) rt;
                    break;
                case SUB:
                    EXMEM_REG.ALURESULT = (i32) rs - (i32) rt;
                    break;
                case SUBU:
                    EXMEM_REG.ALURESULT = rs - rt;
                    break;
                case AND:
                    EXMEM_REG.ALURESULT = rs & rt;
                    break;
                case OR:
                    EXMEM_REG.ALURESULT = rs | rt;
                    break;
                case XOR:
                    EXMEM_REG.ALURESULT = rs ^ rt;
                    break;
                case NOR:
                    EXMEM_REG.ALURESULT = ~(rs | rt);
                    break;
                case SLT:
                    EXMEM_REG.ALURESULT = (i32) IDEX_REG.RSDATA < (i32) IDEX_REG.RTDATA ? 1 : 0;
                    break;
                case SLTU:
                    EXMEM_REG.ALURESULT = IDEX_REG.RSDATA < IDEX_REG.RTDATA ? 1 : 0;
                    break;
                case MFLO:
                    EXMEM_REG.ALURESULT = IDEX_REG.LO;
                    break;
                case MFHI:
                    EXMEM_REG.ALURESULT = IDEX_REG.HI;
                    break;
                case MTLO:
                    EXMEM_REG.ALURESULT2 = IDEX_REG.RSDATA;
                    break;
                case MTHI:
                    EXMEM_REG.ALURESULT = IDEX_REG.RSDATA;
                    break;
                case ADDU:
                    EXMEM_REG.ALURESULT = ALUDATA1 + ALUDATA2;
                    break;
                case SYSCALL:
                    break;
                }
                break;
            case REGIMM:
                switch (IDEX_REG.RT)
                {
                case BLTZ:
                    EXMEM_REG.JUMPADDRESS = (IDEX_REG.PCPLUS4 - 4) + ((IDEX_REG.EXTENDEDIMM << 2));
                    EXMEM_REG.BranchGate = ((rs >> 31) != 0) && (IDEX_REG.Branch == HIGH) ? HIGH : LOW;
                    break;
                case BGEZ:
                    EXMEM_REG.JUMPADDRESS = (IDEX_REG.PCPLUS4 - 4) + ((IDEX_REG.EXTENDEDIMM << 2));
                    EXMEM_REG.BranchGate = ((rs >> 31) == 0) && (IDEX_REG.Branch == HIGH) ? HIGH : LOW;
                    break;
                case BLTZAL:
                    EXMEM_REG.ALURESULT = IDEX_REG.PCPLUS4;
                    EXMEM_REG.JUMPADDRESS  = (IDEX_REG.PCPLUS4) + ((IDEX_REG.EXTENDEDIMM << 2) - 4);
                    EXMEM_REG.BranchGate = (rs >> 31) != 0 ? HIGH : LOW;
                    break;
                case BGEZAL:
                    EXMEM_REG.ALURESULT = IDEX_REG.PCPLUS4;
                    EXMEM_REG.JUMPADDRESS = (IDEX_REG.PCPLUS4) + ((IDEX_REG.EXTENDEDIMM << 2) - 4);
                    EXMEM_REG.BranchGate = (rs >> 31) == 0 ? HIGH : LOW;
                    break;
                }
                break;
        }
    }
    else
    {
        if (IDEX_REG.Jump == HIGH)
        {
            switch (op)
            {
            case JAL:
                EXMEM_REG.ALURESULT = IDEX_REG.PCPLUS4;
            case J:
                EXMEM_REG.JUMPADDRESS = (IDEX_REG.TARGET << 2);
                break;
            case SPECIAL:
                switch (IDEX_REG.FUNCT)
                {
                case JR:
                    EXMEM_REG.JUMPADDRESS = IDEX_REG.RSDATA;
                    break;
                }
            }
        }
    }

    // Pipe to EXMEM
    pipe_to_EXMEM();

    // Reset ID/EX
    reset_IDEX_pipeline();
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
        case SB:
        case SH:
        case SW:
            IDEX_REG.MemWrite = HIGH;
            CONTROL_UNIT.RegDst = HIGH; //TODO REGDST FOR MUX
            CONTROL_UNIT.ALUOp = HIGH;
            CONTROL_UNIT.ALUSrc = HIGH;
            IDEX_REG.RD = rt(IR);
            IDEX_REG.EXTENDEDIMM = convert_to_32(imm(IR), 16);
            break;
        case LW:
        case LB:
        case LH:
        case LBU:
        case LHU:
            IDEX_REG.MemRead = HIGH;
        case XORI:
        case ANDI:
        case SLTIU:
        case SLTI:
        case LUI:
        case ORI:
        case ADDI: // I-Type
        case ADDIU:
            printf("I-Type\n");
            CONTROL_UNIT.RegDst = HIGH; //TODO REGDST FOR MUX
            CONTROL_UNIT.ALUOp = HIGH;
            CONTROL_UNIT.RegWrite = HIGH;
            CONTROL_UNIT.ALUSrc = HIGH;
            IDEX_REG.RD = rt(IR);
            printf("CHECK: %u\n", IDEX_REG.RD);
            IDEX_REG.EXTENDEDIMM = convert_to_32(imm(IR), 16);
            break;
        case BLEZ:
        case BGTZ:
        case BNE:
        case BEQ: // I-Type (but branch)
            CONTROL_UNIT.Branch = HIGH;
            CONTROL_UNIT.ALUOp = HIGH;
            IDEX_REG.EXTENDEDIMM = convert_to_32(imm(IR), 16);
            break;
        case JAL:
            CONTROL_UNIT.RegWrite = HIGH;
            IDEX_REG.RD = 31;
        case J: // J-Type
            printf("Jump\n");
            CONTROL_UNIT.Jump = HIGH;
            IDEX_REG.TARGET = target(IR);
            break;
        case SPECIAL: // R-Type
            switch(specialOpCode) // This op code needs RegWrite
            {
                case SLL:
                case SRL:
                case SRA:
                case SLLV:
                case SRLV:
                case SRAV:
                case ADD:
                case SUB:
                case SUBU:
                case AND:
                case OR:
                case XOR:
                case NOR:
                case SLT:
                case SLTU:
                case ADDU:
                    CONTROL_UNIT.RegDst = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    CONTROL_UNIT.RegWrite = HIGH;
                    IDEX_REG.RD = rd(IR);
                    IDEX_REG.FUNCT = funct(IR);
                    break;
                case MULT:
                case MULTU:
                case DIV:
                case DIVU:
                    CONTROL_UNIT.RegWrite = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    IDEX_REG.SpecialRegHi = HIGH;
                    IDEX_REG.SpecialRegLo = HIGH;
                    IDEX_REG.FUNCT = funct(IR);
                    break;
                case MFHI:
                case MFLO:
                    CONTROL_UNIT.RegWrite = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    IDEX_REG.RD = rd(IR);
                    IDEX_REG.FUNCT = funct(IR);
                    break;
                case MTLO:
                    CONTROL_UNIT.RegWrite = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    IDEX_REG.SpecialRegLo = HIGH;
                    IDEX_REG.FUNCT = funct(IR);
                    break;
                case MTHI:
                    CONTROL_UNIT.RegWrite = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    IDEX_REG.SpecialRegHi = HIGH;
                    IDEX_REG.FUNCT = funct(IR);
                    break;
                case JR:
                    CONTROL_UNIT.Jump = HIGH;
                    IDEX_REG.FUNCT = funct(IR);
                    break;
                case SYSCALL:
                    RUN_BIT = 0;
                    break;
            }
            break;
        case REGIMM:
            switch (rt(IR))
            {
                case BLTZAL:
                case BGEZAL:
                    CONTROL_UNIT.RegWrite = HIGH;
                    IDEX_REG.RD = 31;
                case BLTZ:
                case BGEZ:
                    CONTROL_UNIT.Branch = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    IDEX_REG.EXTENDEDIMM = convert_to_32(imm(IR), 16);
                    break;
            }
    }

    // Stalling should be here
    // Pipe everything into IDEX register pipeline
    pipe_to_IDEX();

    // RESET IFID
    reset_IFID_pipeline();

    // RESET CONTROL UNIT
    reset_control_unit();
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