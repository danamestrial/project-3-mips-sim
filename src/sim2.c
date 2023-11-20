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

#ifdef DEBUG_FLAG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

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
    enum Signal Syscall;
};

struct IDEX_PILELINE_REG IDEX_REG = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

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
    IDEX_REG.Syscall = LOW;
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
    u32 RS;
    u32 RT;
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
    enum Signal Syscall;
};

struct EXMEM_PIPELINE_REG EXMEM_REG = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

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
    EXMEM_REG.RS = 0;
    EXMEM_REG.RT = 0;
    EXMEM_REG.RegDst = LOW;
    EXMEM_REG.Jump = LOW;
    EXMEM_REG.BranchGate = LOW;
    EXMEM_REG.MemRead = LOW;
    EXMEM_REG.MemToReg = LOW;
    EXMEM_REG.MemWrite = LOW;
    EXMEM_REG.RegWrite = LOW;
    EXMEM_REG.SpecialRegHi = LOW;
    EXMEM_REG.SpecialRegLo = LOW;
    EXMEM_REG.Syscall = LOW;
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
    enum Signal Syscall;
};

struct MEMWB_PIPELINE_REG MEMWB_REG = {0, 0, 0, 0, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

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
    MEMWB_REG.Syscall = LOW;
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

struct STALL_CYCLE
{
    int Fetch;
    int Decode;
};

struct STALL_CYCLE STALL = {0, 0};

void reset_stall_cycle()
{
    STALL.Decode = 0;
    STALL.Fetch = 0;
}

struct HAZARD_UNIT
{
    int MAIN[34];
    int IDEX[34];
    int EXMEM[34];
    int MEMWB[34];
};

struct HAZARD_UNIT HAZARD = {{
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1
},{
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1
},{
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1
},{
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1
}};

void reset_hazard_unit()
{
    for (int i = 0; i < 34 ; i++)
    {
        HAZARD.MAIN[i] = 1;
        HAZARD.IDEX[i] = 1;
        HAZARD.EXMEM[i] = 1;
        HAZARD.MEMWB[i] = 1;
    }
};

struct FORWARDING_UNIT
{
    u32 DATA[34];
};

struct FORWARDING_UNIT FORWARD = {{
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0
}};

void reset_forwarding_unit()
{
    for (int i = 0; i < 34 ; i++)
    {
        FORWARD.DATA[i] = 0;
    }
};

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
    IDEX_REG.RS = rs(IFID_REG.IR);
    IDEX_REG.RT = rt(IFID_REG.IR);
    // IDEX_REG.RSDATA = CURRENT_STATE.REGS[rs(IFID_REG.IR)];
    // IDEX_REG.RTDATA = CURRENT_STATE.REGS[rt(IFID_REG.IR)];
    IDEX_REG.SA = shamt(IFID_REG.IR);
    // IDEX_REG.HI = CURRENT_STATE.HI;
    // IDEX_REG.LO = CURRENT_STATE.LO;
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
    EXMEM_REG.Syscall = IDEX_REG.Syscall;
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
    MEMWB_REG.Syscall = EXMEM_REG.Syscall;
}

////////////////////////////
//                        //
//    HAZARD FUNCTIONS    //
//                        //
////////////////////////////

void skew_hazard()
{
    for (int i = 0; i < 34; i++)
    {
        HAZARD.MEMWB[i] = HAZARD.EXMEM[i];
    }

    for (int i = 0; i < 34; i++)
    {
        HAZARD.EXMEM[i] = HAZARD.IDEX[i];
    }

    for (int i = 0; i < 34; i++)
    {
        HAZARD.IDEX[i] = HAZARD.MAIN[i];
        HAZARD.MAIN[i] = 1;
    }
}

int check_single(u32 rs)
{
    if (HAZARD.IDEX[rs] == 0) {
        STALL.Decode = 3;
        STALL.Fetch = 3;
        return FALSE;
    }
    else if (HAZARD.EXMEM[rs] == 0)
    {
        STALL.Decode = 2;
        STALL.Fetch = 2;
        return FALSE;
    }
    else if (HAZARD.MEMWB[rs] == 0)
    {
        STALL.Decode = 1;
        STALL.Fetch = 1;
        return FALSE;
    }
    return TRUE;
}

int check_double(u32 rs, u32 rt)
{
    if (HAZARD.IDEX[rs] == 0 || HAZARD.IDEX[rt] == 0) {
        STALL.Decode = 3;
        STALL.Fetch = 3;
        return FALSE;
    }
    else if (HAZARD.EXMEM[rs] == 0 || HAZARD.EXMEM[rt] == 0)
    {
        STALL.Decode = 2;
        STALL.Fetch = 2;
        return FALSE;
    }
    else if (HAZARD.MEMWB[rs] == 0 || HAZARD.MEMWB[rt] == 0)
    {
        STALL.Decode = 1;
        STALL.Fetch = 1;
        return FALSE;
    }
    return TRUE;
}


void check_dependency(u32 IR)
{
    if (STALL.Decode > 0) { return; }

    u32 rs = rs(IR);
    u32 rt = rt(IR);

    switch(op(IR))
    {
        case SB:
        case SH:
        case SW:
            check_double(rs, rt);
            break;
        case LW:
        case LB:
        case LH:
        case LBU:
        case LHU:
            check_single(rs);
            break;
        case XORI:
        case ANDI:
        case SLTIU:
        case SLTI:
        case LUI:
        case ORI:
        case ADDI:
        case ADDIU:
            check_single(rs);
            break;
        case BLEZ:
        case BGTZ:
            if (check_single(rs))
            {
                STALL.Fetch = 4;
            }
            break;
        case BNE:
        case BEQ:
            if (check_double(rs, rt))
            {
                STALL.Fetch = 4;
            }
            break;
        case JAL:
        case J:
            STALL.Fetch = 4;
            break;
        case SPECIAL: // R-Type
            switch(funct(IR)) // This op code needs RegWrite
            {
                case SLL:
                case SRL:
                case SRA:
                    check_single(rt);
                    break;
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
                    check_double(rs, rt);
                    break;
                case MULT:
                case MULTU:
                case DIV:
                case DIVU:
                    check_double(rs, rt);
                    break;
                case MFHI:
                    // index 33 is HI
                    check_single(33);
                    break;
                case MFLO:
                    // index 32 is LO
                    check_single(32);
                    break;
                case MTLO:
                case MTHI:
                    // Uses rs to store to HI/LO
                    check_single(rs);
                    break;
                case JR:
                    STALL.Fetch = 4;
                    break;
                case SYSCALL:
                    break;
            }
            break;
        case REGIMM:
            switch (rt(IR))
            {
                case BLTZAL:
                case BGEZAL:
                case BLTZ:
                case BGEZ:
                    if (check_single(rs))
                    {
                        STALL.Fetch = 4;
                    }
                    break;
            }
    }
}


int forward_single_rs(u32 rs)
{
    if (HAZARD.IDEX[rs] == 0 || HAZARD.EXMEM[rs] == 0 || HAZARD.MEMWB[rs] == 0)
    {
        DEBUG_PRINT("Found data to forward at: [%d] with value %d\n", rs, FORWARD.DATA[rs]);

        if (rs == 32)
        {
            IDEX_REG.LO = FORWARD.DATA[32];
        }
        else if (rs == 33)
        {
            IDEX_REG.HI = FORWARD.DATA[33];
        }
        else
        {
            IDEX_REG.RSDATA = FORWARD.DATA[rs];
        }
    }
    return TRUE;
}

int forward_single_rt(u32 rt)
{
    if (HAZARD.IDEX[rt] == 0 || HAZARD.EXMEM[rt] == 0 || HAZARD.MEMWB[rt] == 0)
    {
        DEBUG_PRINT("Found data to forward at: [%d] with value %d\n", rt, FORWARD.DATA[rt]);

        if (rt == 32)
        {
            IDEX_REG.LO = FORWARD.DATA[32];
        }
        else if (rt == 33)
        {
            IDEX_REG.HI = FORWARD.DATA[33];
        }
        else
        {
            IDEX_REG.RTDATA = FORWARD.DATA[rt];
        }

        // return FALSE;
    }
    return TRUE;
}

int forward_double(u32 rs, u32 rt)
{
    if ((HAZARD.IDEX[rs] == 0 || HAZARD.IDEX[rt] == 0) || (HAZARD.EXMEM[rs] == 0 || HAZARD.EXMEM[rt] == 0) || (HAZARD.MEMWB[rs] == 0 || HAZARD.MEMWB[rt] == 0))
    {
        DEBUG_PRINT("Found data to forward at: [%d] %d or [%d] %d\n", rs,FORWARD.DATA[rs], rt, FORWARD.DATA[rt]);

        IDEX_REG.RSDATA = FORWARD.DATA[rs];
        IDEX_REG.RTDATA = FORWARD.DATA[rt];
    }
    return TRUE;
}


void forward(u32 IR)
{
    if (STALL.Decode > 0) { return; }

    u32 rs = rs(IR);
    u32 rt = rt(IR);

    switch(op(IR))
    {
        case SB:
        case SH:
        case SW:
            forward_double(rs, rt);
            break;
        case LW:
        case LB:
        case LH:
        case LBU:
        case LHU:
            forward_single_rs(rs);
            break;
        case XORI:
        case ANDI:
        case SLTIU:
        case SLTI:
        case LUI:
        case ORI:
        case ADDI:
        case ADDIU:
            forward_single_rs(rs);
            break;
        case BLEZ:
        case BGTZ:
            forward_single_rs(rs);
            break;
        case BNE:
        case BEQ:
            forward_double(rs, rt);
            break;
        case JAL:
        case J:
            break;
        case SPECIAL: // R-Type
            switch(funct(IR)) // This op code needs RegWrite
            {
                case SLL:
                case SRL:
                case SRA:
                    forward_single_rt(rt);
                    break;
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
                    forward_double(rs, rt);
                    break;
                case MULT:
                case MULTU:
                case DIV:
                case DIVU:
                    forward_double(rs, rt);
                    break;
                case MFHI:
                    // index 33 is HI
                    forward_single_rs(33);
                    break;
                case MFLO:
                    // index 32 is LO
                    forward_single_rs(32);
                    break;
                case MTLO:
                case MTHI:
                    // Uses rs to store to HI/LO
                    forward_single_rs(rs);
                    break;
                case JR:
                    break;
                case SYSCALL:
                    break;
            }
            break;
        case REGIMM:
            switch (rt(IR))
            {
                case BLTZAL:
                case BGEZAL:
                case BLTZ:
                case BGEZ:
                    forward_single_rs(rs);
                    break;
            }
    }
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

    if (MEMWB_REG.Syscall == HIGH) { RUN_BIT = FALSE; return;}

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

        DEBUG_PRINT("Wrote %u to REG[%u]\n", ALURESULT, RD);
    }

    // if (MEMWB_REG.Jump == HIGH)
    // {
    //     DEBUG_PRINT("jumped\n");
    //     CURRENT_STATE.PC = MEMWB_REG.JUMPADDRESS;
    // }
    // else if (MEMWB_REG.BranchGate == HIGH)
    // {
    //     DEBUG_PRINT("branched\n");
    //     CURRENT_STATE.PC = MEMWB_REG.JUMPADDRESS;
    // }
    // else
    // {
    //     // CURRENT_STATE.PC = CURRENT_STATE.PC + 4;
    // }

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

        FORWARD.DATA[IDEX_REG.RD] = EXMEM_REG.ALURESULT;
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
                DEBUG_PRINT("BNE %d != %d\n", ALUDATA1, ALUDATA2);
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

                    FORWARD.DATA[33] = EXMEM_REG.ALURESULT;
                    FORWARD.DATA[32] = EXMEM_REG.ALURESULT2;
                    }
                    break;
                case MULTU:
                    {
                    uint64_t multiplied = (uint64_t) rs * (uint64_t) rt;
                    EXMEM_REG.ALURESULT = (multiplied >> 32) & 0xffffffff;
                    EXMEM_REG.ALURESULT2 = multiplied & 0xffffffff;

                    DEBUG_PRINT("FORWARDING %d to [%d]\n", EXMEM_REG.ALURESULT, 33);
                    DEBUG_PRINT("FORWARDING %d to [%d]\n", EXMEM_REG.ALURESULT2, 32);

                    FORWARD.DATA[33] = EXMEM_REG.ALURESULT;
                    FORWARD.DATA[32] = EXMEM_REG.ALURESULT2;
                    }
                    break;
                case DIV:
                    if (rt != 0)
                    {
                    EXMEM_REG.ALURESULT = (i32) rs % (i32) rt;
                    EXMEM_REG.ALURESULT2 = (i32) rs / (i32) rt;

                    FORWARD.DATA[33] = EXMEM_REG.ALURESULT;
                    FORWARD.DATA[32] = EXMEM_REG.ALURESULT2;
                    }
                    break;
                case DIVU:
                    if (rt != 0)
                    {
                    DEBUG_PRINT("DIVU between %d & %d\n", rs, rt);

                    EXMEM_REG.ALURESULT = rs % rt;
                    EXMEM_REG.ALURESULT2 = rs / rt;

                    DEBUG_PRINT("FORWARDING %d to [%d]\n", EXMEM_REG.ALURESULT, 33);
                    DEBUG_PRINT("FORWARDING %d to [%d]\n", EXMEM_REG.ALURESULT2, 32);

                    FORWARD.DATA[33] = EXMEM_REG.ALURESULT;
                    FORWARD.DATA[32] = EXMEM_REG.ALURESULT2;
                    }
                    break;
                case SLL:
                    EXMEM_REG.ALURESULT = rt << IDEX_REG.SA;
                    break;
                case SRL:
                    EXMEM_REG.ALURESULT = get_bits_between(rt >> IDEX_REG.SA, 0, 32 - IDEX_REG.SA);

                    DEBUG_PRINT("SRL RESULT: %d >> %d -> %d\n", rt, IDEX_REG.SA, EXMEM_REG.ALURESULT);
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
                    DEBUG_PRINT("SUB %d - %d = %d\n", (i32) rs , (i32) rt, (i32) rs - (i32) rt);
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
                    DEBUG_PRINT("GOT HI : %d\n", IDEX_REG.HI);
                    break;
                case MTLO:
                    EXMEM_REG.ALURESULT2 = IDEX_REG.RSDATA;

                    DEBUG_PRINT("FORWARDING %d to [%d]\n", EXMEM_REG.ALURESULT2, 32);
                    FORWARD.DATA[32] = EXMEM_REG.ALURESULT2;
                    break;
                case MTHI:
                    EXMEM_REG.ALURESULT = IDEX_REG.RSDATA;

                    DEBUG_PRINT("FORWARDING %d to [%d]\n", EXMEM_REG.ALURESULT, 33);
                    FORWARD.DATA[33] = EXMEM_REG.ALURESULT;
                    DEBUG_PRINT("MOVE TO HI : %d\n", IDEX_REG.RSDATA);
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
                    EXMEM_REG.JUMPADDRESS  = (IDEX_REG.PCPLUS4) + ((IDEX_REG.EXTENDEDIMM << 2));
                    EXMEM_REG.BranchGate = (rs >> 31) != 0 ? HIGH : LOW;
                    break;
                case BGEZAL:
                    EXMEM_REG.ALURESULT = IDEX_REG.PCPLUS4;
                    EXMEM_REG.JUMPADDRESS = (IDEX_REG.PCPLUS4) + ((IDEX_REG.EXTENDEDIMM << 2));
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
                case JALR:
                    EXMEM_REG.ALURESULT = IDEX_REG.PCPLUS4;
                case JR:
                    EXMEM_REG.JUMPADDRESS = IDEX_REG.RSDATA;
                    break;
                }
            }
        }
    }


    if (IDEX_REG.Jump == HIGH)
    {
        DEBUG_PRINT("jumped\n");
        CURRENT_STATE.PC = EXMEM_REG.JUMPADDRESS;
        // SQUASH Decode and Fetch
        // Reset Decode and Fetch
        // RESET IFID
        reset_IFID_pipeline();

        // RESET CONTROL UNIT
        reset_control_unit();
    }
    else if (EXMEM_REG.BranchGate == HIGH)
    {
        DEBUG_PRINT("branched\n");
        CURRENT_STATE.PC = EXMEM_REG.JUMPADDRESS;
        // RESET IFID
        reset_IFID_pipeline();

        // RESET CONTROL UNIT
        reset_control_unit();
    }

    switch(op)
    {
        case SPECIAL:
            switch(IDEX_REG.FUNCT)
            {
                case DIVU:
                    break;

                default:
                    DEBUG_PRINT("FORWARDING %d to [%d]\n", EXMEM_REG.ALURESULT, IDEX_REG.RD);
                    FORWARD.DATA[IDEX_REG.RD] = EXMEM_REG.ALURESULT;
                    break;
            }
            break;

        default:
            DEBUG_PRINT("FORWARDING %d to [%d]\n", EXMEM_REG.ALURESULT, IDEX_REG.RD);
            FORWARD.DATA[IDEX_REG.RD] = EXMEM_REG.ALURESULT;
            break;
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

    if (IR == 0) { return; }


    IDEX_REG.RSDATA = CURRENT_STATE.REGS[rs(IFID_REG.IR)];
    IDEX_REG.RTDATA = CURRENT_STATE.REGS[rt(IFID_REG.IR)];

    IDEX_REG.HI = CURRENT_STATE.HI;
    IDEX_REG.LO = CURRENT_STATE.LO;

    forward(IR);
    // check_dependency(IR);
    if (STALL.Decode > 0) { DEBUG_PRINT("STALLING in DECODE [%d]\n", STALL.Decode); STALL.Decode--; return; }

    DEBUG_PRINT("DECODE RAN\n");

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
            IDEX_REG.EXTENDEDIMM = convert_to_32(imm(IR), 16);
            break;
        case LW:
        case LB:
        case LH:
        case LBU:
        case LHU:
            IDEX_REG.MemRead = HIGH;
            STALL.Decode = 1;
            STALL.Fetch = 1;
        case XORI:
        case ANDI:
        case SLTIU:
        case SLTI:
        case LUI:
        case ORI:
        case ADDI: // I-Type
        case ADDIU:
            CONTROL_UNIT.RegDst = HIGH; //TODO REGDST FOR MUX
            CONTROL_UNIT.ALUOp = HIGH;
            CONTROL_UNIT.RegWrite = HIGH;
            CONTROL_UNIT.ALUSrc = HIGH;
            IDEX_REG.RD = rt(IR);
            DEBUG_PRINT("ADDIU: %u\n", IDEX_REG.RD);
            IDEX_REG.EXTENDEDIMM = convert_to_32(imm(IR), 16);

            HAZARD.MAIN[IDEX_REG.RD] = 0;
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

            HAZARD.MAIN[IDEX_REG.RD] = 0;
        case J: // J-Type
            DEBUG_PRINT("Jump\n");
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
                    DEBUG_PRINT("ADDU at REG: %u\n", IDEX_REG.RD);

                    HAZARD.MAIN[IDEX_REG.RD] = 0;
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

                    HAZARD.MAIN[33] = 0;
                    HAZARD.MAIN[32] = 0;
                    break;
                case MFHI:
                case MFLO:
                    CONTROL_UNIT.RegWrite = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    IDEX_REG.RD = rd(IR);
                    IDEX_REG.FUNCT = funct(IR);

                    HAZARD.MAIN[IDEX_REG.RD] = 0;
                    break;
                case MTLO:
                    CONTROL_UNIT.RegWrite = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    IDEX_REG.SpecialRegLo = HIGH;
                    IDEX_REG.FUNCT = funct(IR);

                    HAZARD.MAIN[32] = 0;
                    break;
                case MTHI:
                    CONTROL_UNIT.RegWrite = HIGH;
                    CONTROL_UNIT.ALUOp = HIGH;
                    IDEX_REG.SpecialRegHi = HIGH;
                    IDEX_REG.FUNCT = funct(IR);

                    HAZARD.MAIN[33] = 0;
                    break;
                case JR:
                    CONTROL_UNIT.Jump = HIGH;
                    IDEX_REG.FUNCT = funct(IR);
                    break;
                case JALR:
                    CONTROL_UNIT.Jump = HIGH;
                    CONTROL_UNIT.RegWrite = HIGH;
                    IDEX_REG.FUNCT = funct(IR);
                    IDEX_REG.RD = rd(IR) == 0 ? 31 : rd(IR);

                    HAZARD.MAIN[IDEX_REG.RD] = 0;
                    break;
                case SYSCALL:
                    DEBUG_PRINT("GOT SYSCALL\n");
                    IDEX_REG.Syscall = HIGH;
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

                    HAZARD.MAIN[IDEX_REG.RD] = 0;
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
    if (STALL.Fetch > 0) { DEBUG_PRINT("STALLING in FETCH [%d]\n", STALL.Fetch); STALL.Fetch--; return; }

    DEBUG_PRINT("FETCHED \n");

    u32 IR = instruction_memory(CURRENT_STATE.PC);
    u32 PCPLUS4 = CURRENT_STATE.PC + 4;

    if (IR == 0) { return; }

    IFID_REG.IR = IR;
    IFID_REG.PCPLUS4 = PCPLUS4;
    CURRENT_STATE.PC = CURRENT_STATE.PC + 4;
}

/*

NORMAL INSTRUCTIONS SWITCH CASE HERE ( process_instruction() )

*/

void process_instruction()
{
    skew_hazard();

    writeback();

    memory();

    execute();

    decode();

    fetch();

    if (RUN_BIT == FALSE)
    {
        reset_control_unit();
        reset_hazard_unit();
        reset_stall_cycle();
        reset_IFID_pipeline();
        reset_IDEX_pipeline();
        reset_EXMEM_pipeline();
        reset_MEMWB_pipeline();
        reset_forwarding_unit();
    }

    // fetch();
    // DEBUG_PRINT("f\n");

    // decode();
    // DEBUG_PRINT("d\n");

    // execute();
    // DEBUG_PRINT("e\n");

    // memory();
    // DEBUG_PRINT("m\b");

    // writeback();
    // DEBUG_PRINT("wb\n");

    /* execute one instruction here. You should use CURRENT_STATE and modify
     * values in NEXT_STATE. You can call mem_read_32() and mem_write_32() to
     * access memory. */
}