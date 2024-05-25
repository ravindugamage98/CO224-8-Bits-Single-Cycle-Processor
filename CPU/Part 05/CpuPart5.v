// CPU module 
module CPU (PC, INSTRUCTION, CLK, RESET);

    // Port Declaration
    input RESET;
    input CLK;
    input [31:0] INSTRUCTION;
    output [31:0] PC;

    // Declaring wires to connect the modules 
    wire [7:0] opcode;
    wire [7:0] immediate;
    wire [7:0] offset7;
    wire [2:0] readReg1_add, readReg2_add, writeReg_add;
    
    // Decoder module Instantiation
    Decoder decoderInstance (INSTRUCTION, opcode, immediate, offset7, readReg1_add, readReg2_add, writeReg_add);
    // module Decoder (INSTRUCTION, OPCODE, IMMEDIATE, OFFSET, RT, RS, RD);

    wire sub_trigger, imm_trigger, writeenable, j, branch_eq,branch_nq;
    wire [2:0] alu_op;

    // Control_Unit module Instantiation
    Control_Unit controlUnitInstance (opcode, sub_trigger, imm_trigger, alu_op, writeenable, j, branch_eq,branch_nq);
    // module Control_Unit (OPCODE, SUB_TRIGGER, IMM_TRIGGER, ALU_OP, WRITE_ENABLE, J, BRANCH);

    wire [7:0] regOut1, regOut2, aluResult;

    // Reg_file module Instantiation
    reg_file registerInstance (aluResult, regOut1, regOut2, writeReg_add, readReg1_add, readReg2_add, writeenable, CLK, RESET);
    // module reg_file (IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET)

    wire [7:0] twoscomplement;

    // TwoS_Complement module Instantiation
    TwoS_Complement twoscomplementInstance (regOut2, twoscomplement);
    // module TwoS_Complement (VALUE, TWOS_COMPLEMENT)

    wire [7:0] mux1_Out;

    // MUX module Instantiation(Mux1)
    MUX7 mux1 (regOut2, twoscomplement, sub_trigger, mux1_Out);
    // module MUX (REG1, REG2, SELECT, RESULT)

    wire [7:0] mux2_Out;

    // MUX module Instantiation(Mux2)
    MUX7 mux2 (mux1_Out, immediate, imm_trigger, mux2_Out);
    // module MUX (REG1, REG2, SELECT, RESULT)

    wire zero;

    // ALU module Instantiation
    alu aluInstance (regOut1, mux2_Out, aluResult, zero, alu_op);
    // module alu (DATA1, DATA2, ALURESULT, ZERO, ALUOP);

    wire [31:0] PCNEXT;
    wire [31:0] PCJBNext;
    wire [31:0] PCtobeExecuted;
    wire [31:0] offset32;

    ShiftingExtension shiftExtensionInstance (offset7, offset32);
    // module ShiftingExtension (CURRENT_OFFSET, UPDATED_OFFSET);

    wire beqOK;
    wire bnqOK;
    wire BJOK;

    wire zeroBar;
    
     // Module to invert the ALU's ZERO output
    logicalNOT NOTInstance (zero, zeroBar); //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // module logicalNOT (ALU_ZERO, ALU_ZERO_BAR); 
    

    // Module to check whether the requirements to perform a BEQ instruction are present
    logicalAND ANDInstance1 (branch_eq, zero, beqOK);
    // module logicalAND (BEQ, ALU_ZERO, BEQ_OK);

    // Module to check whether the requirements to perform a BNQ instruction are present
    logicalAND ANDInstance2 (branch_nq, zeroBar, bnqOK); //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // module logicalAND (BEQ, ALU_ZERO, BEQ_OK);  

    // module to check are there any BEQ or Jump instruction to perfrom
    logicalOR ORInstance (beqOK,j, bnqOK, BJOK);
    //module logicalOR (BEQ_OK, J, BNQ_OK, BJ_OK);

    // PC_Adder module Instantiation
    PC_Adder pcAdderInstance (PCNEXT ,PC);
    // module PC_Adder (PC_NEXT, PC);

    // PC_JBEQ_ADDER module Instatiation
    PC_JB_ADDER pcJBAdder (PCNEXT, offset32, INSTRUCTION, PCJBNext);
    // module PC_JB_ADDER (PC_NEXT, OFFSET_32, INSTRUCTION, PC_JB_NEXT);

    // module to select PC value to be executed next from the two adders' outputs
    MUX32 MUX32Instance (PCNEXT, PCJBNext, BJOK, PCtobeExecuted);
    // module MUX32 (PC_NEXT, PC_JBEQ_NEXT, BEQ_J_OK, PC_tobe_Executed);

    // PC module Instantiation
    PC pcInstance (RESET, CLK, PCtobeExecuted, PC);
    // module PC (RESET, CLK, PC_tobe_Executed, PC);

endmodule

/****************************************************************************************************************************************/
// Module to simulate NOT gate's functionality
module logicalNOT (ALU_ZERO, ALU_ZERO_BAR);

    // Port Declaration
    input ALU_ZERO;
    output reg ALU_ZERO_BAR;

    // This always block will execute whenever we change the value of 
    // the values of the 2 inputs for module
    always @(ALU_ZERO) begin

        // performing the logical NOT operation
        ALU_ZERO_BAR = !ALU_ZERO;

    end

endmodule

/****************************************************************************************************************************************/
// Module to simulate AND gate's functionality
module logicalAND (BEQ, ALU_ZERO, BEQ_OK);

    // Port Declaration
    input BEQ;
    input ALU_ZERO;
    output reg BEQ_OK;

    // This always block will execute whenever we change the value of 
    // the values of the 2 inputs for module
    always @(*) begin

        // performing the logical AND operation
        BEQ_OK = BEQ && ALU_ZERO;

    end

endmodule

/****************************************************************************************************************************************/
// Module to simulate OR gate's functionality
module logicalOR (BEQ_OK, J, BNQ_OK, BJ_OK);

    // Port Declaration
    input BEQ_OK;
    input BNQ_OK;
    input J;
    output reg BJ_OK;

    

    // This always block will execute whenever we change the value of 
    // the values of the 3 inputs for module
    always @(BEQ_OK,BNQ_OK,J) begin

        // performing the logical OR operation
        BJ_OK = BEQ_OK || BNQ_OK || J;

    end

endmodule

/****************************************************************************************************************************************/
// Module to simulate the functionality of a PC register 
module PC (RESET, CLK, PC_tobe_Executed, PC);

    // Port Declaration
    input RESET, CLK;
    output reg [31:0] PC;
    input [31:0] PC_tobe_Executed;

    always @(posedge CLK) begin

        // Reseting
        if (RESET) begin
            #1 PC = 32'b00_000_000_000_000_000_000_000_000_000_000;
        end
    end

    // Updating the PC register
    always @(posedge CLK) begin

        #1 PC = PC_tobe_Executed;

    end
endmodule

/****************************************************************************************************************************************/
// module to perform PC +4 Adder operation  
module PC_Adder (PC_NEXT, PC);

    // Port Declaration
    output reg [31:0] PC_NEXT;
    input [31:0] PC;

    // This module will update the PC_NEXT reg whenever the PC changes it's value
    always @(PC) begin
       #1 
       PC_NEXT =  PC + 3'b100;
    end
endmodule

/****************************************************************************************************************************************/
// module(adder) to perform jump/branch instructions
module PC_JB_ADDER (PC_NEXT, OFFSET_32, INSTRUCTION, PC_JB_NEXT);

    // Port Declaration
    input [31:0] PC_NEXT;
    input [31:0] INSTRUCTION;
    input [31:0] OFFSET_32;    
    output reg [31:0] PC_JB_NEXT;

    // This module will update the PC_JBEQ_NEXT reg whenever the INSTRUCTION changes it's value
    always @(INSTRUCTION) begin

        #2
        PC_JB_NEXT = PC_NEXT + OFFSET_32;

    end
endmodule

/****************************************************************************************************************************************/
// module to perform left shifting & sign extension 
module ShiftingExtension (CURRENT_OFFSET, UPDATED_OFFSET);

    // Port Declaration
    input [7:0] CURRENT_OFFSET;
    output reg [31:0] UPDATED_OFFSET;

    // Declaring a integer data type to access the bits of the register through looping
    integer counter;

    always @ (CURRENT_OFFSET) begin

        for (counter = 0; counter <32; ++counter) begin
            if(counter < 8) begin
                UPDATED_OFFSET[counter] = CURRENT_OFFSET[counter];
            end
            else begin
                UPDATED_OFFSET[counter] = CURRENT_OFFSET[7];
            end
        end

        UPDATED_OFFSET = UPDATED_OFFSET << 2;           // Left Shifting
                                                        // X = X * 2

        /* 
        Alternative method to do these operations at once
        UPDATED_OFFSET = {{22{CURRENT_OFFSET[7]}},CURRENT_OFFSET,2'b00};  */

    end
    
endmodule

/****************************************************************************************************************************************/
// Module to decode the 32 bit instruction 
module Decoder (INSTRUCTION, OPCODE, IMMEDIATE, OFFSET, RT, RS, RD);

    // Port Declaration
    input [31:0] INSTRUCTION;
    output reg [7:0] OPCODE;
    output reg [7:0] IMMEDIATE;
    output reg [7:0] OFFSET;
    output reg [2:0] RS;
    output reg [2:0] RT;
    output reg [2:0] RD;

    // Assigning the values for the relevant outputs(decoding)
    always @(INSTRUCTION) begin

        // Updating the OPCODE
        OPCODE = INSTRUCTION [31:24];

        // If it is a jump instruction
        if (OPCODE == 8'b00_000_110) begin
            OFFSET = INSTRUCTION [23:16]; 
        end 

        // IF it is a Branch Instruction(BEQ)
        else if (OPCODE == 8'b00_000_111) begin
            OFFSET = INSTRUCTION [23:16];
            RS = INSTRUCTION [2:0];
            RT = INSTRUCTION [10:8]; 
        end

        // IF it is a Branch Instruction(BNQ)
        else if (OPCODE == 8'b00_001_000) begin
            OFFSET = INSTRUCTION [23:16];
            RS = INSTRUCTION [2:0];
            RT = INSTRUCTION [10:8]; 
        end

        else begin
            IMMEDIATE = INSTRUCTION [7:0];
            RS = INSTRUCTION [2:0];
            RT = INSTRUCTION [10:8];
            RD = INSTRUCTION [18:16];
        end
    end

endmodule

/****************************************************************************************************************************************/
// Module to perform the functionality of the control unit
module Control_Unit (OPCODE, SUB_TRIGGER, IMM_TRIGGER, ALU_OP, WRITE_ENABLE, J, BRANCH_EQ,BRANCH_NQ);

    // Port Declaration
    input [7:0] OPCODE;
    output reg [2:0] ALU_OP;
    output reg WRITE_ENABLE;
    output reg SUB_TRIGGER;
    output reg IMM_TRIGGER;
    output reg J;                         // Added in part 4
    output reg BRANCH_EQ;                 // Added in part 4
    output reg BRANCH_NQ;                 // Added in part 5


    // This always block will execute whenever we change the value of the OPCODE
    always @(OPCODE) begin 
    // The values of the opcode will be fed into the alu using a case structure
    case  (OPCODE)

        8'b00_000_000 : begin #1
            ALU_OP = 3'b000;            // loadi operation
            IMM_TRIGGER = 1'b1;         // Triggering the output for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabiling the trigger for BNQ instruction

            end

        8'b00_000_001 : begin #1
            ALU_OP = 3'b000;            // mov operation
            IMM_TRIGGER = 1'b0;         // Triggering the output for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabiling the trigger for BNQ instruction
            end

        8'b00_000_010 : begin #1
            ALU_OP = 3'b001;            // add operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabiling the trigger for BNQ instruction
            end

        8'b00_000_011 : begin #1
            ALU_OP = 3'b001;            // sub operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b1;         // Triggering the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabiling the trigger for BNQ instruction
            end

        8'b00_000_100 : begin #1
            ALU_OP = 3'b010;            // and operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabiling the trigger for BNQ instruction
            end

        8'b00_000_101 : begin #1
            ALU_OP = 3'b011;            // or operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabiling the trigger for BNQ instruction
            end

        8'b00_000_110 : begin #1        // Jump Instruction
            WRITE_ENABLE = 1'b0;        // Disabling the write enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            J = 1'b1;                   // Enabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabiling the trigger for BNQ instruction
        end 

        8'b00_000_111 : begin #1        // Branch Instruction (BEQ)
            ALU_OP = 3'b001;            // acts as a sub operation
            WRITE_ENABLE = 1'b0;        // Disabling the write enable signal
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            SUB_TRIGGER = 1'b1;         // Triggering the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b1;           // Enabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabiling the trigger for BNQ instruction
        end

        8'b00_001_000 : begin #1        // Branch Instruction (BNE)
            ALU_OP = 3'b001;            // acts as a sub operation
            WRITE_ENABLE = 1'b0;        // Disabling the write enable signal
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            SUB_TRIGGER = 1'b1;         // Triggering the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b1;           // Enabling the trigger for BNQ instruction
        end

        8'b00_001_001 : begin #1        // logical Shift Right
            ALU_OP = 3'b100;            // new alu opcode for lsr
            WRITE_ENABLE = 1'b1;        // Enabling the write enable signal
            IMM_TRIGGER = 1'b1;         // Enabling the trigger for an immediate value
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabling the trigger for BNQ instruction
        end

        8'b00_001_010 : begin #1        // logical Shift Left
            ALU_OP = 3'b101;            // new alu opcode for lsl
            WRITE_ENABLE = 1'b1;        // Enabling the write enable signal
            IMM_TRIGGER = 1'b1;         // Enabling the trigger for an immediate value
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabling the trigger for BNQ instruction
        end

        8'b00_001_011 : begin #1        // Arithmetic Shift Right
            ALU_OP = 3'b110;            // new alu opcode for asr
            WRITE_ENABLE = 1'b1;        // Enabling the write enable signal
            IMM_TRIGGER = 1'b1;         // Enabling the trigger for an immediate value
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabling the trigger for BNQ instruction
        end

        8'b00_001_100 : begin #1        // Rotate Right
            ALU_OP = 3'b111;            // new alu opcode for ror
            WRITE_ENABLE = 1'b1;        // Enabling the write enable signal
            IMM_TRIGGER = 1'b1;         // Enabling the trigger for an immediate value
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BRANCH_EQ = 1'b0;           // Disabling the trigger for BEQ instruction
            BRANCH_NQ = 1'b0;           // Disabling the trigger for BNQ instruction
        end

    endcase
    end
endmodule

/****************************************************************************************************************************************/
// Module to convert a given binary number to it's 2's Compliment
module TwoS_Complement (VALUE, TWOS_COMPLEMENT);

    // Port Declaration
    input [7:0] VALUE;
    output reg [7:0] TWOS_COMPLEMENT;
    reg [7:0] Temp;

    always @(VALUE) begin
        // computing the 2's complement value of a given value
        Temp = ~VALUE + 8'b00_000_001;

        // final output
        #1 TWOS_COMPLEMENT = Temp;
    end
endmodule

/****************************************************************************************************************************************/
// Module to simulate a MUX's functionality
module MUX32 (PC_NEXT, PC_JBEQ_NEXT, BEQ_J_OK, PC_tobe_Executed);

    // Port Declaration
    input [31:0] PC_NEXT;
    input [31:0] PC_JBEQ_NEXT;
    input BEQ_J_OK;
    output reg [31:0] PC_tobe_Executed;

    // This always block will execute whenever we change the value of 
    // the values of the 3 inputs for the MUX
    always @(*) begin

        // If (SELECT == 3'b001) 
        if (BEQ_J_OK) begin 
            PC_tobe_Executed = PC_JBEQ_NEXT;       // Final result will be the value of the REG2
        end 

        else PC_tobe_Executed = PC_NEXT;           // else the final result will be the value of the REG1

    end

endmodule

/****************************************************************************************************************************************/
// Module to simulate a MUX's functionality
module MUX7 (REG1, REG2, MUXSELECT, MUXOUT);

    // Port Declaration
    input [7:0] REG1;
    input [7:0] REG2;
    input MUXSELECT;
    output reg [7:0] MUXOUT;

    // This always block will execute whenever we change the value of 
    // the values of the 3 inputs for the MUX
    always @(*) begin

        // If (SELECT == 3'b001) 
        if (MUXSELECT) begin 
            MUXOUT = REG2;       // Final result will be the value of the REG2
        end 

        else MUXOUT = REG1;      // else the final result will be the value of the REG1

    end

endmodule

//*********************************************************** 8-bit ALU *****************************************************************//
// 8-bit ALU module
module alu (DATA1, DATA2, ALURESULT, ZERO, ALUOP);

    // Port Declaration
    input [7:0] DATA1, DATA2;
    input [2:0] ALUOP;
    output  [7:0] ALURESULT;
    output reg ZERO;
    wire [7:0] fwdResult, addResult, andResult, orResult,lsrResult,lslResult,asrResult,rorResult;
    reg [2:0]temp;

    always@(DATA2) begin
        temp=DATA2[2:0];
    end

    // Instantiating the modules
    FWD fwd1(DATA2, fwdResult);                          // Forward 
    ADD add1(DATA1, DATA2, addResult);                   // ADD 
    AND and1(DATA1, DATA2, andResult);                   // Bitwise AND 
    OR or1(DATA1, DATA2, orResult);                      // Bitwise OR 

    logicalShiftRight lsr(DATA1,temp,lsrResult);         // Logical Shift Right 
    logicalShiftLeft lsl(DATA1,temp, lslResult);         // Logical Shift Left
    arithmeticShiftRight asr(DATA1,temp,asrResult);      // Arithmetic Shift Right
    rotateRight ror(DATA1,temp, rorResult);              // Rotate Right


    // This always block will execute whenever we change the DATA1, DATA2 & ALUOP inputs
    assign ALURESULT = (ALUOP == 3'b000 ) ? fwdResult : 
                    (ALUOP == 3'b001) ? addResult :
                    (ALUOP == 3'b010) ? andResult :
                    (ALUOP == 3'b011) ? orResult :
                    (ALUOP == 3'b100) ? lsrResult :
                    (ALUOP == 3'b101) ? lslResult :
                    (ALUOP == 3'b110) ? asrResult :
                    (ALUOP == 3'b111) ? rorResult : 8'b0000_0000;

    always @(addResult) begin

        if (addResult == 8'b00_000_000) begin
            ZERO = 1'b1;
        end

        else begin
            ZERO = 1'b0;
        end
    end 

endmodule

// module to perform forward instruction 
module FWD(DATA2, RESULT);

    // Port Declaration
    input [7:0] DATA2;
    output [7:0] RESULT;

        // Updating the result after #1 unit time delay
        assign #1 RESULT = DATA2;

endmodule

// module to perform add instruction 
module ADD(DATA1, DATA2, RESULT);

    // Port Declaration
    input [7:0] DATA1, DATA2;
    output [7:0] RESULT;

        // Updating the result after #2 unit time delay
        assign #2 RESULT = DATA1 + DATA2;

endmodule

// module to perform and instruction 
module AND(DATA1, DATA2, RESULT);

    // Port Declaration
    input [7:0] DATA1, DATA2;
    output [7:0] RESULT;

        // Updating the result after #1 unit time delay
        assign #1 RESULT = DATA1 & DATA2;

endmodule

// module to perform or instruction 
module OR(DATA1, DATA2, RESULT);

    // Port Declaration
    input [7:0] DATA1, DATA2;
    output [7:0] RESULT;

        // Updating the result after #1 unit time delay
        assign #1 RESULT = DATA1 | DATA2;

endmodule

//module to perform logical shift right
module logicalShiftRight(in, ctrl, allout);
  input  [7:0] in;
  input [2:0] ctrl;
  output reg  [7:0] allout;//>>>>>>>>>>>>>>
  wire [7:0] out;//>>>>>>>>>>>>>>>
  wire [7:0] x,y;

  always@(in) begin
    #2 allout = out;
  end

  //4bit shift right
  mux2X1  ins_17 (.in0(in[7]),.in1(1'b0),.sel(ctrl[2]),.out(x[7]));
  mux2X1  ins_16 (.in0(in[6]),.in1(1'b0),.sel(ctrl[2]),.out(x[6]));
  mux2X1  ins_15 (.in0(in[5]),.in1(1'b0),.sel(ctrl[2]),.out(x[5]));
  mux2X1  ins_14 (.in0(in[4]),.in1(1'b0),.sel(ctrl[2]),.out(x[4]));
  mux2X1  ins_13 (.in0(in[3]),.in1(in[7]),.sel(ctrl[2]),.out(x[3]));
  mux2X1  ins_12 (.in0(in[2]),.in1(in[6]),.sel(ctrl[2]),.out(x[2]));
  mux2X1  ins_11 (.in0(in[1]),.in1(in[5]),.sel(ctrl[2]),.out(x[1]));
  mux2X1  ins_10 (.in0(in[0]),.in1(in[4]),.sel(ctrl[2]),.out(x[0]));
  
  //2 bit shift right
  mux2X1  ins_27 (.in0(x[7]),.in1(1'b0),.sel(ctrl[1]),.out(y[7]));
  mux2X1  ins_26 (.in0(x[6]),.in1(1'b0),.sel(ctrl[1]),.out(y[6]));
  mux2X1  ins_25 (.in0(x[5]),.in1(x[7]),.sel(ctrl[1]),.out(y[5]));
  mux2X1  ins_24 (.in0(x[4]),.in1(x[6]),.sel(ctrl[1]),.out(y[4]));
  mux2X1  ins_23 (.in0(x[3]),.in1(x[5]),.sel(ctrl[1]),.out(y[3]));
  mux2X1  ins_22 (.in0(x[2]),.in1(x[4]),.sel(ctrl[1]),.out(y[2]));
  mux2X1  ins_21 (.in0(x[1]),.in1(x[3]),.sel(ctrl[1]),.out(y[1]));
  mux2X1  ins_20 (.in0(x[0]),.in1(x[2]),.sel(ctrl[1]),.out(y[0]));
  
  //1 bit shift right
  mux2X1  ins_07 (.in0(y[7]),.in1(1'b0),.sel(ctrl[0]),.out(out[7]));
  mux2X1  ins_06 (.in0(y[6]),.in1(y[7]),.sel(ctrl[0]),.out(out[6]));
  mux2X1  ins_05 (.in0(y[5]),.in1(y[6]),.sel(ctrl[0]),.out(out[5]));
  mux2X1  ins_04 (.in0(y[4]),.in1(y[5]),.sel(ctrl[0]),.out(out[4]));
  mux2X1  ins_03 (.in0(y[3]),.in1(y[4]),.sel(ctrl[0]),.out(out[3]));
  mux2X1  ins_02 (.in0(y[2]),.in1(y[3]),.sel(ctrl[0]),.out(out[2]));
  mux2X1  ins_01 (.in0(y[1]),.in1(y[2]),.sel(ctrl[0]),.out(out[1]));
  mux2X1  ins_00 (.in0(y[0]),.in1(y[1]),.sel(ctrl[0]),.out(out[0]));
 
endmodule

//module to perform logical shift left
module logicalShiftLeft(in, ctrl, allout);
  input  [7:0] in;
  input [2:0] ctrl;
  output reg [7:0] allout;//>>>>>>>>>>>>>>
  wire [7:0] out;//>>>>>>>>>>>>>>>
  wire [7:0] x,y;

 always@(in) begin
    #2 allout = out;
 end

    //4bit shift right
    mux2X1  ins_17 (.in0(in[0]),.in1(1'b0),.sel(ctrl[2]),.out(x[7]));
    mux2X1  ins_16 (.in0(in[1]),.in1(1'b0),.sel(ctrl[2]),.out(x[6]));
    mux2X1  ins_15 (.in0(in[2]),.in1(1'b0),.sel(ctrl[2]),.out(x[5]));
    mux2X1  ins_14 (.in0(in[3]),.in1(1'b0),.sel(ctrl[2]),.out(x[4]));
    mux2X1  ins_13 (.in0(in[4]),.in1(in[0]),.sel(ctrl[2]),.out(x[3]));
    mux2X1  ins_12 (.in0(in[5]),.in1(in[1]),.sel(ctrl[2]),.out(x[2]));
    mux2X1  ins_11 (.in0(in[6]),.in1(in[2]),.sel(ctrl[2]),.out(x[1]));
    mux2X1  ins_10 (.in0(in[7]),.in1(in[3]),.sel(ctrl[2]),.out(x[0]));
    
    //2 bit shift right
    mux2X1  ins_27 (.in0(x[7]),.in1(1'b0),.sel(ctrl[1]),.out(y[7]));
    mux2X1  ins_26 (.in0(x[6]),.in1(1'b0),.sel(ctrl[1]),.out(y[6]));
    mux2X1  ins_25 (.in0(x[5]),.in1(x[7]),.sel(ctrl[1]),.out(y[5]));
    mux2X1  ins_24 (.in0(x[4]),.in1(x[6]),.sel(ctrl[1]),.out(y[4]));
    mux2X1  ins_23 (.in0(x[3]),.in1(x[5]),.sel(ctrl[1]),.out(y[3]));
    mux2X1  ins_22 (.in0(x[2]),.in1(x[4]),.sel(ctrl[1]),.out(y[2]));
    mux2X1  ins_21 (.in0(x[1]),.in1(x[3]),.sel(ctrl[1]),.out(y[1]));
    mux2X1  ins_20 (.in0(x[0]),.in1(x[2]),.sel(ctrl[1]),.out(y[0]));
    
    //1 bit shift right
    mux2X1  ins_07 (.in0(y[7]),.in1(1'b0),.sel(ctrl[0]),.out(out[0]));
    mux2X1  ins_06 (.in0(y[6]),.in1(y[7]),.sel(ctrl[0]),.out(out[1]));
    mux2X1  ins_05 (.in0(y[5]),.in1(y[6]),.sel(ctrl[0]),.out(out[2]));
    mux2X1  ins_04 (.in0(y[4]),.in1(y[5]),.sel(ctrl[0]),.out(out[3]));
    mux2X1  ins_03 (.in0(y[3]),.in1(y[4]),.sel(ctrl[0]),.out(out[4]));
    mux2X1  ins_02 (.in0(y[2]),.in1(y[3]),.sel(ctrl[0]),.out(out[5]));
    mux2X1  ins_01 (.in0(y[1]),.in1(y[2]),.sel(ctrl[0]),.out(out[6]));
    mux2X1  ins_00 (.in0(y[0]),.in1(y[1]),.sel(ctrl[0]),.out(out[7]));
 
endmodule

//module to perform arithmetic shift right
module arithmeticShiftRight (in, ctrl, allout);
  input  [7:0] in;
  input [2:0] ctrl;
  output reg [7:0] allout;//>>>>>>>>>>>>>>
  wire [7:0] out;//>>>>>>>>>>>>>>>
  wire [7:0] x,y;

 always@(in) begin
    #2 allout = out;
 end
 
    //4bit shift right
    mux2X1  ins_17 (.in0(in[7]),.in1(in[7]),.sel(ctrl[2]),.out(x[7]));
    mux2X1  ins_16 (.in0(in[6]),.in1(in[7]),.sel(ctrl[2]),.out(x[6]));
    mux2X1  ins_15 (.in0(in[5]),.in1(in[7]),.sel(ctrl[2]),.out(x[5]));
    mux2X1  ins_14 (.in0(in[4]),.in1(in[7]),.sel(ctrl[2]),.out(x[4]));
    mux2X1  ins_13 (.in0(in[3]),.in1(in[7]),.sel(ctrl[2]),.out(x[3]));
    mux2X1  ins_12 (.in0(in[2]),.in1(in[6]),.sel(ctrl[2]),.out(x[2]));
    mux2X1  ins_11 (.in0(in[1]),.in1(in[5]),.sel(ctrl[2]),.out(x[1]));
    mux2X1  ins_10 (.in0(in[0]),.in1(in[4]),.sel(ctrl[2]),.out(x[0]));
    
    //2 bit shift right
    mux2X1  ins_27 (.in0(x[7]),.in1(in[7]),.sel(ctrl[1]),.out(y[7]));
    mux2X1  ins_26 (.in0(x[6]),.in1(in[7]),.sel(ctrl[1]),.out(y[6]));
    mux2X1  ins_25 (.in0(x[5]),.in1(x[7]),.sel(ctrl[1]),.out(y[5]));
    mux2X1  ins_24 (.in0(x[4]),.in1(x[6]),.sel(ctrl[1]),.out(y[4]));
    mux2X1  ins_23 (.in0(x[3]),.in1(x[5]),.sel(ctrl[1]),.out(y[3]));
    mux2X1  ins_22 (.in0(x[2]),.in1(x[4]),.sel(ctrl[1]),.out(y[2]));
    mux2X1  ins_21 (.in0(x[1]),.in1(x[3]),.sel(ctrl[1]),.out(y[1]));
    mux2X1  ins_20 (.in0(x[0]),.in1(x[2]),.sel(ctrl[1]),.out(y[0]));
    
    //1 bit shift right
    mux2X1  ins_07 (.in0(y[7]),.in1(in[7]),.sel(ctrl[0]),.out(out[7]));
    mux2X1  ins_06 (.in0(y[6]),.in1(y[7]),.sel(ctrl[0]),.out(out[6]));
    mux2X1  ins_05 (.in0(y[5]),.in1(y[6]),.sel(ctrl[0]),.out(out[5]));
    mux2X1  ins_04 (.in0(y[4]),.in1(y[5]),.sel(ctrl[0]),.out(out[4]));
    mux2X1  ins_03 (.in0(y[3]),.in1(y[4]),.sel(ctrl[0]),.out(out[3]));
    mux2X1  ins_02 (.in0(y[2]),.in1(y[3]),.sel(ctrl[0]),.out(out[2]));
    mux2X1  ins_01 (.in0(y[1]),.in1(y[2]),.sel(ctrl[0]),.out(out[1]));
    mux2X1  ins_00 (.in0(y[0]),.in1(y[1]),.sel(ctrl[0]),.out(out[0]));
 
endmodule

//module to perform rotate right 
module rotateRight(in, ctrl, allout);
  input  [7:0] in;
  input [2:0] ctrl;
  output reg [7:0] allout;//>>>>>>>>>>>>>>
  wire [7:0] out;//>>>>>>>>>>>>>>>
  wire [7:0] x,y;
 
    always@(in) begin
        #2 allout = out;
    end

    //4bit shift right
    mux2X1  ins_17 (.in0(in[7]),.in1(in[3]),.sel(ctrl[2]),.out(x[7]));
    mux2X1  ins_16 (.in0(in[6]),.in1(in[2]),.sel(ctrl[2]),.out(x[6]));
    mux2X1  ins_15 (.in0(in[5]),.in1(in[1]),.sel(ctrl[2]),.out(x[5]));
    mux2X1  ins_14 (.in0(in[4]),.in1(in[0]),.sel(ctrl[2]),.out(x[4]));
    mux2X1  ins_13 (.in0(in[3]),.in1(in[7]),.sel(ctrl[2]),.out(x[3]));
    mux2X1  ins_12 (.in0(in[2]),.in1(in[6]),.sel(ctrl[2]),.out(x[2]));
    mux2X1  ins_11 (.in0(in[1]),.in1(in[5]),.sel(ctrl[2]),.out(x[1]));
    mux2X1  ins_10 (.in0(in[0]),.in1(in[4]),.sel(ctrl[2]),.out(x[0]));
    
    //2 bit shift right
    
    mux2X1  ins_27 (.in0(x[7]),.in1(x[1]),.sel(ctrl[1]),.out(y[7]));
    mux2X1  ins_26 (.in0(x[6]),.in1(x[0]),.sel(ctrl[1]),.out(y[6]));
    mux2X1  ins_25 (.in0(x[5]),.in1(x[7]),.sel(ctrl[1]),.out(y[5]));
    mux2X1  ins_24 (.in0(x[4]),.in1(x[6]),.sel(ctrl[1]),.out(y[4]));
    mux2X1  ins_23 (.in0(x[3]),.in1(x[5]),.sel(ctrl[1]),.out(y[3]));
    mux2X1  ins_22 (.in0(x[2]),.in1(x[4]),.sel(ctrl[1]),.out(y[2]));
    mux2X1  ins_21 (.in0(x[1]),.in1(x[3]),.sel(ctrl[1]),.out(y[1]));
    mux2X1  ins_20 (.in0(x[0]),.in1(x[2]),.sel(ctrl[1]),.out(y[0]));
    
    //1 bit shift right
    mux2X1  ins_07 (.in0(y[7]),.in1(y[0]),.sel(ctrl[0]),.out(out[7]));
    mux2X1  ins_06 (.in0(y[6]),.in1(y[7]),.sel(ctrl[0]),.out(out[6]));
    mux2X1  ins_05 (.in0(y[5]),.in1(y[6]),.sel(ctrl[0]),.out(out[5]));
    mux2X1  ins_04 (.in0(y[4]),.in1(y[5]),.sel(ctrl[0]),.out(out[4]));
    mux2X1  ins_03 (.in0(y[3]),.in1(y[4]),.sel(ctrl[0]),.out(out[3]));
    mux2X1  ins_02 (.in0(y[2]),.in1(y[3]),.sel(ctrl[0]),.out(out[2]));
    mux2X1  ins_01 (.in0(y[1]),.in1(y[2]),.sel(ctrl[0]),.out(out[1]));
    mux2X1  ins_00 (.in0(y[0]),.in1(y[1]),.sel(ctrl[0]),.out(out[0]));
 
endmodule

//multiplexer module to select a bit out of two bits
module mux2X1( in0,in1,sel,out);
    input in0,in1;
    input sel;
    output out;

    assign out=(sel)?in1:in0;
endmodule


/****************************************************************************************************************************************/
// 8x8 Register File
module reg_file (IN, REGOUT1, REGOUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);

    // Port Declaration
    input [2:0] INADDRESS, OUT1ADDRESS, OUT2ADDRESS;
    input [7:0] IN;
    output [7:0] REGOUT1, REGOUT2;
    input WRITE, CLK, RESET;

    // Declaring the 8x8-bit registers (register0 - register7)
    reg [7:0] register [7:0];
    
    // Reading data asynchronously from the registers to be given to the ALU 
        assign #2 REGOUT1 = register[OUT1ADDRESS];
        assign #2 REGOUT2 = register[OUT2ADDRESS];

    // Declaring a integer data type to access the registers 
    integer index;

    always @(posedge CLK) begin
        if (RESET) begin
            // for loop to reset all the register values to zero 
            #1
                for (index = 0; index < 8; ++index) begin
                    register[index] <= 8'b00_000_000;
                end
        end

        if (WRITE) begin
            // writing the data in the IN port to the relevant address
            #1 register[INADDRESS] <= IN;
        end
    end

    initial begin
        $dumpfile("cpu_wavedata_G18_part5.vcd");
        for(index=0;index<8;++index)
            $dumpvars(1,register[index]);
    end

    initial
    begin
    #5
    $display("\n\t\t\t==================================================================");
    $display("\t\t\t Change of Register Content Starting from Time #5");
    $display("\t\t\t==================================================================\n");
    $display("\t\ttime\tregs0\tregs1\tregs2\tregs3\tregs4\tregs5\tregs6\tregs7");
    $display("\t\t-------------------------------------------------------------------------------------");
    $monitor($time, "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", register[0], register[1], register[2], register[3], register[4], register[5], register[6], register[7]);
    end
endmodule