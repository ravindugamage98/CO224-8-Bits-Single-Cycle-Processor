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

    wire sub_trigger, imm_trigger, writeenable, j, beq;
    wire [2:0] alu_op;

    // Control_Unit module Instantiation
    Control_Unit controlUnitInstance (opcode, sub_trigger, imm_trigger, alu_op, writeenable, j, beq);
    // module Control_Unit (OPCODE, SUB_TRIGGER, IMM_TRIGGER, ALU_OP, WRITE_ENABLE, J, BEQ);

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
    MUX8 mux1 (regOut2, twoscomplement, sub_trigger, mux1_Out);
    // module MUX (REG1, REG2, SELECT, RESULT)

    wire [7:0] mux2_Out;

    // MUX module Instantiation(Mux2)
    MUX8 mux2 (mux1_Out, immediate, imm_trigger, mux2_Out);
    // module MUX (REG1, REG2, SELECT, RESULT)

    wire BeqResult;

    // ALU module Instantiation
    alu aluInstance (regOut1, mux2_Out, aluResult, BeqResult, alu_op);
    // module alu (DATA1, DATA2, ALURESULT, BEQRESULT, ALUOP);

    wire [31:0] PCNEXT;
    wire [31:0] PCJBeqNext;
    wire [31:0] PCtobeExecuted;
    wire [31:0] offset32;

    ShiftingExtension shiftExtensionInstance (offset7, offset32);
    // module ShiftingExtension (CURRENT_OFFSET, UPDATED_OFFSET);

    wire beqOK;
    wire beqJOK;

    // Module to check whether the requirements to perform a BEQ instruction are present
    logicalAND ANDInstance (beq, BeqResult, beqOK);
    // module logicalAND (BEQ, ALU_ZERO, BEQ_OK);

    // module to check are there any BEQ or Jump instruction to perfrom
    logicalOR ORInstance (beqOK, j, beqJOK);
    // module logicalOR (BEQ, J, BEQ_J_OK);

    // PC_Adder module Instantiation
    PC_Adder pcAdderInstance  (PCNEXT ,PC);
    // module PC_Adder (PC_NEXT, PC);

    // PC_JBEQ_ADDER module Instatiation
    PC_JBEQ_ADDER pcJBeqAdder (PCNEXT, offset32, INSTRUCTION, PCJBeqNext);
    // module PC_JBEQ_ADDER (PC_NEXT, OFFSET_32, INSTRUCTION, PC_JBEQ_NEXT);

    // module to select PC value to be executed next from the two adders' outputs
    MUX32 MUX32Instance (PCNEXT, PCJBeqNext, beqJOK, PCtobeExecuted);
    // module MUX32 (PC_NEXT, PC_JBEQ_NEXT, BEQ_J_OK, PC_tobe_Executed);

    // PC module Instantiation
    PC pcInstance (RESET, CLK, PCtobeExecuted, PC);
    // module PC (RESET, CLK, PC_tobe_Executed, PC);

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
module logicalOR (BEQ_OK, J, BEQ_J_OK);

    // Port Declaration
    input BEQ_OK;
    input J;
    output reg BEQ_J_OK;

    // This always block will execute whenever we change the value of 
    // the values of the 2 inputs for module
    always @(*) begin

        // performing the logical OR operation
        BEQ_J_OK = BEQ_OK || J;

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

// module(adder) to perform jump/branch instructions
module PC_JBEQ_ADDER (PC_NEXT, OFFSET_32, INSTRUCTION, PC_JBEQ_NEXT);

    // Port Declaration
    input [31:0] PC_NEXT;
    input [31:0] INSTRUCTION;
    input [31:0] OFFSET_32;    
    output reg [31:0] PC_JBEQ_NEXT;

    // This module will update the PC_JBEQ_NEXT reg whenever the INSTRUCTION changes it's value
    always @(INSTRUCTION) begin

        #2
        PC_JBEQ_NEXT = PC_NEXT + OFFSET_32;

    end
endmodule

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
module Control_Unit (OPCODE, SUB_TRIGGER, IMM_TRIGGER, ALU_OP, WRITE_ENABLE, J, BEQ);

    // Port Declaration
    input [7:0] OPCODE;
    output reg [2:0] ALU_OP;
    output reg WRITE_ENABLE;
    output reg SUB_TRIGGER;
    output reg IMM_TRIGGER;
    output reg J;                       // Added in part 4
    output reg BEQ;                     // Added in part 4


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
            BEQ = 1'b0;                 // Disabling the trigger for BEQ instruction
            end

        8'b00_000_001 : begin #1
            ALU_OP = 3'b000;            // mov operation
            IMM_TRIGGER = 1'b0;         // Triggering the output for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BEQ = 1'b0;                 // Disabling the trigger for BEQ instruction
            end

        8'b00_000_010 : begin #1
            ALU_OP = 3'b001;            // add operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BEQ = 1'b0;                 // Disabling the trigger for BEQ instruction
            end

        8'b00_000_011 : begin #1
            ALU_OP = 3'b001;            // sub operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b1;         // Triggering the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BEQ = 1'b0;                 // Disabling the trigger for BEQ instruction
            end

        8'b00_000_100 : begin #1
            ALU_OP = 3'b010;            // and operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BEQ = 1'b0;                 // Disabling the trigger for BEQ instruction
            end

        8'b00_000_101 : begin #1
            ALU_OP = 3'b011;            // or operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BEQ = 1'b0;                 // Disabling the trigger for BEQ instruction
            end

        8'b00_000_110 : begin #1        // Jump Instruction
            WRITE_ENABLE = 1'b0;        // Disabling the write enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            J = 1'b1;                   // Enabling the trigger for a jump instruction
            BEQ = 1'b0;                 // Disabling the trigger for BEQ instruction
        end 

        8'b00_000_111 : begin #1        // Branch Instruction (BEQ)
            ALU_OP = 3'b001;            // acts as a sub operation
            WRITE_ENABLE = 1'b0;        // Disabling the write enable signal
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            SUB_TRIGGER = 1'b1;         // Triggering the output for a sub instruction
            J = 1'b0;                   // Disabling the trigger for a jump instruction
            BEQ = 1'b1;                 // Enabling the trigger for BEQ instruction
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
module MUX8(REG1, REG2, MUXSELECT, MUXOUT);

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
module alu (DATA1, DATA2, ALURESULT, BEQRESULT, ALUOP);

    // Port Declaration
    input [7:0] DATA1, DATA2;
    input [2:0] ALUOP;
    output reg [7:0] ALURESULT;
    output reg BEQRESULT;
    wire [7:0] fwdResult, addResult, andResult, orResult;

    // Instantiating the modules
    FWD fwd1(DATA2, fwdResult);                          // Forward Function
    ADD add1(DATA1, DATA2, addResult);                   // ADD function
    AND and1(DATA1, DATA2, andResult);                   // Bitwise AND function
    OR or1(DATA1, DATA2, orResult);                      // Bitwise OR function

    // This always block will execute whenever we change the DATA1, DATA2 & ALUOP inputs
    always @(fwdResult, addResult, andResult, orResult) begin
    // Case structure to simulate the mux's operations
    case (ALUOP)
        3'b000 :  ALURESULT = fwdResult;
        3'b001 :  ALURESULT = addResult;
        3'b010 :  ALURESULT = andResult;
        3'b011 :  ALURESULT = orResult;
        default:  ALURESULT = 8'b00000000;
    endcase
    end

    always @(addResult) begin

        if (addResult == 8'b00_000_000) begin
            BEQRESULT = 1'b1;
        end

        else begin
            BEQRESULT = 1'b0;
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
        $dumpfile("cpu_wavedata_Grp18CPU.vcd");
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