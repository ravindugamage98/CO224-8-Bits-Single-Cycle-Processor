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
    wire [2:0] readReg1_add, readReg2_add, writeReg_add;
    
    // Decoder module Instantiation
    Decoder decoderInstance (INSTRUCTION, opcode, immediate, readReg1_add, readReg2_add, writeReg_add);
    // module Decoder (INSTRUCTION, OPCODE, IMMEDIATE, RS, RT, RD)

    wire sub_trigger, imm_trigger, writeenable;
    wire [2:0] alu_op;

    // Control_Unit module Instantiation
    Control_Unit controlUnitInstance (opcode, sub_trigger, imm_trigger, alu_op, writeenable);
    // module Control_Unit (OPCODE, SUB_TRIGGER, IMM_TRIGGER, ALU_OP, WRITE_ENABLE)

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
    MUX mux1 (regOut2, twoscomplement, sub_trigger, mux1_Out);
    // module MUX (REG1, REG2, SELECT, RESULT)

    wire [7:0] mux2_Out;

    // MUX module Instantiation(Mux2)
    MUX mux2 (mux1_Out, immediate, imm_trigger, mux2_Out);
    // module MUX (REG1, REG2, SELECT, RESULT)

    // ALU module Instantiation
    alu aluInstance (regOut1, mux2_Out, aluResult, alu_op);
    // module alu (DATA1, DATA2, RESULT, SELECT)

    wire [2:0] adder_Result;

    // PC module Instantiation
    PC pcInstance (RESET, CLK, adder_Result, PC);
    // module PC (RESET, CLK, add4, PC_REG)

    // PC_Adder module Instantiation
    PC_Adder pcAdderInstance  (adder_Result,PC);
    // module PC_Adder(VALUE,PC)

endmodule


/****************************************************************************************************************************************/
// Module to simulate the functionality of a PC register 
module PC (RESET, CLK, add4, PC_REG);

    // Port Declaration
    input RESET;
    input CLK;
    output reg [31:0] PC_REG;
    input [2:0] add4;

    always @(posedge CLK) begin

        // Reseting
        if (RESET) begin
            #1 PC_REG = 32'b00_000_000_000_000_000_000_000_000_000_000;
        end
    end

    always @(posedge CLK) begin

        // PC Update
        #1 PC_REG = PC_REG + add4;

    end
endmodule

// module to perform PC +4 Adder operation
module PC_Adder(VALUE,PC);

    // Port Declaration
    output reg [2:0] VALUE;
    input [31:0] PC;

    // This module will output a 3 bit value(4) whenever the PC changes it's value
    always @(PC) begin
       #1 VALUE = 3'b100;
    end
endmodule

/****************************************************************************************************************************************/
// Module to decode the 32 bit instruction 
module Decoder (INSTRUCTION, OPCODE, IMMEDIATE, RS, RT, RD);

    // Port Declaration
    input [31:0] INSTRUCTION;
    output [7:0] OPCODE;
    output [7:0] IMMEDIATE;
    output [2:0] RS;
    output [2:0] RT;
    output [2:0] RD;

    // Assigning the values for the relevant outputs(decoding)
    assign OPCODE = INSTRUCTION [31:24] ;
    assign IMMEDIATE = INSTRUCTION [7:0];
    assign RS = INSTRUCTION [2:0];
    assign RT = INSTRUCTION [10:8];
    assign RD = INSTRUCTION [18:16];

endmodule

/****************************************************************************************************************************************/
// Module to perform the functionality of the control unit
module Control_Unit (OPCODE, SUB_TRIGGER, IMM_TRIGGER, ALU_OP, WRITE_ENABLE);

    // Port Declaration
    input [7:0] OPCODE;
    output reg [2:0] ALU_OP;
    output reg WRITE_ENABLE;
    output reg SUB_TRIGGER;
    output reg IMM_TRIGGER;


    // This always block will execute whenever we change the value of the OPCODE
    always @(OPCODE) begin 
    // The values of the opcode will be fed into the alu using a case structure
    case  (OPCODE)

        8'b00_000_000 : begin #1
            ALU_OP = 3'b000;            // loadi operation
            IMM_TRIGGER = 1'b1;         // Triggering the output for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            end

        8'b00_000_001 : begin #1
            ALU_OP = 3'b000;            // mov operation
            IMM_TRIGGER = 1'b1;         // Triggering the output for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            end

        8'b00_000_010 : begin #1
            ALU_OP = 3'b001;            // add operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            end

        8'b00_000_011 : begin #1
            ALU_OP = 3'b001;            // sub operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b1;         // Triggering the output for a sub instruction
            end

        8'b00_000_100 : begin #1
            ALU_OP = 3'b010;            // and operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
            end

        8'b00_000_101 : begin #1
            ALU_OP = 3'b011;            // or operation
            IMM_TRIGGER = 1'b0;         // Disabling the trigger for an immediate value
            WRITE_ENABLE = 1'b1;        // Enabling the written enable signal
            SUB_TRIGGER = 1'b0;         // Disabling the output for a sub instruction
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
module MUX (REG1, REG2, MUXSELECT, MUXOUT);

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
module alu (DATA1, DATA2, ALURESULT, ALUOP);

    // Port Declaration
    input [7:0] DATA1, DATA2;
    input [2:0] ALUOP;
    output reg [7:0] ALURESULT;
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
endmodule