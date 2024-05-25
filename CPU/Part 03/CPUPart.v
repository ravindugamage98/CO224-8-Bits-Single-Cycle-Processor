// CPU module 
module CPU (PC, INSTRUCTION, CLK, RESET);

    // Port Declaration
    input RESET;
    input CLK;
    input [31:0] INSTRUCTION;
    output [31:0] PC;

    wire [7:0] opcode;
    wire [7:0] immediate;
    wire [2:0] readReg1_add, readReg2_add, writeReg_add;

    Decoder decoderInstance (INSTRUCTION, opcode, immediate, readReg1_add, readReg2_add, writeReg_add);

    wire sub_trigger, imm_trigger, writeenable;
    wire [2:0] alu_op;

    Control_Unit controlUnitInstance (opcode, sub_trigger, imm_trigger, alu_op, writeenable);

    wire [7:0] regOut1, regOut2, aluResult;

    reg_file registerInstance (aluResult, regOut1, regOut2, writeReg_add, readReg1_add, readReg2_add, writeenable, CLK, RESET);

    wire [7:0] twoscomplement;

    TwoS_Complement twoscomplementInstance (regOut2, twoscomplement);

    wire [7:0] mux1_Out;

    MUX mux1 (regOut2, twoscomplement, sub_trigger, mux1_Out);

    wire [7:0] mux2_Out;

    MUX mux2 (mux1_Out, immediate, imm_trigger, mux2_Out);

    alu aluInstance (regOut1, mux2_Out, aluResult, alu_op);

    wire [2:0] adder_Result;

    pc pcInstance (RESET, CLK, adder_Result, PC);

    PC_Adder pcAdderInstance  (adder_Result,PC);

endmodule





