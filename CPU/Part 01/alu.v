// Stimulus block
module testbench;

    // Declaring the connection variables 
    reg [7:0] data1, data2;
    wire [7:0] r;
    reg [2:0] sel;

    //Created an instance from the alu module
    alu alu1(data1,data2,r,sel);

    initial
    begin

        // Printing the result whenever we change the input
        $monitor("data1 = %b, data2 = %b     result=%b",data1,data2,r);
        $dumpfile("wavedata_grp18_aluOutput.vcd");
        $dumpvars(0,testbench);

        $display("Forward Operation : ");
        data1 = 8'b00_000_001;
        data2 = 8'b00_001_011;
        sel = 3'b000;
        
        #5;
        $display("ADD Operation : ");
        data1 = 8'b00_000_001;
        data2 = 8'b00_001_110;
        sel = 3'b001;

        #5;
        $display("Bitwise AND Operation : ");
        data1 = 8'b00_000_001;
        data2 = 8'b00_000_010;
        sel = 3'b010;

        #5;
        $display("Bitwise OR Operation : ");
        data1 = 8'b00_001_001;
        data2 = 8'b00_001_111;
        sel = 3'b011;
       
        
    end
endmodule

//*********************************************************** 8-bit ALU *****************************************************************//
// 8-bit ALU module
module alu (DATA1, DATA2, RESULT, SELECT);

    // Port Declaration
    input [7:0] DATA1, DATA2;
    input [2:0] SELECT;
    output reg [7:0] RESULT;
    wire [7:0] fwdResult, addResult, andResult, orResult;

    // Instantiating the modules
    FWD fwd1(DATA2, fwdResult);                          // Forward Function
    ADD add1(DATA1, DATA2, addResult);                   // ADD function
    AND and1(DATA1, DATA2, andResult);                   // Bitwise AND function
    OR or1(DATA1, DATA2, orResult);                      // Bitwise OR function

    // This always block will execute whenever we change the DATA1, DATA2 & SELECT inputs
    always @(fwdResult, addResult, andResult, orResult) begin
    // Case structure to simulate the mux's operations
    case (SELECT)
        3'b000 :  RESULT = fwdResult;
        3'b001 :  RESULT = addResult;
        3'b010 :  RESULT = andResult;
        3'b011 :  RESULT = orResult;
        default:  RESULT = 8'b00000000;
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