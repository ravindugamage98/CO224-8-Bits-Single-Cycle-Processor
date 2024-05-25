//******************************************************************************************************************************//
//************************Register Module for the 8-8bit registers by Group 18 of CO224 - Lab 05 part 2*************************//
//******************************************************************************************************************************//

module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET);//The Module

    input [7:0] IN;/*Input port "IN" is declared as a wire, this will take the input bits for the register file to write in
                    one of it's registers*/
    output  [7:0] OUT1,OUT2; /* Output ports "OUT1, OUT2" are declared as wires, these two will take the output which will 
                                contain register values specified by OUT1ADDRESS and OUT2ADDRESS respectively*/

    input [2:0] INADDRESS,OUT1ADDRESS,OUT2ADDRESS; /* Input ports "INADDRESS, OUT1ADDRESS, OUT2ADDRESS" are declared as wires
                                                    which will specify which registers to be written on/to be read from*/

    input WRITE;//this input wire will go high when writing needs to happen
    input CLK,RESET; // CLK: clock for synchronising and RESET for clearing all the registers at once

    reg [7:0] registers [7:0];// array of registers( 8,8bit registers) to work as registers in register file

//////////////////////////////////////////////////This will Clearing(Reseting) all the registers
    integer count;//count interger for the for loop
    always@(posedge CLK ) //triger this block everytime a postive edge arrises
    begin
        #1;//wait a one time unit - artificial delays to simulate the delays in a circuit
        if(RESET) begin //if RESET is high
            for(count =0;count<8;++count) begin
                registers[count] <= 8'b000_000_00;//set all the registers to zero at the end all at once
            end
        end 
    end


//////////////////////////////////////////////// This will read from registers mentioned in OUT1ADDRESS and OUT2ADDRESS
                                             /// and copy that register value to OUT1 and OUT2 respectively
    assign #2 OUT1 = registers[OUT1ADDRESS];// 2 time unis of delay to simulate circuit delays
    assign #2 OUT2 = registers[OUT2ADDRESS];// 2 time unis of delay to simulate circuit delays
    

//////////////////////////////////////////////// This will put wite to the register mentioned in INADDRESS the value in IN 
    always @(posedge CLK) // triger this block when the positive edge of the clock arises 
        begin
            #1;// 1 time uni of delay to simulate circuit delays
            if(WRITE)//if WRITE is high
            registers[INADDRESS] <= IN;//copy IN value to the register specified by INADDRESS
            
        end

    //This is to get the register values in the vcd file : Commented out for now:
    
    /* initial begin
        $dumpfile("reg_file_wavedata_G18.vcd");
        for(count=0;count<8;++count)
            $dumpvars(1,registers[count]);
    end
 */
endmodule//this is the end of the module

