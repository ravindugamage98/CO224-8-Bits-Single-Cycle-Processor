//Testbech for the 8-8bit registers by Group 18 of CO224 - Lab 05 part 2
`include "registers.v"

module testbench;
reg [7:0] in;
reg [2:0] inaddress,out1address,out2address;
reg write,clk,reset;
wire [7:0] out1,out2;

reg_file r1(in,out1,out2,inaddress,out1address,out2address,write,clk,reset);//Module in a seperate .v file

initial begin//monitor changes in all the wires and registers mentioned above
    $monitor("in= %b, out1= %b, out2= %b,inaddress= %b,out1address= %b,out2address= %b,write= %b,clk= %b,reset= %b",in,out1,out2,inaddress,out1address,out2address,write,clk,reset);
    $dumpfile("reg_file_wavedata_G18.vcd");//put that signal changes in .vcd file 
    $dumpvars(0,r1);
end

initial begin // reading walata module eka athule delay eka 2 **************************writing and reset 1
    clk = 1'b0;
    in = 8'b111_111_11; //some value to store in a register
    inaddress =3'b000;// that register is specified here
    write = 1'b1;//write is high
    
    #2;
    $display("-------------copied value 111_111_11 to reg 0");
    $display("--------------------------------------");
   
    out1address <= 3'b000;//specifing which register to be retrieved from
    write = 1'b0;//write is low
    #2;
    $display("-------------took reg0 value to out1");
    $display("--------------------------------------");

    in = 8'b000_000_11; //some value to store in a register
    inaddress =3'b010; // that register is specified here
    write = 1'b1;//write is high
    #2;
    $display("-------------copied value 000_000_11 to reg 2");
    $display("--------------------------------------");

    out2address = 3'b010;//specifing which register to be retrieved from
    write = 1'b0;//write is low
    #2;
    $display("-------------took reg2 value to out2");
    $display("--------------------------------------");


    reset=1'b1; //reset is high:
    #2;
    $display("-------------all register reseted");
    $display("--------------------------------------");
    reset =1'b0;
    #1;
    
  
    out1address = 3'b000;//to check whether all the registers have been reseted set outaddresses to some registers
    out2address = 3'b010;
    #1;
    $display("-------------displayed OUT1 and OUT2");
    $display("--------------------------------------");
    $finish;

end
always #1 clk=~clk;//clock for synchronising 
endmodule 