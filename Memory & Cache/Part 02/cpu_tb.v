`include "CPU_Lab6_Part2.v"
`include "DataMemory_LAB06_Part2.v"
`include "dcacheFSM_Grp18.v"
`timescale 1ns/100ps
module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;

    wire [7:0] aluResult,regOut1,readData;// new wires to connect with Data memory
    wire read,write, busyWait;

    wire mem_Read,mem_Write,mem_busywait;
    wire [5:0] mem_Address;
    wire  [31:0]   mem_Writedata;
	wire  [31:0]	mem_readdata;
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    //  Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    reg [7:0 ]instr_mem [1023:0];
    
    //  Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    always @(PC) begin
       #2 INSTRUCTION = {instr_mem[PC+2'b11],instr_mem[PC+2'b10],instr_mem[PC+1'b1],instr_mem[PC]};
    end
    
    
    initial //instruction read from .mem file
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        //{instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        //{instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        //{instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    
     
    //------------------------------
    // CPU
    //------------------------------
    
    CPU mycpu(PC, INSTRUCTION, CLK, RESET,aluResult,regOut1,readData,read,write,busyWait);
    //module CPU (PC, INSTRUCTION, CLK, RESET, aluResult, regOut1, readdata, read, write, busywait);

    dcache mydata_cache(CLK,RESET,read,write,aluResult,regOut1,readData,busyWait,mem_Read,mem_Write,mem_Address,mem_Writedata,mem_readdata,mem_busywait);
    //module dcache (CLOCK, RESET, READ, WRITE, ADDRESS, WRIITEDATA, READDATA, BUSYWAIT, Mem_READ, Mem_WRITE, Mem_ADDRESS, Mem_WRITEDATA, Mem_READDATA, Mem_BUSYWAIT);

    data_memory my_data_memory(CLK,RESET,mem_Read,mem_Write,mem_Address,mem_Writedata,mem_readdata,mem_busywait);
    //module data_memory(clock,reset,read,write,address,writedata,readData,busyWait);

    initial // clock and start and reset and dump to .vcd
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata_Grp18CPU.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        RESET = 1'b1;
        #10;
        RESET = 1'b0;
        // finish simulation after some time
        #1500
        $finish;
        
    end

    
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule