/*
Module  : Instruction Cache 
Author  : Ravindu Mihiranga
Date    : 07/03/2022

*/

`timescale 1ns/100ps

module icache (PC, CLOCK, RESET, imem_READDATA, imem_BUSYWAIT, imem_READ, INSTRUCTION, imem_ADDRESS, iCache_BUSYWAIT);

    // Port Declaration--------------------------------------------------------------------------------------------------------------------------------
    input [31:0] PC;                    
    input CLOCK;
    input RESET;
    input [127:0] imem_READDATA;         
    input imem_BUSYWAIT;  

    output reg imem_READ;              
    output [31:0] INSTRUCTION;          
    output reg [5:0] imem_ADDRESS;      
    output reg iCache_BUSYWAIT;

    //-------------------------------------------------------------------------------------------------------------------------------------------------

    reg [0:0] valid_bit_array [7:0];            // icache valid bit array 8-bits
    reg [2:0] address_tag_array [7:0];          // icache tag array 8x3-bits
    reg [127:0] icache_data_array [7:0];        // icache data array 8x128-bits
    reg [9:0] address;                          // 10 bit address from PC to fetch required instruction from cache

    // registers to store a data for a given operation
    wire [2:0] current_tag;                                   
    reg [127:0] data_block;
    wire valid_bit, hit, comparator_result;

    integer i;

    // Dumping the data arrays to the VCD file
    initial
    begin
        $dumpfile("cpu_wavedata_Grp18CPU.vcd");
        for(i=0;i<8;i++)
            $dumpvars(1,icache_data_array[i]);
        for(i=0;i<8;i++)
        begin
            $dumpvars(1,address_tag_array[i]);
            $dumpvars(1,valid_bit_array[i]);
        end
    end

    //-------------------------------------------------------------------------------------------------------------------------------------------------

    // Reset instruction cache
    always @ (posedge CLOCK, RESET) begin
        if (RESET) begin
            for(i = 0; i < 8; i++) begin
                valid_bit_array[i] = 1'd0;
                address_tag_array[i] = 3'dx;
                icache_data_array[i] = 128'dx;
            end
        end
    end

    always @ (PC) begin
        address = {PC[9:0]};                //  Extracting 10 bit address from PC 
    end

    // CPU should be stalled in order to perform instruction memory read
    always @ (PC) begin
        iCache_BUSYWAIT = 1'b1;
    end

    // extracting required data from arrays for ongoing operations 
    // fetching the data according to the index
    always @(*) begin
        #1
        data_block = icache_data_array[address[6:4]];
    end

    assign #1 current_tag = address_tag_array[address[6:4]];
    assign #1 valid_bit = valid_bit_array[address[6:4]];

     // Tag comparison
    assign #0.9 comparator_result = (current_tag == address[9:7]) ? 1 : 0;

    // deciding the hit status
    assign hit = comparator_result && valid_bit;

    // If it is a hit, CPU should not be stalled. So, cache BUSYWAIT should be de-asserted
    always @(posedge CLOCK, hit) begin
        if (hit) begin
            iCache_BUSYWAIT = 1'b0;
        end
    end

    // if it is a hit then read the INSTRUCTION according to the offset of the given address 
    assign #1 INSTRUCTION = ((address[3:2] == 2'b01) && hit) ? data_block[63:32] :
                            ((address[3:2] == 2'b10) && hit) ? data_block[95:64] :
                            ((address[3:2] == 2'b11) && hit) ? data_block[127:96] : data_block[31:0];

    
    /* Cache Controller FSM Start */

    parameter IDLE = 2'b00, MEM_READ = 2'b01, CACHE_UPDATE = 2'b10;
    reg [1:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if (!hit)  
                    next_state = MEM_READ;          // If it is a 'miss', missing 16-Byte block should be read from the instruction memory
                else
                    next_state = IDLE;              // If it is a 'hit', read instruction from cache
            
            MEM_READ:
                if (imem_BUSYWAIT)
                    next_state = MEM_READ;          // Keep reading 16-Byte block from instruction memory until the memory de-asserts its busywait signal
                else    
                    next_state = CACHE_UPDATE;      // Update cache memory with 16-Byte block read from instruction memory

            CACHE_UPDATE:
                next_state = IDLE;                  // Either update 16-Byte block in cache or read instruction from cache
            
        endcase
    end

    // combinational output logic
    always @(state)
    begin
        case(state)

            // Either update 16-Byte block in cache or read instruction from cache (Without accessing instruction memory)
            IDLE:
            begin

                imem_READ = 0;
                imem_ADDRESS = 6'dx;
                iCache_BUSYWAIT = 0;

            end
         
            // State of fetching required 16-Byte block from memory
            MEM_READ: 
            begin

                imem_READ = 1;                      // Enable 'imem_READ' to send to instruction memory to assert 'busywait' in order to stall the CPU
                imem_ADDRESS = {address[9:4]};      // Derive block address from the address to send to instruction memory

            end
            
            // State of updating cache memory with 16-Byte block read from instruction memory
            CACHE_UPDATE:
            begin

                imem_READ = 0;
                imem_ADDRESS = 6'dx;

                #1
                icache_data_array[address[6:4]] = imem_READDATA;    // Update current block with newly fetched 16-Byte block from instruction memory
                address_tag_array[address[6:4]] = address[9:7];     // Update 'STORE_TAG' array with tag bits corresponding to the address
                valid_bit_array[address[6:4]] = 1'b1;               // Set the newly fetched block from instruction memory as valid


            end

        endcase
    end

    // sequential logic for state transitioning 
    always @(posedge CLOCK, RESET)
    begin
        if(RESET)
            state = IDLE;
        else
            state = next_state;
    end

    /* Cache Controller FSM End */
endmodule