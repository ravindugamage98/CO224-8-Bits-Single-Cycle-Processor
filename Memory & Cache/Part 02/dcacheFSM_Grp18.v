/*
Module  : Data Cache 
Author  : Ridma Jayasundara, Ravindu Mihiranga
Date    : 05/03/2022

Description	: 

This file presents a skeleton implementation of the cache controller using a Finite State Machine model. Note that this code is not complete.
*/
`timescale 1ns/100ps

module dcache (CLOCK, RESET, READ, WRITE, ADDRESS, WRIITEDATA, READDATA, BUSYWAIT, Mem_READ, Mem_WRITE, Mem_ADDRESS, Mem_WRITEDATA, Mem_READDATA, Mem_BUSYWAIT);

    // Port Declaration--------------------------------------------------------------------------------------------------------------------------------
    input CLOCK, READ, WRITE, RESET;
    input [7:0] WRIITEDATA;
    input [7:0] ADDRESS;

    output reg BUSYWAIT;
    output reg [7:0] READDATA;

    input Mem_BUSYWAIT;
    input [31:0] Mem_READDATA;

    output reg Mem_READ, Mem_WRITE;
    output reg [31:0] Mem_WRITEDATA;
    output reg [5:0] Mem_ADDRESS;

    //-------------------------------------------------------------------------------------------------------------------------------------------------

    reg [31:0] cache_data_array [7:0];                  // cache data array 8x32-bits                      
    reg [2:0] address_tag_array [7:0];                  // cache tag array 8x3-bits 
    reg valid_bit_array [7:0];                          // cache valid bit array 8-bits
    reg dirty_bit_array [7:0];                          // cache dirty array 8-bits

    // registers to store a data for a given operation
    wire [2:0] current_tag;                                   
    reg [31:0] data_block;
    wire valid_bit, dirty_bit, hit;

    reg readaccess,writeaccess;

    integer i;
    wire comparator_result;

    // Combinational part for indexing, tag comparison for hit deciding, etc.-------------------------------------------------------------------------- 

    //assert cache busywait
    always @ (READ, WRITE, ADDRESS) begin
        if (READ || WRITE) begin
            BUSYWAIT = 1'b1;
            readaccess =1'b1;
            writeaccess=1'b1;

        end else begin
            BUSYWAIT = 1'b0;
        end
    end

    // Reset data cache
    always @ (RESET) begin
        for(i = 0; i < 8; i++) begin
            valid_bit_array[i] = 1'd0;
            dirty_bit_array[i] = 1'd0;
            address_tag_array[i] = 3'dx;
            cache_data_array[i] = 32'dx;
        end
    end

    // extracting required data from arrays for ongoing operations 
    always @(*) begin
        #1
        data_block = cache_data_array[ADDRESS[4:2]];
    end

    assign #1 current_tag = address_tag_array[ADDRESS[4:2]];
    assign #1 valid_bit = valid_bit_array[ADDRESS[4:2]];
    assign #1 dirty_bit = dirty_bit_array[ADDRESS[4:2]];

    // Tag comparison
    assign #0.9 comparator_result = (current_tag == ADDRESS[7:5]) ? 1 : 0;

    // deciding the hit status
    assign hit = comparator_result && valid_bit;

    // If it is a hit, CPU should not be stalled. So, cache BUSYWAIT should be de-asserted
    always @(posedge CLOCK) begin
        if (hit) begin
            BUSYWAIT = 1'b0;
        end
    end

    //--------------------------------------------------------------------------//
    always @(ADDRESS, readaccess, hit, data_block) begin
        if (readaccess && hit) begin
            case(ADDRESS[1:0]) 
                2'b00 : READDATA = data_block[7:0];
                2'b01 : READDATA = data_block[15:8];
                2'b10 : READDATA = data_block[23:16];
                2'b11 : READDATA = data_block[31:24];
            endcase
            readaccess =1'b0;
        end
    end

    // Writing data blocks to the cache if it is a 'hit' according to the offset
    always @ (posedge CLOCK) begin
        if (hit && writeaccess) begin
            #1;
            dirty_bit_array[ADDRESS[4:2]] = 1'b1;       // dirty bit of the index is set to high to indicate that the block of data is inconsistant

            case (ADDRESS[1:0])
                2'b00 : cache_data_array[ADDRESS[4:2]][7:0] = WRIITEDATA;
                2'b01 : cache_data_array[ADDRESS[4:2]][15:8] = WRIITEDATA;
                2'b10 : cache_data_array[ADDRESS[4:2]][23:16] = WRIITEDATA;
                2'b11 : cache_data_array[ADDRESS[4:2]][31:24] = WRIITEDATA;
            endcase
            writeaccess=1'b0;
        end
    end

    /* Cache Controller FSM Start */

    parameter IDLE = 2'b00, MEM_READ = 2'b01, MEM_WRITE = 2'b10, CACHE_UPDATE = 2'b11;
    reg [1:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if ((READ || WRITE) && !dirty_bit && !hit)  
                    next_state = MEM_READ;          // If it is a 'miss' and the block isn’t dirty, the missing data block should be READ from the memory
                else if ((READ || WRITE) && dirty_bit && !hit)
                    next_state = MEM_WRITE;         // If it is a 'miss' and the block is dirty, that block must be written back to the memory
                else
                    next_state = IDLE;              // If it is a 'hit', either update data block in cache or read data from cache
            
            MEM_READ:
                if (Mem_BUSYWAIT)
                    next_state = MEM_READ;          // Keep reading whole data word from memory until the memory de-asserts its busywait signal
                else    
                    next_state = CACHE_UPDATE;      // Update cache memory with data word read from data memory

            MEM_WRITE:
                if (Mem_BUSYWAIT)
                    next_state = MEM_WRITE;         // Keep writing data to the memory until the memory de-asserts its busywait signal
                else    
                    next_state = MEM_READ;          // Fetch required data word from memory

            CACHE_UPDATE:
                next_state = IDLE;                  // Either update data block in cache or read data from cache
            
        endcase
    end

    // combinational output logic
    always @(state)
    begin
        case(state)

            // Either update data block in cache or read data from cache (Without accessing data memory)
            IDLE:
            begin

                Mem_READ = 0;
                Mem_WRITE = 0;
                Mem_ADDRESS = 6'dx;
                Mem_WRITEDATA = 32'dx;
                BUSYWAIT = 0;

            end
         
            // State of fetching required data word from memory
            MEM_READ: 
            begin

                Mem_READ = 1;                       // Enable 'mem_read' to send to data memory to assert 'mem_busywait' in order to stall the CPU
                Mem_WRITE = 0;
                Mem_ADDRESS = {ADDRESS[7:2]};       // Derive block address from the address coming from ALU to send to data memory
                Mem_WRITEDATA = 32'dx;

            end
            
            // State of writing data to the memory
            MEM_WRITE: 
            begin

                Mem_READ = 0;
                Mem_WRITE = 1;                              // Enable 'mem_write' to send to data memory to assert 'mem_busywait' in order to stall the CPU
                Mem_ADDRESS = {current_tag,ADDRESS[4:2]};   // Derive block address to send to data memory to store a existing cache data word
                Mem_WRITEDATA = data_block;                 // Getting existing cache data word corresponding to index

            end

            // State of updating cache memory with data word read from data memory
            CACHE_UPDATE:
            begin

                Mem_READ = 0;
                Mem_WRITE = 0;
                Mem_ADDRESS = 6'dx;
                Mem_WRITEDATA = 32'dx;

                #1
                cache_data_array[ADDRESS[4:2]] = Mem_READDATA;      // Update current cache data word with newly fetched data from memory
                address_tag_array[ADDRESS[4:2]] = ADDRESS[7:5];     // Update 'STORE_TAG' array with tag bits corresponding to the address
                valid_bit_array[ADDRESS[4:2]] = 1'b1;               // Set the newly fetched data from memory as valid
                dirty_bit_array[ADDRESS[4:2]] = 1'b0;               // Set that newly fetched data is consistant with the data word in memory

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
