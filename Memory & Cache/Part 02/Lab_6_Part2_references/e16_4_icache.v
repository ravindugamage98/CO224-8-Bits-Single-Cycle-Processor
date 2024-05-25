/*
Author	: D.L.C. Amarasinghe (E/16/022)
Date	: 21-June-2020
*/

`timescale 1ns/100ps

module instcache(
    PC,
    clock,
    reset,
    inst_readdata,
    inst_busywait,
    inst_read,
    instruction,
    inst_address,
    busywait
);


    input [31:0] PC;                    // Indicate what instruction should be fetched
    input clock;
    input reset;
    input [127:0] inst_readdata;        // Newly fetched 16-Byte block from the instruction memory
    input inst_busywait;                // Signal coming from instruction memory indicating instruction memory is busy or not
    output reg inst_read;               // Send inst_read signal indicating that instruction memory is busy or not with reading
    output [31:0] instruction;          // Data blocks, read asynchronously according to the offset from the cache to send to CPU
    output reg [5:0] inst_address;      // Send block address to instruction memory to fetch 16-Byte block from the instruction memory
    output reg busywait;                // Send signal to stall the CPU when reading instruction memory


    reg STORE_VALID [7:0];              // 8 Registers to store 1 bit valid for each data block
    reg [2:0] STORE_TAG [7:0];          // 8 Registers to store 3 bit tag along with every data block
    reg [127:0] STORE_DATA [7:0];       // 8 Registers to store 128 bit data block (16 Bytes data block)
    reg [9:0] address;                  // 10 bit address from PC to fetch required instruction from cache

    integer i;

    // Reset instruction cache
    always @ (reset)
    begin
        for(i = 0; i < 8; i++) begin
            STORE_VALID[i] = 1'd0;
            STORE_TAG[i] = 3'dx;
            STORE_DATA[i] = 32'dx;
        end
    end

    always @ (PC)
    begin
        if (PC != -4) begin
            address = {PC[9:0]};    // Getting 10 bit address from PC to fetch required instruction from cache
        end
    end

    wire VALID;         // To store 1 bit valid bit corresponding to index given by address
    wire [2:0] TAG;     // To store 3 bit tag corresponding to index given by address
    reg [127:0] DATA;   // To store 128 bit data corresponding to index given by address


    // Decide whether CPU should be stalled in order to perform instruction memory read
    always @ (PC)
    begin
        if (PC == -4) begin
            busywait = 1'b0;
        end else begin
            busywait = 1'b1;
        end
    end

    always @ (*)
    begin
        #1
        DATA = STORE_DATA[address[6:4]];            // Getting 128 bit block corresponding to index given by address
    end

    assign #1 VALID = STORE_VALID[address[6:4]];    // Getting valid bit corresponding to index given by address
    assign #1 TAG = STORE_TAG[address[6:4]];        // Getting tag 3 bits corresponding to index given by address


    wire COMPARATORSIGNAL;  // To store whether tag bits in corresponding index & tag bits given by address matches
    wire HITSIGNAL;         // To store whether a 'hit' or a 'miss'


    // Getting whether tag bits in corresponding index & tag bits given by address matches
    assign #0.9 COMPARATORSIGNAL = (TAG == address[9:7]) ? 1 : 0;


    // If tag bits given by address matches with tag bits in corresponding index of cache memory & if it is a valid data block, it is a 'hit'(1)
    // If tag bits given by address mismatches with tag bits in corresponding index of cache memory or if it is not a valid data block, it is a 'miss'(0)
    assign HITSIGNAL = COMPARATORSIGNAL && VALID;

    // If it is a hit, CPU should not be stalled. So, inst_busywait should be de-asserted
    always @ (posedge clock)
    if (HITSIGNAL) begin
        busywait = 1'b0;
    end


    // Reading instruction asynchronously from the cache to send to CPU according to the offset, if it is a hit
    assign #1 instruction = ((address[3:2] == 2'b01) && HITSIGNAL) ? DATA[63:32] :
                         ((address[3:2] == 2'b10) && HITSIGNAL) ? DATA[95:64] :
                         ((address[3:2] == 2'b11) && HITSIGNAL) ? DATA[127:96] : DATA[31:0];
    


    /* Cache Controller FSM Start */

    parameter IDLE = 2'b00, MEM_READ = 2'b01, CACHE_UPDATE = 2'b10;
    reg [1:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if (!HITSIGNAL)  
                    next_state = MEM_READ;          // If it is a 'miss', missing 16-Byte block should be read from the instruction memory
                else
                    next_state = IDLE;              // If it is a 'hit', read instruction from cache
            
            MEM_READ:
                if (inst_busywait)
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

                inst_read = 0;
                inst_address = 6'dx;
                busywait = 0;

            end
         
            // State of fetching required 16-Byte block from memory
            MEM_READ: 
            begin

                inst_read = 1;                      // Enable 'inst_read' to send to instruction memory to assert 'busywait' in order to stall the CPU
                inst_address = {address[9:4]};      // Derive block address from the address to send to instruction memory

            end
            
            // State of updating cache memory with 16-Byte block read from instruction memory
            CACHE_UPDATE:
            begin

                inst_read = 0;
                inst_address = 6'dx;

                #1
                STORE_DATA[address[6:4]] = inst_readdata;   // Update current block with newly fetched 16-Byte block from instruction memory
                STORE_TAG[address[6:4]] = address[9:7];     // Update 'STORE_TAG' array with tag bits corresponding to the address
                STORE_VALID[address[6:4]] = 1'b1;           // Set the newly fetched block from instruction memory as valid


            end

        endcase
    end

    // sequential logic for state transitioning 
    always @(posedge clock, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

    /* Cache Controller FSM End */

endmodule