`timescale 1ns/100ps

module dcache (
    clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
    busywait,
    mem_read,
    mem_write,
    mem_address,
    mem_writedata,
    mem_readdata,
    mem_busywait    
);

    input clock;
    input reset;
    input read;                         // memory read signal coming from CPU
    input write;                        // memory write signal coming from CPU
    input [7:0] address;                // memory address coming from ALU
    input [7:0] writedata;              // data coming from register file
    input mem_busywait;                 // Signal coming from data memory indicating memory is busy or not
    input [31:0] mem_readdata;          // Newly fetched data word from memory
    output reg [7:0] readdata;              // Data blocks, read asynchronously according to the offset from the cache to send to register file
    output reg mem_read, mem_write;     // Send mem_read, mem_write signals to data memory indicating memory is busy or not reading & writing
    output reg busywait;                // Send signal to stall the CPU on a memory read/write instruction
    output reg [5:0] mem_address;       // Send block address to data memory to fetch data words
    output reg [31:0] mem_writedata;    // Send data word to write to data memory

    
    /*
    Combinational part for indexing, tag comparison for hit deciding, etc.
    ...
    ...
    */

    reg STORE_VALID [7:0];              // 8 Registers to store 1 bit valid for each data block
    reg STORE_DIRTY [7:0];              // 8 Registers to store 1 bit dirty for each data block
    reg [2:0] STORE_TAG [7:0];          // 8 Registers to store 3 bit tag for each data block
    reg [31:0] STORE_DATA [7:0];        // 8 Registers to store 32 bit data block

    integer i;

    // Reset data cache
    always @ (reset)
    begin
        for(i = 0; i < 8; i++) begin
            STORE_VALID[i] = 1'd0;
            STORE_DIRTY[i] = 1'd0;
            STORE_TAG[i] = 3'dx;  // added 0 instead of x
            STORE_DATA[i] = 32'dx;
        end
    end

    wire VALID, DIRTY;      // To store 1 bit valid & 1 bit dirty bits corresponding to index given by memory address
    wire [2:0] TAG;         // To store 3 bit tag corresponding to index given by memory address
    reg [31:0] DATA;        // To store 32 bit data corresponding to index given by memory address


    // Decide whether CPU should be stalled in order to perform memory read or write
    always @ (read, write)  // add address as well
    begin
        if (read || write) begin
            busywait = 1'b1;
        end else begin
            busywait = 1'b0;
        end
    end

    always @ (*)
    begin
        #1
        DATA = STORE_DATA[address[4:2]];            // Getting 32 bit data corresponding to index given by memory address
    end

    assign #1 VALID = STORE_VALID[address[4:2]];    // Getting valid bit corresponding to index given by memory address
    assign #1 DIRTY = STORE_DIRTY[address[4:2]];    // Getting dirty bit corresponding to index given by memory address
    assign #1 TAG = STORE_TAG[address[4:2]];        // Getting tag 3 bits corresponding to index given by memory address


    wire COMPARATORSIGNAL;  // To store whether tag bits in corresponding index & tag bits given by memory address matches
    wire HITSIGNAL;         // To store whether a 'hit' or a 'miss'


    // Getting whether tag bits in corresponding index & tag bits given by memory address matches
    assign #0.9 COMPARATORSIGNAL = (TAG == address[7:5]) ? 1 : 0;


    // If tag bits given by memory address matches with tag bits in corresponding index of cache memory & if it is a valid data block, it is a 'hit'(1)
    // If tag bits given by memory address mismatches with tag bits in corresponding index of cache memory or if it is not a valid data block, it is a 'miss'(0)
    assign HITSIGNAL = COMPARATORSIGNAL && VALID;

    // If it is a hit, CPU should not be stalled. So, mem_busywait should be de-asserted
    always @ (posedge clock)
    if (HITSIGNAL) begin
        busywait = 1'b0;
    end
//--------------------------------------------------------------------------//
always@(address,read,HITSIGNAL,DATA) begin
    if(read && HITSIGNAL) begin
    case(address[1:0]) 
        2'b00 : readdata = DATA[7:0];
        2'b01 : readdata = DATA[15:8];
        2'b10 : readdata = DATA[23:16];
        2'b11 : readdata = DATA[31:24];
    endcase
    end
end

//--------------------------------------------------------------------------//
  /*  assign #1 readdata =  ((address[1:0] == 2'b01) && read && HITSIGNAL) ? DATA[15:8] :
                            ((address[1:0] == 2'b10) && read && HITSIGNAL) ? DATA[23:16] :
                            ((address[1:0] == 2'b11) && read && HITSIGNAL) ? DATA[31:24] :
                            ((address[1:0] == 2'b00) && read && HITSIGNAL) ? DATA[7:0] :DATA[15:8] ;*/
                          
//-----------------------------------------------------------------//



    // Writing data blocks to the cache if it is a 'hit' according to the offset
    always @ (posedge clock)
    begin
        if (HITSIGNAL && write) begin
            #1;
            STORE_DIRTY[address[4:2]] = 1'b1;       // dirty bit of the index is set as data block in cache is updated with data coming from register file (indicate that the block of data is inconsistant)

            if (address[1:0] == 2'b00) begin
                STORE_DATA[address[4:2]][7:0] = writedata;
            end else if (address[1:0] == 2'b01) begin
                STORE_DATA[address[4:2]][15:8] = writedata;
            end else if (address[1:0] == 2'b10) begin
                STORE_DATA[address[4:2]][23:16] = writedata;
            end else if (address[1:0] == 2'b11) begin
                STORE_DATA[address[4:2]][31:24] = writedata;
            end
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
                if ((read || write) && !DIRTY && !HITSIGNAL)  
                    next_state = MEM_READ;          // If it is a 'miss' and the block isn’t dirty, the missing data block should be read from the memory
                else if ((read || write) && DIRTY && !HITSIGNAL)
                    next_state = MEM_WRITE;         // If it is a 'miss' and the block is dirty, that block must be written back to the memory
                else
                    next_state = IDLE;              // If it is a 'hit', either update data block in cache or read data from cache
            
            MEM_READ:
                if (mem_busywait)
                    next_state = MEM_READ;          // Keep reading whole data word from memory until the memory de-asserts its busywait signal
                else    
                    next_state = CACHE_UPDATE;      // Update cache memory with data word read from data memory

            MEM_WRITE:
                if (mem_busywait)
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

                mem_read = 0;
                mem_write = 0;
                mem_address = 6'dx;
                mem_writedata = 32'dx;
                busywait = 0;

            end
         
            // State of fetching required data word from memory
            MEM_READ: 
            begin

                mem_read = 1;                       // Enable 'mem_read' to send to data memory to assert 'mem_busywait' in order to stall the CPU
                mem_write = 0;
                mem_address = {address[7:2]};       // Derive block address from the address coming from ALU to send to data memory
                mem_writedata = 32'dx;

            end
            
            // State of writing data to the memory
            MEM_WRITE: 
            begin

                mem_read = 0;
                mem_write = 1;                      // Enable 'mem_write' to send to data memory to assert 'mem_busywait' in order to stall the CPU
                mem_address = {TAG,address[4:2]};   // Derive block address to send to data memory to store a existing cache data word
                mem_writedata = DATA;               // Getting existing cache data word corresponding to index

            end

            // State of updating cache memory with data word read from data memory
            CACHE_UPDATE:
            begin

                mem_read = 0;
                mem_write = 0;
                mem_address = 6'dx;
                mem_writedata = 32'dx;

                #1
                STORE_DATA[address[4:2]] = mem_readdata;    // Update current cache data word with newly fetched data from memory
                STORE_TAG[address[4:2]] = address[7:5];     // Update 'STORE_TAG' array with tag bits corresponding to the address
                STORE_VALID[address[4:2]] = 1'b1;           // Set the newly fetched data from memory as valid
                STORE_DIRTY[address[4:2]] = 1'b0;           // Set that newly fetched data is consistant with the data word in memory

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