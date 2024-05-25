/*
    Memory Address:
    <-TAG BITS-><-CACHE ID-><OFFSET->
    | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
*/
module cache_controller (clock, reset, read, mem_read, write, mem_write, address, mem_address, writedata, mem_writedata, readdata, mem_readdata, busywait, mem_busywait);
    // Port declaration
    input clock, reset, read, write, mem_busywait;
    input [7:0] address, writedata;
    input [31:0] mem_readdata;
    output reg busywait, mem_read, mem_write;
    output reg [5:0] mem_address;
    output reg [7:0] readdata;
    output reg [31:0] mem_writedata;

    // hit: Temporary signal to store whether the cache access is a hit or a miss
    // comparator: Temporary signal from the comparator which compares the tag bits of the indexed cache Line and requested address by the CPU
    // writeaccess: The temporary signal to indicate that the writedata should be written into the cache
    reg hit, comparator, writeaccess;
    // cache: vector of 8, 37-bit long vectors for the cache memory
    // 3 most significant bits store the tag for the particular Line
    // 32nd and 33rd bits store the dirty bit and the valid bit for the particular Line respectively
    // line: The current indexed cache line bus
    reg [36:0] cache [7:0], line;
    // The data word in the selected cache line corresponding to the block offset
    reg [7:0] readword;

    // cache_index: The bus which carries the index of the cache line
    // tag_bits: The bus which carries the tag bits of the CPU request
    wire [2:0] cache_index, tag_bits;
    // block offset * 8
    wire [4:0] word_index;

    // Splitting the address
    assign word_index = {address[1:0], 3'b000};                 // Multiplication by 8 to get the starting index of the data word
    assign cache_index = address[4:2];
    assign tag_bits = address[7:5];

    // Generating the busywait signal
    always @ (posedge read, posedge write)
        busywait = read || write;

    always @ (line, tag_bits) begin
        // The comparator which compares the requested tag bits with the tag bits of the indexed cache line
        #0.9 comparator = tag_bits == line[36:34];
        // The hit / miss signal
        hit = line[33] && comparator;
    end

    always @ (posedge busywait, negedge mem_busywait) begin
        if (busywait) begin
            #1 // Wait for the address to be settled
            // Extracting the cache line by indexing the cache memory
            #1 line = cache[cache_index];
            // MUX to read select the requested data word from the cache memory by the requested block offset (address[1:0]) as the control signal
            #1 readword = line[word_index +: 8];
        end

        // If the request is not a data memory fetch
        if (!mem_read) begin
            // If the cache access is a hit
            if (hit) begin
                // If the cache access is a read-hit
                if (read) begin
                    readdata = readword;                        // Direct the readword bus to the readdata port
                    busywait = 0;                               // No need to stall the CPU. Set the busywait signal to low
                end
                // If the cache access is a write-hit
                if (write) begin
                    writeaccess = 1;                            // The requested data has to be written to the cache memory on the next positive clock edge
                    busywait = 0;                               // No need to stall the CPU. Set the busywait signal to low
                end
            end
            // If the cache access is a miss and the dirty bit is high
            else if (line[32]) begin
                mem_write = 1;                                  // A write-back has to be done prior to the data memory fetch
                mem_address = {line[36:34], cache_index};       // Direct the corresponding memory address of the block to the mem_address port
                mem_writedata = line[31:0];                     // Direct the cache data block to the mem_writedata port
            end
            // If the cache access is a miss and the dirty bit is low
            else begin
                mem_read = 1;                                   // The missing block has to be fetched from the data memory
                mem_address = address[7:2];                     // Set the last 6 bits of the requested address to the mem_address port
            end
        end
    end

    // Data memory read, write-back and writting to the cache
    always @ (posedge clock) begin
        if (writeaccess) begin
            // DEMUX to select the word in the cache line to be written into by the requested block offset (address[1:0]) as the control signal
            #1 cache[cache_index][word_index +: 8] = writedata;
            // Mark the line as dirty
            cache[cache_index][32] = 1;
            // Set the writeaccess signal to low
            writeaccess = 0;
        end
    end

    // After a writeback or a memory read has been completed
    always @ (negedge mem_busywait) begin
        // If the previous action was a writeback
        if (mem_write) begin
            mem_write = 0;
            // If the current access is a miss with dirty bit high, the next action should be a data memory fetch
            if ((read || write) && !hit && line[32]) begin
                mem_read = 1;
                mem_address = address[7:2];
            end
        end
        // If the previous action was a data memory fetch
        else if (mem_read) begin
            mem_read = 0;
            // Write the fetched data into the particular indexed line of the cache
            #1 cache[cache_index] = {tag_bits, 2'b10, mem_readdata};
        end
    end

    integer i;

    always @ (posedge clock) begin
        if (reset) begin
            // Set the cache lines to be invalid
            for (i = 0; i < 8; i = i + 1)
                cache[i][33] = 0;
            busywait = 0;
            mem_read = 0;
            mem_write = 0;
            writeaccess = 0;
        end
    end

    // initial begin
    //     $dumpfile("cpu_ext.vcd");
    //     $dumpvars(0, cache[0], cache[1], cache[2], cache[3], cache[4], cache[5], cache[6], cache[7]);
    // end
endmodule
