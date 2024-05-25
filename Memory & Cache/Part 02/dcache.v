
`timescale 1ns/100ps

module dcache (clock,reset,read,write,address,writedata,readdata,busywait,mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busywait);

    // Port Declaration
    input clock;
    input reset;
    input read;                         
    input write;                       
    input [7:0] address;                
    input [7:0] writedata;
    output reg [7:0] readdata; 
    output reg busywait;

    output reg mem_read, mem_write;  
    output reg [5:0] mem_address; 
    output reg [31:0] mem_writedata; 
    input [31:0] mem_readdata;   
    input mem_busywait;    
                 
           
    /*Combinational part for indexing, tag comparison for hit deciding...*/

    //arrays to store the cache values including valid and dirty 
    reg valid_array [7:0];                              // cache valid bit array 
    reg dirty_array [7:0];                              // cache dirty bit array
    reg [2:0] tag_array [7:0];                          // cache tag array 8x3-bits 
    reg [31:0] cache_data_array [7:0];                  // cache data array 8x32-bits

    wire valid, dirty;                                  // valid &  dirty bits corresponding to index 
    wire [2:0] tag;                                     // 3 bit tag corresponding to index 
    reg [31:0] data;                                    // 32 bit data corresponding to index
    wire [2:0] index;                                   // index of a given data address

    reg readaccess,writeaccess;

    assign index = address[4:2];

    // when read or write is high PC needs to be stalled
    always @ (read, write,address)
    begin
        if (read || write) begin
            busywait = 1'b1;
            readaccess =1'b1;
            writeaccess=1'b1;
            //#1;
        end
    end

    always @ (*)
    begin
        #1
        data = cache_data_array[index];                        // fetching the relevant data from the cache
    end
    // fetching the data from the cache according to the index
    assign #1 valid = valid_array[index];                       // respective valid bit is taken
    assign #1 dirty = dirty_array[index];                       // respective dirty bit is taken
    assign #1 tag = tag_array[index];                           // respective tag is taken     


    wire tagCompare; 
    wire hit;                                                   // to store whether it is a hit or a miss


    //tag comparison
    assign #0.9 tagCompare = (tag == address[7:5]) ? 1 : 0;


    //checking for a hit or a miss
    assign hit = tagCompare && valid;

    // If it is a hit, PC should not be stalled. So, mem_busywait should be de-asserted
    always @ (hit,posedge clock)
    if (hit) begin
        busywait = 1'b0;
    end

always@(address,readaccess,hit,data) begin
    #1;
    if(readaccess && hit) begin                              //when it is a hit, readdata is filled with cache data
    case(address[1:0]) 
        2'b00 : readdata = data[7:0];
        2'b01 : readdata = data[15:8];
        2'b10 : readdata = data[23:16];
        2'b11 : readdata = data[31:24];
    endcase
    
    readaccess=1'b0;
    end
end

    // Writing data blocks to the cache if it is a 'hit' according to the offset
    always @ (posedge clock)
    begin
        if (hit && writeaccess) begin
            #1;
            dirty_array[index] = 1'b1;                          // dirty bit of the index is set to high to indicate that the block of data is inconsistant     

            if (address[1:0] == 2'b00) 
            begin
                cache_data_array[index][7:0] = writedata;
            end 
            else if (address[1:0] == 2'b01) 
            begin
                cache_data_array[index][15:8] = writedata;
            end 
            else if (address[1:0] == 2'b10) 
            begin
                cache_data_array[index][23:16] = writedata;
            end 
            else if (address[1:0] == 2'b11) 
            begin
                cache_data_array[index][31:24] = writedata;
            end
            writeaccess=1'b0;
        end
        
    end

    integer i;
        // Reset data cache
    always @ (posedge clock) begin
    if(reset)
    begin
        for(i = 0; i < 8; i++) begin
            valid_array[i] = 1'b0;
            dirty_array[i] = 1'b0;
            tag_array[i] = 3'bx;
            cache_data_array[i] = 32'bx;
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
                if ((read || write) && !dirty && !hit)//if it is not dirty and is miss 
                    next_state = MEM_READ;          
                else if ((read || write) && dirty && !hit)//if dirty and miss
                    next_state = MEM_WRITE;        
                else
                    next_state = IDLE;              
            
            MEM_READ:
                if (mem_busywait)
                    next_state = MEM_READ;          
                else    
                    next_state = CACHE_UPDATE; //after mem read it updates the cache    

            MEM_WRITE:
                if (mem_busywait)
                    next_state = MEM_WRITE;         
                else    
                    next_state = MEM_READ;  //afer write, then mem read        

            CACHE_UPDATE:
                next_state = IDLE;  //after being written to cache, next idle                
            
        endcase
    end

    // combinational output logic
    always @(state)
    begin
        case(state)    
            IDLE:
            begin
                mem_read = 0;//when idle, mem read is low
                mem_write = 0;//mem write is also low
                mem_address = 6'bx;//there is no address to read from mem
                mem_writedata = 32'bx;// nothing to be written
                busywait = 0;//lower busywait
            end
           
            MEM_READ: 
            begin
                mem_read = 1;//set memory read high                    
                mem_write = 0;//set memory write low
                mem_address = {address[7:2]};  //address to be  read is given     
                mem_writedata = 32'bx;//nothing to be written
            end       
            
            MEM_WRITE: 
            begin
                mem_read = 0;//when its writting to memory, mem read is low
                mem_write = 1;// mem write is high                  
                mem_address = {tag,index}; //address to be written is given
                mem_writedata = data; // what is to be written is also given      
            end

        
            CACHE_UPDATE:
            begin
                mem_read = 0;//in cache update state, mem read is low
                mem_write = 0;//mem write is low
                mem_address = 6'bx;
                mem_writedata = 32'bx;

                #1
                cache_data_array[index] = mem_readdata;   //cache update is done here 
                tag_array[index] = address[7:5];     
                valid_array[index] = 1'b1;           
                dirty_array[index] = 1'b0;          
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