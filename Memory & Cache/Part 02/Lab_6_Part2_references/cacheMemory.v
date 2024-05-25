`timescale  1ns/100ps
module cacheMemory(
	clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
    busywait,
    mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busywait);

input               clock;
input               reset;
input               read;
input               write;
input[7:0]          address;
input[7:0]         writedata;
output[7:0]         readdata;
output           busywait; 
output reg            mem_read,mem_write;
output reg [31:0]     mem_writedata;
output reg [5:0]      mem_address;
input [31:0]     mem_readdata;
input            mem_busywait; 


/* Cache memory storage register files */
reg[31:0] cache [0:7];
reg[2:0] cacheTag [0:7];
reg cacheDirty [0:7];
reg cacheValid [0:7];

reg[1:0] Offset;
reg[2:0] Index;
reg[2:0] Tag;

/* dividing address to respective tag index and offset Asynchronousyly */
always@(address) begin
 if(read || write)begin
 #1
 Offset <= address[1:0];
 Index <= address[4:2];
 Tag <= address[7:5];
 end
end

/*Asynchronous comparator to compare tag and AND gate to check valid bit is set */
wire comparator;
wire hit,dirty;
wire[2:0] comparatorTagIN;
assign comparatorTagIN = cacheTag[Index];

assign #0.9 comparator = Tag[0]~^comparatorTagIN[0] && Tag[1]~^comparatorTagIN[1] && Tag[2]~^comparatorTagIN[2];   //xnor each bit and did and operation

assign hit = cacheValid[Index] && comparator;

/*for future usage*/
assign dirty = cacheDirty[Index];


/*Asynchronous data extraction and assigning*/
wire [7:0] dataExtract;
wire[31:0] data;
assign data = cache[Index];

assign dataExtract = (Offset[1]) ? (Offset[0]? data[31:24]:data[23:16]) : (Offset[0]? data[15:8]:data[7:0]) ;
/*always@(*) begin /////////////////////////////////////////////////////////////////////////////////
case(Offset) 
            2'b11 : dataExtract = data[31:24];
            2'b10 : dataExtract = data[23:16];
            2'b01 : dataExtract = data[15:8];
            2'b00 : dataExtract = data[7:0]; 
        endcase 
end*/




wire readdata;
assign #1 readdata = dataExtract;

/*set busywait whenever a write or read signal received*/
reg Busywait;                                        
reg readaccess, writeaccess;


always @(read, write)
begin
    #0.1
    //$display($time,"value (216)of Busywait before : %d",Busywait);  
    Busywait = (read || write)? 1 : 0; 
    //$display("value of Busywait after : %d",Busywait);          
    readaccess = (read && !write)? 1 : 0;
    if(!read && write) begin writeaccess = 1; end

    //writeaccess = (!read && write)? 1 : 0;
end


/* to set busywait to zero when a hit occured */
    initial begin
        #5;
     //   $monitor($time, "\t%d\t%d\t%d\t%d", cache[0], cache[1], cache[2], cache[3], /*cache[4], cache[5], cache[6], cache[7]*/);
        //$monitor($time, "hi \t%b\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n\n", address_tag_array[0], address_tag_array[1], address_tag_array[2], address_tag_array[3], address_tag_array[4], address_tag_array[5], address_tag_array[6], address_tag_array[7]);
        $monitor($time,"Busywait (after 216?): %d ",Busywait);
    end

always@(readdata,writedata,writedata1,hit,readaccess) begin
#0.1/////////////////////////////////////////////
if (readaccess && hit)
    Busywait = 1'b0;                                
    
 else if (writeaccess && hit)   
    Busywait = 1'b0;

 
end 




always@(posedge clock)begin
 if (readaccess && hit)
    begin
   Busywait = 1'b0;     
                         
    end
 else if (writeaccess && hit)   
    begin
  Busywait = 1'b0;                                
    #1   
    cacheDirty[Index] = 1'b1;            //setting dirty bit to 1,this is the only place where dirty bit is set to 1                                //here cache write undergo even after busywait set to zero  

    case(Offset)                         //then set the input data into correct place in the cache block
    2'b11:
        cache[Index][31:24] = writedata;
    2'b10:
        cache[Index][23:16] = writedata;
    2'b01:
        cache[Index][15:8] = writedata;
    2'b00: 
        cache[Index][7:0] = writedata;
    endcase 
    end	
end


/*here i put the 32bit data block provided by data memory to the correct place in cache
and set valid bit to 1 and dirty bit to zero */
always@(mem_busywait)begin
    if(!mem_busywait)
    begin
    #1
	cache[Index] = mem_readdata;
    cacheValid[Index] = 1'b1;
    cacheDirty[Index] = 1'b0;
    cacheTag[Index] = Tag;
    end
end

/* here i provide the block tag and the data to be written to the data memory when there was a miss occured and 
   the block is dirty (to complete WRITE_BACK state) */
reg[31:0] writedata1;   //these are inputs to my cache controller
reg[2:0] Tag1;          //these are inputs to my cache controller
always@(hit)begin
	if(!hit && dirty)
       begin
	   writedata1 = cache[Index];
       Tag1 = cacheTag[Index];
       end
end


/* cache controller to handle data memory control signals(mem_read,mem_write etc) whenever a miss occured in cachememory */
reg controllerBusywait;                                  
//cacheController    mycacheController(clock,reset,read,write,address,writedata,controllerBusywait,mem_busywait,Tag1,writedata1,Tag,Index,hit,dirty,mem_read,mem_write,mem_writedata,mem_address);
//DELETE THIS ABOVE ONE

/* overall busywait is set to zero whenever cachecontroller busywait and cachememory busywait both set to zero */
wire busywait;                                           
assign busywait = (Busywait || controllerBusywait)? 1:0;        
integer i;

//Reset Cache memory
always @(posedge reset)
begin
    if (reset)
    begin
        for (i=0;i<8; i=i+1)
            begin
            cache[i] = 0;
            cacheTag[i] = 0;
            cacheDirty[i] = 0;
            cacheValid[i] = 0;
            end
        readaccess = 0;
        writeaccess = 0;
        Busywait = 0;
    end
end



    /* Cache Controller FSM Start */

    /*    here i used three states
          cache controller is used to set data memory control signals whenever a miss
              occured in cache memory
          please find the attached state diagrame for better understanding    */     


    parameter IDLE = 2'b00, MEM_READ = 2'b01, WRITE_BACK = 2'b10; 
    reg [1:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                 if ((read || write) && dirty && !hit)
                    begin
                     next_state = WRITE_BACK;
                    end             
                 else if ((read || write) && !dirty && !hit)
                    begin
                     next_state = MEM_READ;
                    end  
                 else 
                    begin
                     next_state = IDLE;
                    end            
            
            MEM_READ:
                 if (!mem_busywait)
                    begin
                     next_state = IDLE;
                    end
                 else 
                    begin   
                     next_state = MEM_READ;
                    end  

            WRITE_BACK:
                 if (!mem_busywait)
                    begin
                     next_state = MEM_READ;
                    end
                 else 
                    begin   
                     next_state = WRITE_BACK;
                    end

        endcase
    end

    // combinational output logic
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 6'dx;
                mem_writedata = 32'dx;
                controllerBusywait = 0;
            end

            MEM_READ: 
            begin
                mem_read = 1;
                mem_write = 0;
                mem_address = {Tag, Index};
                mem_writedata = 32'dx;
                controllerBusywait = 1;
            end
            
            WRITE_BACK:
            begin
            	mem_read = 0;
                mem_write = 1;
                mem_address = {Tag1, Index};
                mem_writedata = writedata1;
                controllerBusywait = 1;
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




/*loadi 0 0x06
swi 0 0x01
swi 0 0x05
lwi 4 0x01
lwi 5 0x05
loadi 7 0x06*/