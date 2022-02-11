`timescale  1ns/100ps
module dcache (mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busywait,busywait,READDATA,WRITEDATA,ADDRESS,read,write,clk,reset);
        
        //INPUT PORT DECLARATION
        input clk,reset,read,write; //Control signals coming from the CPU
        input [7:0] WRITEDATA, ADDRESS; //Data to be written and cache address
        input [31:0] mem_readdata; //Data read from the memory
        input mem_busywait; //Busywait signal coming from memory

        //OUTPUT PORT DECLARATION
        output reg [0:31] mem_writedata;  //data to be written to memory when dirty bit =1 and hit=0
        output reg busywait,mem_read,mem_write; //busywait signal going out to the CPU, memory read,write signals
        output reg [7:0] READDATA; //data read from the cache
        output reg [5:0] mem_address; //memory address to be read or written

        reg valid_bit [0:7];  //valid bit array
        reg dirty_bit [0:7];  //dirty bit array
        reg [2:0] tag_bits [0:7];  //tag bits array
        reg [31:0] data [0:8];  //cache blocks
        output [7:0] out; //datablock selection mux output
        wire [2:0] tag,cache_tag,index; //tag of the input address, tag in the cache index,index of the address
        wire [1:0] offset; //offset of the address
        wire [31:0] data_block; //data block in the cache index
        wire dirty,valid,tag_comp; 
        reg hit;

        //FOR TESTING--------------------------------------------------
        initial
	    begin 
		$dumpfile("cpu_wavedata.vcd");
		for(i=0;i<8;i++)
			$dumpvars(1,valid_bit[i],tag_bits[i],data[i],dirty_bit[i]);
           

	    end

      /*  initial
	begin
		#5;
		/*$display("\n\t\t\t___________________________________________________");
		$display("\n\t\t\t CHANGE OF REGISTER CONTENT STARTING FROM TIME #5");
		$display("\n\t\t\t___________________________________________________\n");
		$display("\t\ttime\treg0\treg1\treg2\treg3\treg4\treg5\treg6\treg7");
		$display("\t\t____________________________________________________________________");
		$monitor($time, "\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%h",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
	end*/
        //______________________________________________________________

        //assert busywait when read or write signals change
        always @(read,write)
        begin
            busywait = (read||write)? 1 : 0;

        end

        //decode the address to get tag,index and offset bits
        assign {tag,index,offset} = ADDRESS;

        //extract values stored in the given index
        //delay #1
        assign #1 {dirty,valid,cache_tag,data_block} = {dirty_bit[index],valid_bit[index],tag_bits[index],data[index]};
        
        //select the correct cache element depending on the offset (delay #1 is in the mux)
            //elements        //00           //01               //10              //11
        mux4x1 mux1 (data_block[31:24],data_block[23:16],data_block[15:8],data_block[7:0],out,offset);

        //detect a hit or a miss
        assign tag_comp = ~(tag ^ cache_tag) ;
        always @(*)
            begin
                if((read||write))
                #0.9 hit = tag_comp && valid;
                     
            end
      
       
    //reading the cache when hit=1 is done asynchronously

    always @(hit,read,out)
        begin
            if(hit && read && !write)
            begin
                busywait = 0;
                READDATA = out;
             end
        end

    //writing to the cache when hit=1, done synchronously
    always @(posedge clk)
    begin
     if(write && !read && hit )
                begin
                busywait = 0;

                //Demux to writedata to the correct dataelement             
                case(offset)
                    2'b00:   data[index][31:24] = #1 WRITEDATA; 
                    2'b01:   data[index][23:16]=  #1 WRITEDATA; 
                    2'b10:   data[index][15:8] =  #1 WRITEDATA; 
                    2'b11:   data[index][7:0]=    #1 WRITEDATA; 
                   	
		        endcase
                //update dirty bit and valid bit
                dirty_bit[index] =1;
                valid_bit[index] =1;
                              
                end


    end

    /* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001,MEM_WRITE = 3'b010,CACHE_WRITE = 3'b011;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if ((read || write) && !dirty && !hit)  
                    next_state = MEM_READ;
                else if ((read || write) && dirty && !hit)
                    next_state = MEM_WRITE;
                else
                    next_state = IDLE;
                  
            MEM_READ:
           
                if ( !mem_busywait)
                    next_state = IDLE;
                else    
                    next_state = MEM_READ;

            MEM_WRITE:
           
                if((read || write)&& !mem_busywait)
                    next_state = MEM_READ;
                else
                    next_state = MEM_WRITE;

            
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
                mem_address = 8'dx;
                mem_writedata = 8'dx;
                busywait = 0;
               
            end
         
            MEM_READ: 
            begin
                
                mem_read = 1;
                mem_write = 0;
                mem_address = {tag, index};
                mem_writedata = 32'dx;
                busywait = 1;
                #1 //delay to write data loaded from memory to cache
                if(mem_busywait==0)
                begin
                    busywait = 1;
                    data[index] = {mem_readdata[7:0],mem_readdata[15:8],mem_readdata[23:16],mem_readdata[31:24]};
                    tag_bits[index] = tag;
                    dirty_bit[index] = 0;
                    valid_bit[index] = 1;
                end
            end

            MEM_WRITE:
            begin
                mem_read = 0;
                mem_write = 1;
                mem_address = {cache_tag,index};
                mem_writedata = {data_block[7:0],data_block[15:8],data_block[23:16],data_block[31:24]};
                busywait = 1;
            end
                  
        endcase
    end

    // sequential logic for state transitioning 
    always @(posedge clk, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

    /* Cache Controller FSM End */
 integer i;
        always @(reset)
        begin
            if (reset)
            begin
               // busywait =0;
            for (i=0;i<8; i=i+1)
                begin
                valid_bit[i] = 0;
                dirty_bit[i]=0;
                end
            end
        end
endmodule

module mux4x1(in0,in1,in2,in3,out,sel);
    input [7:0] in0,in1,in2,in3; 
    output reg [7:0] out;
    input [1:0] sel;

    always @(in0,in1,in2,in3,sel)
	begin
		case (sel)
			2'b00:  #1 out = in0; 
			2'b01:  #1 out = in1; 
			2'b10:  #1 out = in2; 
			2'b11:  #1 out = in3; 
			default: #1 out = -1;
			 
		endcase
	end
endmodule