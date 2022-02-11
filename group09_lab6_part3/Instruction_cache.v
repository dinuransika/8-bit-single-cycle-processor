`timescale  1ns/100ps
//10 bit address--> x x x   x x x    x x x x
//                 | tag || index | |offset|
module instr_cache(clk,reset, address_in,read_inst_in, busywait_in, busywait_out, address_out, read_instr, read);

//Input/output poert declaration
input clk, busywait_in, reset; //busywait_in=busywait signal coming from instruction memory
input [31:0] address_in; //address given from CPU 
input [127:0] read_inst_in; //Instruction read from the memory
output reg [5:0] address_out; //address given to the memory
output reg [31:0] read_instr; //Instruction directed to the CPU
output reg busywait_out, read; //busywait_out = busywait signal for the CPU
wire [9:0] temp_addr; //10 bit address extracted from the 32 bit PC address

wire [2:0] tag, index, cache_tag; 
wire [3:0] offset;
reg valid_bit [7:0] ;
reg [2:0] tag_bits [7:0];
reg [127:0] instr [7:0];  //cache array with 128 bit block size (8 blocks)
reg hit;
wire [127:0] data_block;
wire [31:0] out;
wire valid,tag_comp; 



 initial
	    begin 
		$dumpfile("cpu_wavedata.vcd");
		for(i=0;i<8;i++)
			$dumpvars(1,valid_bit[i],tag_bits[i],instr[i]);
           

	    end

//extract the 10 lsbs from 32 bit PC address
assign temp_addr = address_in;
//decode the 10 bit address
assign {tag, index, offset} = temp_addr;
//extract the data stored in the given index in the cache arrays
//delay = #1
assign #1 {cache_tag, valid, data_block} = {tag_bits[index],valid_bit[index], instr[index]};

//extract the instruction according to the offset
//delay #1
mux4x1_1 mux1 (data_block[31:0],data_block[63:32],data_block[95:64],data_block[127:96],out,offset);

//compare the tags
//delay #0.9
assign #0.9 tag_comp = ~(tag ^ cache_tag) ;

//assert the hit signal depending on the tag comparison and valid bit
always @(*)
    
    begin
         hit = tag_comp && valid;            
    end

//output the instruction to the CPU if hit==1
 always @(hit,out)
        begin
            if(hit)
            begin
                busywait_out = 0;
                read_instr = out;
               
             end
        end

// CACHE FSM

parameter IDLE = 1'b0, MEM_READ = 1'b1;
reg  state, next_state;

//next state assignment
always @(*)
    begin
        case (state)
            IDLE:
                if (!hit)  
                    next_state = MEM_READ;
                else 
                    next_state = IDLE;
                  
            MEM_READ:
           
                if ( !busywait_in)
                    next_state = IDLE;
                else    
                    next_state = MEM_READ;
        endcase
    end

//output logic of states
always @(*)
begin
  case (state)
    
      IDLE: 
      begin
        read = 0;
        busywait_out=0;
        address_out = 6'dx;
        //if a miss is detected set the memory read signal to 1 immediately(to avoid the race condition)
        if(!hit)
        begin
          read = 1;
          address_out = {tag, index};
        end
      end

    //read instructions from memory
      MEM_READ:
      begin
        read = 1;
        busywait_out=1;
        address_out = {tag, index};
        //Write the instruction fetched from memory to cache
        #1
        if (!busywait_in)
        begin
          //busywait_out = 1;
          instr[index] = read_inst_in;
          tag_bits[index] =tag;
          valid_bit[index]=1;

        end
      end
  endcase
end

//reset state or assign next state at posedge of clk
always @(posedge clk)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

//reset the cache valid bits asynchronously
integer i;
always @(posedge clk)
begin
    if (reset)
    begin
       // busywait =0;
    for (i=0;i<8; i=i+1)
        begin
        valid_bit[i] = 0;
        end
    end
end

endmodule

//mux to select the correct instruction from the block according to the offset
module mux4x1_1(in0,in1,in2,in3,out,sel);
    input [31:0] in0,in1,in2,in3; 
    output reg [31:0] out;
    input [3:0] sel;

    always @(in0,in1,in2,in3,sel)
	begin
		case (sel)
			4'b0000:  #1 out = in0; 
			4'b0100:  #1 out = in1; 
			4'b1000:  #1 out = in2; 
			4'b1100:  #1 out = in3; 
			default: #1 out = -1;
			 
		endcase
	end
endmodule