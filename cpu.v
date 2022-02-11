//CO224 VERILOG LAB 5- PART 5
// CPU file
//GROUP 09
`timescale  1ns/100ps

module cpu_tb;

    reg CLK, RESET;
	wire IMEM_BUSWAIT, I_BUSWAIT, D_BUSYWAIT,IMEM_READ;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;
    wire [7:0] REGOUT1,RESULT,READDATA;
  	wire WRITE,READ,BUSYWAIT,mem_read,mem_write,mem_busy;
	wire [5:0]mem_address;
	wire [31:0] mem_writedata,mem_readdata;
	wire [127:0] MEM_INSTR;
	wire [5:0] IMEM_ADDRESS;

    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    //reg [7:0] instr_mem [0:1023];

    /*
    initial
    begin
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    //combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    assign #2 INSTRUCTION ={instr_mem[PC+3] , instr_mem[PC+2] , instr_mem[PC+1],instr_mem[PC]};	*/
   /* -----
     CPU
    -----
    */
	//instantiate the cpu
	assign BUSYWAIT = (I_BUSWAIT|| D_BUSYWAIT)? 1:0;
	cpu mycpu(PC, REGOUT1,RESULT,WRITE,READ,READDATA,BUSYWAIT || I_BUSWAIT,INSTRUCTION, CLK, RESET);
	dcache cache1(mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busy,D_BUSYWAIT,READDATA,REGOUT1,RESULT,READ,WRITE,CLK,RESET);
	//instantiate the data memory
	//module instr_cache(clk,reset, address_in,read_inst_in, busywait_in, busywait_out, address_out, read_inst, read);
	instr_cache c3(CLK, RESET, PC, MEM_INSTR, IMEM_BUSWAIT, I_BUSWAIT, IMEM_ADDRESS, INSTRUCTION, IMEM_READ);
	instruction_memory imem(CLK, IMEM_READ, IMEM_ADDRESS, MEM_INSTR, IMEM_BUSWAIT);
	data_memory mem1(CLK,RESET,mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busy);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b1;
	#5 RESET = 1'b0;
        
       // finish simulation after some time
        #5000
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule

/*
__________
CPU MODULE
__________
*/
module cpu(PC, REGOUT1,RESULT,WRITE,READ,READDATA,BUSYWAIT,INSTRUCTION, CLK, RESET);
	//Input and output port decleration
   	input CLK, RESET;
   	input   [31:0] INSTRUCTION; //32 bit instruction.
   	output   [31:0] PC_OUT;		//PC address from the muxs	
    output  [31:0] PC; //PC 32 bit address for the instruction memory.
	
    //decleration of ports  to regfile, alu
    input [7:0] READDATA,WRITEDATA,INPUT2,READREG2,READREG1,WRITEREG;
	input [7:0] OPCODE;
    wire ZERO;
    output [7:0] RESULT,REGOUT1, REGOUT2,temp2, temp3; 	//regfile two outputs ,complement and immediate mux outputs.
		
	//control signals
	/*
	temp_sel = 3 bit ALUOP signal
	comp_flag  for sub,bne,beq signal, mux select to get the complement
	imm_flag ,mux select signal to select the immediate value
	j_flag = 1 for jump and bne
	b_flag = 1 for beq and bne
	WRITE = 1 when data needs to be written to regfile (low for j,bne,beq) 
	*/
	reg [2:0] temp_sel;				//for decoded ALUOP signal
	reg comp_flag, imm_flag,j_flag,b_flag,sra_flag,write_muxsel; //sra_flag =1 arithmatic shift
	output reg READ,WRITE;
	reg WRITEENABLE;
	input BUSYWAIT;
	//PC_adder,PC_jump adder signals
	wire  pc_j, pc_b; //select signals for jump and branch muxes
	input signed [7:0] OFFSET; //8 bit jump/branch offset
	output [31:0] PC_JUMP; //PC+4+OFFSET
	wire [31:0] PC_BRANCH, PC_NEXT; //PC_BRANCH = branch mux(m3) output //PC_NEXT= final pc output(m4 mux)
	input [31:0] PC_UPDATE;    //PC+4

	// compliment2= REGOUT2 2s comp , sra_result = OPERAND2 2s comp (to identify an sra operation) 
	wire [7:0] compliment2, sra_result;
	
	//assigning the opcode,writereg,readregs.
	assign {OPCODE,WRITEREG,READREG1,READREG2} =INSTRUCTION;

	//set memory READ and WRITE signals to 0 at a new PC value (to enable consecutive memory read/write instructions)
	always @(PC)
	begin
		READ = 0;
		WRITE = 0;
	end
	
		
	//PC_NEXT calculation
	assign OFFSET = INSTRUCTION[23:16];
	pc_adder add1(PC, PC_UPDATE); //Adding 4 for the current PC and store it in the next PC
	jump_adder j1(PC_UPDATE, OFFSET, PC_JUMP); //Calculate the pc jump address
	mux2x1_32 m3(PC_JUMP, PC_UPDATE, PC_BRANCH, pc_j); //select the next pc value depending on jump/branch
	mux2x1_32 m4(PC_JUMP,PC_BRANCH,PC_NEXT, pc_b);
	mux2x1_32 m5(PC, PC_NEXT, PC_OUT, BUSYWAIT);
	//update/reset the pc
	PC_update pc1(RESET, PC, PC_OUT, CLK);

	//Update the PC value only if BUSYWAIT =0
/*	always @(posedge CLK, BUSYWAIT,PC_OUT)
	begin
		if( !BUSYWAIT )
		begin
			PC = PC_OUT;
		end
		/*else begin
			PC=PC;
		end
	end	*/
	//Decoding the opcode and assigning them to temp_sel
	complement_module c1(REGOUT2, compliment2);
	//assign #1 compliment2 =  ~REGOUT2 + 1;
	always @(OPCODE,PC)
	begin
	case (OPCODE)
		8'b00000000: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_000_0_1_0_0_0_0_0_0; //loadi
		8'b00000001: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_000_0_0_0_0_0_0_0_0; //mov
		8'b00000010: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_001_0_0_0_0_0_0_0_0; //add
		8'b00000011: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_001_1_0_0_0_0_0_0_0; //sub
		8'b00000100: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_010_0_0_0_0_0_0_0_0; //and
		8'b00000101: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_011_0_0_0_0_0_0_0_0; //or
		8'b00000110: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b0_000_0_0_0_1_0_0_0_0; //jump
		8'b00000111: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b0_001_1_0_1_0_0_0_0_0; //beq
		8'b00001000: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b0_001_1_0_1_1_0_0_0_0; //bne
		8'b00001001: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_100_0_0_0_0_0_0_0_0; //mult
		8'b00001010: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_101_0_1_0_0_0_0_0_0; //sll
		8'b00001011: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_110_0_1_0_0_0_0_0_0; //srl
		8'b00001100: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_110_0_1_0_0_1_0_0_0; //sra
	    8'b00001101: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_111_0_1_0_0_0_0_0_0; //ror	8
		8'b00001110: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_000_0_0_0_0_0_1_0_1; //lwd
		8'b00001111: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b1_000_0_1_0_0_0_1_0_1; //lwi
		8'b00010000: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b0_000_0_0_0_0_0_0_1_0; //swd
		8'b00010001: {WRITEENABLE,temp_sel,comp_flag,imm_flag,b_flag,j_flag,sra_flag,READ,WRITE,write_muxsel} = #1 12'b0_000_0_1_0_0_0_0_1_0; //swi
	
	endcase
	end
	//MUX for choose the 2s' compliment of REGOUT2.
	mux2x1 m1(compliment2, REGOUT2, temp2, comp_flag);
	//MUX to choose immediate data or the regfile data.
	mux2x1 m2(INSTRUCTION[7:0], temp2, temp3, imm_flag);
	//Get the 2s complement of temp3(for sra temp3=immediate value)
	complement_module c2(temp3, sra_result);	  
	//MUX to choose the 2s complemnt of sra_result(sra_flag=1) or temp3(sra_flag=0) , 
	mux2x1 ma(sra_result, temp3, INPUT2, sra_flag);
	//ALU unit for the CPU
    alu alu1(REGOUT1, INPUT2, RESULT, ZERO, temp_sel);
	//mux to select the writedata to the regfile (from memory or alu result)
	mux2x1 write_mux(READDATA,RESULT,WRITEDATA,write_muxsel);
	//Instantiate the register.
   	reg_file reg1(WRITEDATA, REGOUT1, REGOUT2,WRITEREG,READREG1,READREG2, WRITEENABLE, CLK, RESET,BUSYWAIT);
	
	 //Flag for jump instruction. pc_j =1 iff b_flag=0 and j_flag=1(for jump instruction only)
	assign pc_j = ~b_flag && j_flag; 
	// Flag for branch instructions. pc_b=1 iff (bflag=1 and (j=1 and ZERO=0)=>bne) or( bflag=1 and (j=0 and ZERO=1)=>beq)	 
	assign pc_b = b_flag && (j_flag ^ ZERO);  //
	
endmodule


//8 bit 2x1 mux module . to choose complement and immediate values
module mux2x1(in1, in2, out, control);
	input [7:0] in1, in2;
	input control;
	output reg [7:0] out;
	always @(control, in1, in2)
	begin
		if(control)
		begin
		  out = in1;
		end else begin
		  out = in2;
		end
	end
endmodule

//32 bit 2x1 mux module. to choose pc_jump or pc+4 values
module mux2x1_32(in1, in2, out, control);
	input [31:0] in1, in2;
	input control;
	output reg [31:0] out;
	always @(control, in1, in2)
	begin
		if(control)
		begin
		  out = in1;
		end else begin
		  out = in2;
		end
	end
endmodule


//Reset and update PC
module PC_update(RESET, PC_0, PC_NEXT, CLK);
	output reg [31:0] PC_0;  //Current PC
	input CLK, RESET;
	input [31:0] PC_NEXT;
	
	always @(posedge CLK)		//when a positive clk edge
	begin
	  if(RESET) begin			//If reset is 1, initialisse the current PC
	  	PC_0 = #1 0;
	  end else begin
 	 //If reset == 0 , update next PC val to the current PC after a 1 time unit.
		#1 PC_0 =  PC_NEXT;
	  end
	end
endmodule

//PC_JUMP adder
module jump_adder(PC_4, OFFSET, PC_JUMP);
	input [31:0] PC_4;
	output [31:0] PC_JUMP;
	input signed [7:0] OFFSET;
	wire [31:0] temp;
	assign temp = {{22{OFFSET[7]}}, OFFSET[7:0], {2{1'b0}}};  //sign extended, left shifted by 4 offset value
	assign #2 PC_JUMP = temp + PC_4;

endmodule

//for incrementing the PC by 4
module pc_adder(PC_0, PC_1);
	input  [31:0] PC_0;
	output [31:0] PC_1;
	assign #1 PC_1 =  PC_0 +4;
endmodule

//Module to get the 2s complement
module complement_module(DATA, COMPDATA);
	input [7:0] DATA;
	output [7:0] COMPDATA;
	assign #1 COMPDATA = ~DATA + 1;
endmodule

`include "alu_group09.v"			//Include the module ALU
`include "regfile_group09.v"		//Include the module regfile
`include "data_mem_group09.v"
`include "dcache_group09.v"
`include "Instruction_mem_group09.v"
`include "Instruction_cache.v"