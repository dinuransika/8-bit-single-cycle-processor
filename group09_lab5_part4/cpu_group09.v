//CO224 VERILOG LAB 5- PART 4
// CPU file
//GROUP 09


module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;
    
    
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    //  Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    reg [7:0] instr_mem [0:1023];

    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
       
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    // combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
       assign #2 INSTRUCTION ={instr_mem[PC+3] , instr_mem[PC+2] , instr_mem[PC+1],instr_mem[PC]};	
   /* -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b1;
	#5 RESET = 1'b0;
        
              
        // finish simulation after some time
        #300
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule

//CPU module
module cpu(PC, INSTRUCTION, CLK, RESET);
	//Input and output port decleration
   	input CLK, RESET;
    	input  [31:0] INSTRUCTION; //32 bit instruction.
    	output [31:0] PC;			//PC 32 bit address for the instruction memory.
    	//decleration of ports  to regfile and alu
    	input [7:0] WRITEDATA;
		input [2:0] READREG2,READREG1,WRITEREG;
    	input [3:0] opcode;
    	reg WRITE;
		wire ZERO;
    	output [7:0] REGOUT1, REGOUT2; 	//for regfile two outputs.
		output [7:0] temp2, temp3; 		//outputs for muxes.
		reg [2:0] temp_sel;				//for decoded output
		wire  pc_j, pc_b;               //select signals for muxes in the pc
		reg comp_flag, imm_flag,j_flag,b_flag; //select signals for complement mux(m1),immediate mux(m2), and branch and jump
		wire [7:0] compliment2;        //store the 2s complement of operand2

		output [31:0] PC_JUMP;   //pc_jump address
		wire [31:0] mux3out, mux4out;  //jump mux output and branch mux output
		input signed [7:0] OFFSET;   //jump offset
		assign OFFSET = INSTRUCTION[23:16];
		input [31:0] PC_1;       //Next PC


		//assigning the opcode,writereg,readregs.
		assign opcode = INSTRUCTION[31:24];
		assign WRITEREG = INSTRUCTION[23:16];
		assign READREG1 = INSTRUCTION[15:8];
		assign READREG2 = INSTRUCTION[7:0];


	//Calculating next pc
	pc_adder add1(PC, PC_1); //Adding 4 to the current PC and store it in the next PC
	jump_adder j1(PC_1, OFFSET, PC_JUMP);  //calculate the jump pc address
	mux2x1_32 m3(PC_JUMP, PC_1, mux3out, j_flag);  //select either pc+4 or pc_jump depending on jump flag
	mux2x1_32 m4(PC_JUMP, mux3out, mux4out, pc_b); //select either pc+4 or pc_jump depending on branch flag

		
	//reset or update the pc
	PC_reset res(RESET, PC, mux4out, CLK);
	//Instantiate the register.
   	reg_file reg1(WRITEDATA, REGOUT1, REGOUT2,WRITEREG,READREG1,READREG2, WRITE, CLK, RESET);
	//get the 2s complement of operand2
	assign #1 compliment2 =  ~REGOUT2 + 1;
	//Decoding the opcode and assiging select signals
	always @(opcode)
	begin
	case (opcode)
		4'b0000: {WRITE,temp_sel,comp_flag,imm_flag,b_flag,j_flag} = #1 8'b1_000_0_1_0_0; //loadi
		4'b0001: {WRITE,temp_sel,comp_flag,imm_flag,b_flag,j_flag} = #1 8'b1_000_0_0_0_0; //mov
		4'b0010: {WRITE,temp_sel,comp_flag,imm_flag,b_flag,j_flag} = #1 8'b1_001_0_0_0_0; //add
		4'b0011: {WRITE,temp_sel,comp_flag,imm_flag,b_flag,j_flag} = #1 8'b1_001_1_0_0_0; //sub
		4'b0100: {WRITE,temp_sel,comp_flag,imm_flag,b_flag,j_flag} = #1 8'b1_010_0_0_0_0; //and
		4'b0101: {WRITE,temp_sel,comp_flag,imm_flag,b_flag,j_flag} = #1 8'b1_011_0_0_0_0; //or
		4'b0110: {WRITE,comp_flag,imm_flag,b_flag,j_flag} 		   = #1 5'b0_0_0_0_1;     //jump
		4'b0111: {WRITE,temp_sel,comp_flag,imm_flag,b_flag,j_flag} = #1 8'b0_001_1_0_1_0; //beq
	endcase
	end
	//MUX for choose the 2s' compliment.
	mux2x1 m1(compliment2, REGOUT2, temp2, comp_flag);
	//MUX to choose immediate data or the regfile data.
	mux2x1 m2(INSTRUCTION[7:0], temp2, temp3, imm_flag);
	//ALU unit for the CPU
    alu alu1(REGOUT1, temp3, WRITEDATA, ZERO, temp_sel);
	//pc_b =1 if b_flag and zero are both 1(if a branch instruction is given and the operand1 and 2 are equal)
	assign pc_b = b_flag && ZERO;
	
   
endmodule

//8 bit 2x1 mux module to select immediate value and 2s complement
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

//32 bit 2x1 mux module to select pc_jump or pc+4 value as the next pc
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
module PC_reset(RESET, PC_0, PC_1, CLK);
	input [31:0] PC_1;       //Next PC
	output reg [31:0] PC_0;  //Current PC
	input CLK, RESET, b_flag, j_flag; //select signals for branch mux(m4) and jump mux(m3)
	
	always @(posedge CLK)		//when a positive clk edge
	begin
	  if(RESET) begin			//If reset is 1, initialisse the current PC
	  	PC_0 = #1 0;
	  end else begin			//If reset == 0 , update next PC val to the current PC after a 1 time unit.
		#1 PC_0 =  PC_1;
	  end
	end
endmodule

//pc_jump adder
module jump_adder(PC_4, OFFSET, PC_JUMP);
	input [31:0] PC_4;
	output [31:0] PC_JUMP;
	input signed [7:0] OFFSET;
	wire [31:0] temp;
	assign temp = {{22{OFFSET[7]}}, OFFSET[7:0], {2{1'b0}}}; 
	assign #2 PC_JUMP = temp + PC_4;

endmodule

//for incrementing the PC
module pc_adder(PC_0, PC_1);
	input  [31:0] PC_0;
	output [31:0] PC_1;
	assign #1 PC_1 =  PC_0 +4;
endmodule

`include "alu_group09.v"			//Include the module ALU
`include "regfile_group09.v"		//Include the module regfile
