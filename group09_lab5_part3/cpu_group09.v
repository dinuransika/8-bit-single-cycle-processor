//CO224 VERILOG LAB 5- PART 3
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
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    reg [7:0] instr_mem [0:1024];

    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        //{instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        //{instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        //{instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
    assign #2 INSTRUCTION[7:0] = instr_mem[PC];
    assign #2 INSTRUCTION[15:8] = instr_mem[PC+1];
    assign #2 INSTRUCTION[23:16] = instr_mem[PC+2];
    assign #2 INSTRUCTION[31:24]= instr_mem[PC+3];
    	
    initial
	   // PC_CURRENT=0;
	    //$monitor($time," %b %b %b %b",instr_mem[4],instr_mem[5],instr_mem[6],instr_mem[7]);
    	    $monitor($time," PC  %d INSTRUCTION IN MEMORY %b",PC,INSTRUCTION);
			//$monitor($time,"\t%d\t%d\t%d\t%d\t%d\t%d\t%d", regfile[0], regfile[1],regfile[2],regfile[3],regfile[4],regfile[5],regfile[6],regfile[7],);
    /* 
    -----
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
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        
        // finish simulation after some time
        #200
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
    input   [31:0] INSTRUCTION; //32 bit instruction.
    output [31:0] PC;			//PC 32 bit address for the instruction memory.
    //decleration of ports  to regfile and alu
	input [7:0] WRITEDATA;
	input [2:0] opcode;
    reg WRITE;
    output [7:0] REGOUT1, REGOUT2; 	//for regfile two outputs.
	output [7:0] temp2, temp3; 		//outputs for muxes.
	reg [2:0] temp_sel;				//for decoded output
	//assigning the opcode.
	assign opcode = INSTRUCTION[31:24];
	reg comp_flag, imm_flag;
	wire [7:0] compliment2;
	//reset the pc
	PC_reset res(RESET, PC, CLK);
	//Instantiate the register.
    reg_file reg1(WRITEDATA, REGOUT1, REGOUT2, INSTRUCTION[18:16], INSTRUCTION[10:8], INSTRUCTION[2:0], 1'b1, CLK, RESET);
	//Decoding the opcode and assigning them to temp_sel
	assign #1 compliment2 =  ~REGOUT2 + 1;
	always @(opcode)
	begin
	case (opcode)
		3'b000: begin
		  		temp_sel <= #1 3'b000; 
				comp_flag <= #1 0; 
				imm_flag <= #1 1;
			end
		3'b001: begin
				temp_sel <= #1 3'b000;
				comp_flag <= #1 0; 
				imm_flag <= #1 0;
			end 
		3'b010: begin
				temp_sel <= #1 3'b001;
				comp_flag <= #1 0; 
				imm_flag <= #1 0;
			end 
		3'b011: begin
				temp_sel <= #1 3'b001;
				comp_flag <= #1 1; 
				imm_flag <= #1 0;
		end 
		3'b100: begin
				temp_sel <= #1 3'b010;
				comp_flag <= #1 0; 
				imm_flag <= #1 0;
		end 
		3'b101: begin
				temp_sel <= #1 3'b011;
				comp_flag <= #1 0; 
				imm_flag <= #1 0;
		end 
	endcase
	end
	//MUX for choose the 2s' compliment.
	mux2x1 m1(compliment2, REGOUT2, temp2, comp_flag);
	//MUX to choose immediate data or the regfile data.
	mux2x1 m2(INSTRUCTION[7:0], temp2, temp3, imm_flag);
	//ALU unit for the CPU
    alu alu1(REGOUT1, temp3, WRITEDATA, temp_sel);
    //reg_file reg2(WRITEDATA, REGOUT1, REGOUT2, INSTRUCTION[18:16], READREG1, READREG2, WRITE, CLK, RESET);
endmodule


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


//Reset and update PC
module PC_reset(RESET, PC_0, CLK);
	input [31:0] PC_1;       //Next PC
	output reg [31:0] PC_0;  //Current PC
	input CLK, RESET;
	pc_adder add1(PC_0, PC_1); //Adding 4 for the current PC and store it in the next PC
	always @(posedge CLK)		//when a positive clk edge
	begin
	  if(RESET) begin			//If reset is 1, initialisse the current PC
	  	PC_0 = #1 0;
	  end else begin			//If reset == 0 , update next PC val to the current PC after a 1 time unit.
		PC_0 = #1 PC_1;
	  end
	end
endmodule

//for incrementing the PC
module pc_adder(PC_0, PC_1);
	input  [31:0] PC_0;
	output [31:0] PC_1;
	assign #1 PC_1 =  PC_0 +4;
endmodule

`include "alu_group09.v"			//Include the module ALU
`include "regfile_group09.v"		//Include the module regfile