//CO224 - VERILOG LAB 5- PART 1
//ALU MODULE
//GROUP 09

//TEST BENCH MODULE
module test;
	//Input variable declaration
	//OPERAND1 & OPERAND2 = 8 bit signed input data
	//ALUOP = 3 bit selection input
	reg signed [7:0] OPERAND1,OPERAND2;
	reg [2:0] ALUOP;

	//Output variable declaration
	//ALURESULT = 8 bit signed output
	wire signed [7:0] ALURESULT;

	//Instantiate the alu module
	alu alu1(OPERAND1,OPERAND2,ALURESULT,ALUOP);

	//ALUOP Signals
	initial
	begin
		   ALUOP = 0;
		#5 ALUOP = 1;
		#5 ALUOP = 0;
		#3 ALUOP = 3;
		#15 ALUOP = 1;
	       	#5 ALUOP = 2;
		#5 ALUOP = 3;
		#2 ALUOP = 2;	
	end

	//OPERAND1 Signal
	initial
	begin
		   OPERAND1 = 2;
		#7 OPERAND1 = 4;
		#15 OPERAND1 = 7;
		#9 OPERAND1 = 5;
	
	end

	//OPERAND2 Signal
	initial
	begin 
		    OPERAND2 = -5;
		 #7 OPERAND2 = 2;
		 #20 OPERAND2 = 8;
		 #16 OPERAND2 = 6;
 	end
	
	//Monitor changes in SELECT,OPERAND1,OPERAND2
	initial	
	$monitor($time," SELECT = %b OPERAND1 = %d  OPERAND2 = %d | RESULT =  %d",ALUOP,OPERAND1,OPERAND2,ALURESULT);
	
	//Generate the files to create the waveform
	initial
	begin
		$dumpfile("alu.vcd");
		$dumpvars(0,test);
	end

endmodule

//ALU MODULE

module alu(DATA1,DATA2,RESULT,SELECT);
	//Input port declaration
	//DATA 1, DATA2 8 bit inputs
	//SELECT 3 bit input
	input signed [7:0] DATA1,DATA2;
	input [2:0] SELECT;

	//Output port declatation
	//RESULT,forw,addition are 8 bit outputs
	//RESULT= final output of the ALU depending on the select input
	//forwout = output of forwardunit module
	//addout = output of addunit module
	//andout = output of andunit module
	//orout = output of orunit module
	output signed [7:0] RESULT,forwout,addout,andout,orout;

	//Instantiate all the functional units
	forwardunit f1(DATA1,DATA2,forwout);
	addunit a1(DATA1,DATA2,addout);
	andunit and1(DATA1,DATA2,andout);
	orunit or1(DATA1,DATA2,orout);

	//Instantiate the mux to get the final output "RESULT"
	mux m1(forwout,addout,andout,orout,SELECT,RESULT);

endmodule
	
//FOWARD FUNCTION MODULE

module forwardunit(DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs

	input signed[7:0] DATA1,DATA2;

	//Output port declaration
	//RESULT - 8 bit signed output = DATA2
	output reg signed[7:0] RESULT;

	//Generate RESULT whenever DATA1 or DATA2 change
	always @(DATA2,DATA1)
	RESULT= #1 DATA2;
 	
endmodule

//ADD FUNCTION MODULE

module addunit (DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input signed [7:0] DATA1,DATA2;

	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 + DATA2
	output reg signed [7:0] RESULT;
	
	//Generate RESULT whenever DATA1 or DATA2 change
	always @(DATA1,DATA2)
	RESULT =#2 ( DATA1 + DATA2) ;
endmodule

//AND FUNCTION MODULE

module andunit (DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input signed [7:0] DATA1,DATA2;

	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 & DATA2
	output reg signed [7:0] RESULT;
	
	//Generate RESULT whenever DATA1 or DATA2 change
	always @(DATA1,DATA2)
	RESULT =#1 ( DATA1 & DATA2) ;
endmodule

//OR FUNCTION MODULE

module orunit (DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input signed [7:0] DATA1,DATA2;
	
	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 | DATA2
	output reg signed [7:0] RESULT;

	//Generate RESULT whenever DATA1 or DATA2 change
	always @(DATA1,DATA2)
	RESULT = #1 ( DATA1 | DATA2) ;
endmodule

	
//MULTIPLEXER MODULE (4 X 1)

module mux(in0,in1,in2,in3,sel,result);
	//Input port declaration
	//in0 = forwardunit output
	//in1 = addunit output
	//in2 = andunit output
	//in3 = orunit output
	//sel = 3 bit select input
	input signed [7:0] in0,in1,in2,in3;
	input [2:0] sel;

	//Output port declaration
	//result = final result of the mux depending on sel
	output reg signed[7:0] result;

	//Update the result whenever in0,in1,in2,in3 or sel change
	always @(in0,in1,in2,in3,sel)
	begin
		case (sel)
			3'b000:  result = in0;
			3'b001:  result = in1;
			3'b010:  result = in2;
			3'b011:  result = in3;
			default: $display($time," UNUSED SELECTION = %d",sel);
			
		endcase
	end
endmodule
