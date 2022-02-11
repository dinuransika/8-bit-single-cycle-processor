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
	forwardunit f1(DATA2,SELECT,forwout);
	addunit a1(DATA1,DATA2,SELECT,addout);
	andunit and1(DATA1,DATA2,SELECT,andout);
	orunit or1(DATA1,DATA2,SELECT,orout);

	//Instantiate the mux to get the final output "RESULT"
	mux m1(forwout,addout,andout,orout,SELECT,RESULT);

endmodule
	
//FOWARD FUNCTION MODULE

module forwardunit(DATA2,SELECT,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs

	input [7:0] DATA2;
	input [2:0] SELECT;

	//Output port declaration
	//RESULT - 8 bit signed output = DATA2
	output  signed[7:0] RESULT;

	//Generate RESULT whenever DATA1 or DATA2 change
	//always @(SELECT,DATA2)
	 assign #1 RESULT=  DATA2;
 	
endmodule

//ADD FUNCTION MODULE

module addunit (DATA1,DATA2,SELECT,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	
	input signed [7:0] DATA1,DATA2;
	input [2:0] SELECT;

	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 + DATA2
	output  signed [7:0] RESULT;
	
	//Generate RESULT whenever DATA1 or DATA2 change
	//always @(SELECT,DATA1,DATA2)
	 assign #2 RESULT =  ( DATA1 + DATA2) ;
endmodule

//AND FUNCTION MODULE

module andunit (DATA1,DATA2,SELECT,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input signed [7:0] DATA1,DATA2;
	input [2:0] SELECT;

	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 & DATA2
	output   [7:0] RESULT;
	
	//Generate RESULT whenever DATA1 or DATA2 change
	//always @(DATA1,DATA2)
	 assign #1 RESULT = ( DATA1 & DATA2) ;
endmodule

//OR FUNCTION MODULE

module orunit (DATA1,DATA2,SELECT,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input signed [7:0] DATA1,DATA2;
	input [2:0] SELECT;
	
	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 | DATA2
	output [7:0] RESULT;

	//Generate RESULT whenever DATA1 or DATA2 change
	//always @(DATA1,DATA2)
	assign #1 RESULT =  ( DATA1 | DATA2) ;
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
			default: result = -1;
			
		endcase
	end
endmodule
