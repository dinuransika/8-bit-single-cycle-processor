/*module test;
	reg signed  [7:0] op1,op2;
	reg [2:0] sel;
	wire signed  [7:0] out1,out2;
	wire zero;
	alu alu1(op1,op2,out1,zero,sel);
	initial
	begin
        sel=0;
        #5 sel=1;
        #5 sel=0;
        #3 sel=3;
        #15 sel=1;
        #5 sel=2;
        #5 sel=3;
        #2 sel=2;
	end
    initial
    begin
      op1=2;
      #7 op1=4;
      #15 op1=7;
      #9 op1=5;
    end
    initial
    begin
      op2=-5;
      #7 op2=2;
      #20 op2=8;
      #16 op2=6;
    end
	initial	
	$monitor($time," sel = %d op1 (%d), op2 (%d) = out %d, zero = %d",sel,op1,op2,out1, zero);
	initial
	begin
		$dumpfile("alu.vcd");
		$dumpvars(0,test);
	end	
endmodule
*/
module alu(DATA1,DATA2,RESULT,ZERO,SELECT);
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
	output ZERO;

	//Instantiate all the functional units
	forwardunit f1(DATA2,SELECT,forwout);
	addunit a1(DATA1,DATA2,SELECT,addout);
	andunit and1(DATA1,DATA2,SELECT,andout);
	orunit or1(DATA1,DATA2,SELECT,orout);
	//To check the result is equal to 0.
	assign ZERO = ~RESULT[0] && ~RESULT[1] && ~RESULT[2] && ~RESULT[3] && ~RESULT[4] && ~RESULT[5] && ~RESULT[6] && ~RESULT[7];
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
