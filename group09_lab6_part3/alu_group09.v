//CO224 VERILOG LAB 5- PART 5
// ALU file
//GROUP 09
`timescale  1ns/100ps
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
        #2 sel=4;
	#4 sel=5;
	#3 sel=6;
	#5 sel=7;
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
      #5 op2=2;
      #6 op2= 5;
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
/*
__________
ALU MODULE
__________
*/

module alu(DATA1,DATA2,RESULT,ZERO,SELECT);
	//Input port declaration
	//DATA 1, DATA2 8 bit inputs
	//SELECT 3 bit input
	input signed [7:0] DATA1,DATA2;
	input [2:0] SELECT;

	//Output port declatation
	/*RESULT,forw,addout,orout,mulout,sllout,srout,rorout- 8 bit outputs
		RESULT= final output of the ALU depending on the select input
		forwout = output of forwardunit module
		addout  = output of addunit module
		andout  = output of andunit module
		orout   = output of orunit module
		mulout  = output of mulunit module
		sllout  = output of shift_left unit module
		srout   = output of shift_right unit module
		rorout  = output of rotate_right unit module
	  ZERO = wire to determin whether the RESULT =0.
	*/
	output signed [7:0] RESULT,forwout,addout,andout,orout,mulout,sllout,srout,rorout;
	output ZERO;

	//Instantiate all the functional units
	forwardunit f1(DATA2,forwout);
	addunit a1(DATA1,DATA2,addout);
	andunit and1(DATA1,DATA2,andout);
	orunit or1(DATA1,DATA2,orout);
	mulunit mul1(DATA1,DATA2,mulout);
	sr sr1(DATA1,DATA2,srout);
	sl sll1(DATA1,DATA2,sllout);
    rotate_right ror1(DATA1,DATA2,rorout);

	//Instantiate the mux to get the final output "RESULT"
	mux m1(forwout,addout,andout,orout,mulout,sllout,srout,rorout,SELECT,RESULT);
	//set ZER0 to 1 if RESULT = 0 
	and (ZERO,~RESULT[0], ~RESULT[1], ~RESULT[2], ~RESULT[3] , ~RESULT[4], ~RESULT[5], ~RESULT[6], ~RESULT[7]);

endmodule
	
//FOWARD FUNCTION MODULE

module forwardunit(DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input [7:0] DATA2;
	
	//Output port declaration
	//RESULT - 8 bit signed output = DATA2
	output  signed[7:0] RESULT;

	//Generate RESULT whenever DATA1 or DATA2 change
	assign #1 RESULT=  DATA2;
 	
endmodule

//ADD FUNCTION MODULE

module addunit (DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input signed [7:0] DATA1,DATA2;
	
	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 + DATA2
	output  signed [7:0] RESULT;
	
	//Generate RESULT whenever DATA1 or DATA2 change
	assign #2 RESULT =  ( DATA1 + DATA2) ;
endmodule

//AND FUNCTION MODULE

module andunit (DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input signed [7:0] DATA1,DATA2;

	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 & DATA2
	output   [7:0] RESULT;
	
	//Generate RESULT whenever DATA1 or DATA2 change
	assign #1 RESULT = ( DATA1 & DATA2) ;
endmodule

//OR FUNCTION MODULE

module orunit (DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input signed [7:0] DATA1,DATA2;
		
	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 | DATA2
	output [7:0] RESULT;

	//Generate RESULT whenever DATA1 or DATA2 change
	assign #1 RESULT =  ( DATA1 | DATA2) ;
endmodule

//MULTIPLICATION FUNCTION MODULE

module mulunit (DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input  [7:0] DATA1,DATA2;
	
	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 * DATA2
	output  [7:0] RESULT;

	wire [7:0] shift [0:7]; //store the shifted DATA1 values
	wire [7:0] temp0, temp1, temp2,temp3,temp4, temp5, temp6, temp7;   //store either DATA1 or 0 , depending on whether DATA2[i] =1 or 0
	integer i;

	/*MULTIPLICATION ALGORITHM -BINARY MULTIPLICATION
	Check each bit in DATA2 from LSB to MSB. If DATA2[i]=1 , temp[i] = DATA1 , shift[i] = DATA1<<i
	RESULT = addition of data stored in each 8bit element in shift wire.
	Delay for all the left shifting = #1
	Delay for adding the shifted values = #2
	Total delay for mult operation = #3
	*/
	shift_mux2x1 mm1(DATA1, 8'b00000000, temp0, DATA2[0]);
	shift_mux2x1 mm2(DATA1, 8'b00000000, temp1, DATA2[1]);
	shift_mux2x1 mm3(DATA1, 8'b00000000, temp2, DATA2[2]);
	shift_mux2x1 mm4(DATA1, 8'b00000000, temp3, DATA2[3]);
	shift_mux2x1 mm5(DATA1, 8'b00000000, temp4, DATA2[4]);
	shift_mux2x1 mm6(DATA1, 8'b00000000, temp5, DATA2[5]);
	shift_mux2x1 mm7(DATA1, 8'b00000000, temp6, DATA2[6]);
	shift_mux2x1 mm8(DATA1, 8'b00000000, temp7, DATA2[7]);
	
	sl sll2(temp0,8'b00000000,shift[0]);
	sl sll3(temp1,8'b00000001,shift[1]);
	sl sll4(temp2,8'b00000010,shift[2]);
	sl sll5(temp3,8'b00000011,shift[3]);
	sl sll6(temp4,8'b00000100,shift[4]);
	sl sll7(temp5,8'b00000101,shift[5]);
	sl sll8(temp6,8'b00000110,shift[6]);
	sl sll9(temp7,8'b00000111,shift[7]);
	assign #2 RESULT = shift[0]+ shift[1]+ shift[2]+ shift[3]+  shift[4]+ shift[5]+ shift[6]+ shift[7];

endmodule

//SHIFT LEFT FUNCTION MODULE
module sl(DATA1,DATA2,RESULT);
	input [7:0] DATA1,DATA2;

	//Output port declaration
	//RESULT - 8 bit signed output = DATA1 << DATA2
	output [7:0] RESULT;
	
	//use 3 muxs to shift data1. 
	//shifted1 = data1<<1
	//out1,out2,out4 = outputs of muxs
	wire [7:0] shifted1, OUT1, OUT2, OUT4;
	assign shifted1 = {DATA1[6:0], 1'b0};
	
	/*if DATA2[0] =1 ,OUT1 = DATA1<<1
	  if DATA2[1] =1 ,OUT2 = OUT1<<2 (If OUT1=DATA1<<1,OUT2 =DATA1<<3)
	  if DATA2[3] =1 ,OUT4 = OUT2<<4 (If OUT1=DATA1<<1,OUT2 =DATA1<<3,out4 = DATA<<7)
	*/
	shift_mux2x1 ml1(shifted1, DATA1,OUT1, DATA2[0]);
	shift_mux2x1 ml2({OUT1[5:0],2'b00}, OUT1, OUT2, DATA2[1]);
	shift_mux2x1 ml4({OUT2[3:0], 4'b0000}, OUT2, OUT4, DATA2[2]);

	//Delay =1
	assign #1 RESULT = OUT4;
endmodule
	
//SHIFT RIGHT FUNCTION MODULE

module sr(DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input [7:0] DATA1,DATA2;

	//Output port declaration
	//RESULT - 8 bit signed output = logical right shift DATA1 if DATA2[7]=0 , else arithmetic right shift DATA1
	output [7:0] RESULT;
	
	//comp = 2s comp of DATA2
	//mod_data2 = output of mod mux
	//shifted1 = DATA1 >>1
	//OUT1,OUT2,OUT4 = Output of muxs
	wire [7:0] comp,mod_data2,shifted1, OUT1, OUT2, OUT4;
	wire msb;
	
	//2s complement of DATA2
	//Delay=1
	assign #1 comp = ~DATA2 + 1;
	//get the msb of DATA1 if the operation is sra (DATA2[7]=1)
	bit_mux2x1 muxmsb(DATA1[7],1'b0,msb,DATA2[7]);
	//mod_data2=comp id operation is sra(DATA2[7]=1)
	shift_mux2x1 mod(comp,DATA2,mod_data2,DATA2[7]);
	assign shifted1 = {msb, DATA1[7:1]};
	
	/*
	if mod_data2[0] =1 ,OUT1 = DATA1>>1
	if mod_data2[1] =1 ,OUT2 = OUT1>>2 (If OUT1=DATA1>>1,OUT2 =DATA1>>3)
	ifmod_data2[3] =1 ,OUT4 = OUT2>>4 (If OUT1=DATA1>>1,OUT2 =DATA1>>3,out4 = DATA>>7)
	*/
	shift_mux2x1 ml1(shifted1, DATA1,OUT1, mod_data2[0]);
	shift_mux2x1 ml2({{2{msb}},OUT1[7:2]}, OUT1, OUT2, mod_data2[1]);
	shift_mux2x1 ml4({{4{msb}},OUT2[7:4]}, OUT2, OUT4, mod_data2[2]);

	//Delay =1
	assign #1 RESULT = OUT4;
endmodule

//ROTATE RIGHT FUNCTION MODULE
module rotate_right(DATA1,DATA2,RESULT);
	//Input port declaration
	//DATA1,DATA2 - 8 bit signed inputs
	input [7:0] DATA1,DATA2;

	//Output port declaration
	//RESULT - 8 bit signed output = Rotate right DATA1 by DATA2 amount
	output  [7:0] RESULT;

	//rotated1 = DATA1 rotated by 1
	//OUT1,OUT2,OUT4 =mux outputs
	wire [7:0] rotate1,OUT1, OUT2, OUT4;
	assign rotate1 = {DATA1[0],DATA1[7:1]};

	/*
	if DATA2[0] =1 ,OUT1 = DATA1 rotated by 1
	if DATA2[1] =1 ,OUT2 = OUT1 rotated by 2 (If OUT1=DATA1 ror 1,OUT2 =DATA1 ror 3)
	if DATA2[3] =1 ,OUT4 = OUT rotated by 4 (If OUT1=DATA1ror 1,OUT2 =DATA1 ror 3,out4 = DATA ror 7)
	*/
	shift_mux2x1 mrr1(rotate1, DATA1,OUT1, DATA2[0]);
	shift_mux2x1 mrr2({OUT1[1:0],OUT1[7:2]}, OUT1, OUT2, DATA2[1]);
	shift_mux2x1 mrr4({OUT2[3:0],OUT2[7:4]}, OUT2, OUT4, DATA2[2]);

	//Delay = 1
	assign #1 RESULT = OUT4;
		
endmodule

//MULTIPLEXER MODULE (8 X 1)

module mux(in0,in1,in2,in3,in4,in5,in6,in7,sel,result);
	/*Input port declaration
		in0 = forwardunit output
		in1 = addunit output
		in2 = andunit output
		in3 = orunit output
		in4 = mult unit output
		in5 = shift_left unit output
		in6 = shift_right unit output
		in7 = rotate_right unit output
		sel = 3 bit select input
	*/
	input signed [7:0] in0,in1,in2,in3,in4,in5,in6,in7;
	input [2:0] sel;

	//Output port declaration
	//result = final result of the mux depending on sel
	output reg signed[7:0] result;

	//Update the result whenever in0,in1,in2,in3 or sel change
	always @(in0,in1,in2,in3,in4,in5,in6,in7,sel)
	begin
		case (sel)
			3'b000:  result = in0; //forward
			3'b001:  result = in1; //add
			3'b010:  result = in2; //and
			3'b011:  result = in3; //or
			3'b100:  result = in4; //mult
			3'b101:  result = in5; //sll
			3'b110:  result = in6; //srl
			3'b111:  result = in7; //ror
			default: result = -1;
			
		endcase
	end
endmodule

//8 BIT 2x1 MUX MODULE
module shift_mux2x1(in1, in2, out, control);
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

//1 BIT 2x1 MUX MODULE
module bit_mux2x1(in1, in2, out, control);
	input  in1, in2;
	input control;
	output reg  out;
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