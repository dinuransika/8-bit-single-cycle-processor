//CO224 VERILOG LAB 5- PART 2
// 8 x 8 REGISTER FILE
//GROUP 09

//TOP LEVEL STIMULUS MODULE
/*
module test;
	//Input variable declaration
	//WRITEDATA = 8 Bit data input
	//WRITEREG = 3 bit write address
	//READREG1 & READREG2 = 3 bit read addresses
	//CLK = Clock signal
	//WRITEENABLE = Write enable signal (When WRITE =1 at positive clk edge) 
	//RESET = Reset registers signal (When RESET =1 at positive clk edge) 
	reg signed [7:0] WRITEDATA;
	reg [2:0] WRITEREG,READREG1,READREG2;
	reg CLK,WRITEENABLE,RESET;

	//Output variable declaration
	//REGOUT1 , REGOUT2 = 8 bit data stored in registers
	wire signed [7:0] REGOUT1,REGOUT2;
	
	//Monitor changes in variables
	initial 
	$monitor($time," CLK %d RESET %d WRITE %d OUT1ADDRESS %d OUT2ADDRESS %d OUT1=%d OUT2 = %d",CLK,RESET,WRITEENABLE,READREG1,READREG2,REGOUT1,REGOUT2);

	//Instantiate reg_file module
	reg_file r1(WRITEDATA,REGOUT1,REGOUT2,WRITEREG,READREG1,READREG2,WRITEENABLE,CLK,RESET);

	//Generate the CLK signal with period 10
	
	initial
	begin
		CLK=1'b1;
		forever #5 CLK =~CLK;
	end

	//Finish the simulation after 100 time units
	initial
	begin
		#100 $finish;
	end


        //WRITE VARIABLE
	initial
	begin
		WRITEENABLE = 0;
		#5 WRITEENABLE = 1;
	       	#40 WRITEENABLE = 0;
		#20 WRITEENABLE = 1;	
	
	end
	
	//RESET VARIABLE
	initial
	begin
		RESET = 1;
		#15 RESET = 0;
		#50 RESET = 1;
	        #10 RESET = 0;
	
	end

	//WRITING INPUTS (WRITEDATA AND WRITEREG)
	initial
	begin
		WRITEDATA = -23;
	       	WRITEREG = 3;
		#25;
		WRITEDATA = 35;
		WRITEREG = 6;
		#10;
		WRITEDATA = 20;
		WRITEREG = 2;
		#5
		WRITEDATA = 45;
		WRITEREG = 2;
		#20;
		WRITEDATA = 5;
		WRITEREG = 0;
		#25;
		WRITEDATA = 12;
		WRITEREG = 7;

	end

	//READING REGISTER VARIABLES
	initial
	begin
		READREG1 = 0;
		READREG2 = 3;
		#25;
	        READREG1 = 6;
		#10;
		READREG2 = 2;
		#20;
		READREG1 = 0;	
		#25;
		READREG2 = 7;
	
	end
	
	//Generate the wave file
	initial
	begin
		
		$dumpfile("reg.vcd");
		$dumpvars(0,test);
	end

endmodule


//REGISTER FILE MODULE
*/
module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET);

	//INPUT PORT DECLARATION
	//IN = 8 bit write data
	//INADDRESS = 3 bit write address
	//OUT1ADDRESS, OUT2ADDRESS = 3 bit read addresses
	
	input signed [7:0] IN;
	input [2:0] INADDRESS,OUT1ADDRESS,OUT2ADDRESS;
	input CLK,WRITE,RESET;

	//OUTPUT PORT DECLARATION
	//OUT1,OUT2 = 8 bit read data
	output signed [7:0] OUT1,OUT2;
	
	//Register type array of 8 elements to store 8 bit register data
	reg signed [7:0] regfile [0:7];

	//Generate the wave file which includes the values stored in each register	
	//Implemented for testing purposes
	initial
	begin 
		$dumpfile("cpu_wavedata.vcd");
		for(i=0;i<8;i++)
			$dumpvars(1,regfile[i]);

	end

	//READ REGISTERS ASYNCHRONOUSLY
	//OUT1 AND OUT2 MUST CHANGE IF OUT1ADDRESS,OUT2ADDRESS OR THE VALUES IN RELEVANT REGISTERS CHANGE
	//OUT1 AND OUT2 MUST CHANGE AFTER A DELAY OF 2,UNTIL IT CHANGES IT SHOULD DISPLAY THE PREVIOUS VALUE STORED IN THEM
	//DELAY = 2
	assign #2 OUT1 =  regfile[OUT1ADDRESS];
	assign #2 OUT2 =  regfile[OUT2ADDRESS];
	//WRITE TO REGISTER AT THE POSITIVE EDGE OF CLK AND WHEN WRITE=1 AND RESET = 0
	//DELAY = 1
	always @(posedge CLK)
	begin
		if (WRITE && RESET == 1'b0)
		/*begin
			regfile[INADDRESS] <= #1 IN;
		end*/
			#1 regfile[INADDRESS] =  IN;
	end
	 
	//RESET THE REGISTERS WHEN RESET=1 AND CLK IS IN POSITIVE EDGE
	//DELAY = 1
	integer i;
	always @(posedge CLK)
	begin
		if (RESET)
			#1
			begin
				for(i=0;i<8;i=i+1)
					/*begin
						regfile [i] <= #1 0;
					end*/
						regfile[i]  = 0;
			end
	end


endmodule


