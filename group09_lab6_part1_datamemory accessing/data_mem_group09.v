
//DATA MEMORY MODULE
module data_mem(WRITE,READ,RESET,CLK,ADDRESS,WRITEDATA,READDATA,BUSYWAIT);
    /* INPUT/ OUTPUT PORT DECLARATION
    WRITE = 1, for swd,swi
    READ = 1, for lwd,lwi
    ADDRESS = ALU RESULT
    WRITEDATA = Data to be written to memory (swd,swi)
    READDATA = Data read from memory (lwd,lwi)
    BUSYWAIT = 1 when read / write in process
    */
    input WRITE,READ,RESET,CLK;
    input [7:0] ADDRESS,WRITEDATA;
    output reg [7:0] READDATA;
    output reg BUSYWAIT;

    //Instantiate a memory array of 256 registers
    reg [7:0] mem_array [0:255];
    //readaccess = 1 when READ = 1 and WRITE = 0
    //writeaccess = 1 when READ =0 and WRITE = 1
    reg readaccess, writeaccess;

    //testing - dump the values of the memory array to the gtkwave file
    initial
	begin 
		$dumpfile("cpu_wavedata.vcd");
		for(i=0;i<256;i++)
			$dumpvars(1,mem_array[i]);

	end

    //Set BUSYWAIT,readaccess,writeaccess for lwd,lwi,swi,swd commands
    always @(READ, WRITE)
    begin
        BUSYWAIT = (READ || WRITE)? 1 : 0;
        readaccess = (READ && !WRITE)? 1 : 0;
        writeaccess = (!READ  && WRITE)? 1 : 0;
    end

    //Read from memory or write to memory at a positive clock edge
    always @(posedge CLK)
    begin
        if(readaccess)
        begin
            READDATA = #40 mem_array[ADDRESS];
            BUSYWAIT = 0;
            readaccess = 0;
        end
        if(writeaccess && !RESET)
        begin
            mem_array[ADDRESS] = #40 WRITEDATA;
            BUSYWAIT =  0;
            writeaccess = 0;
        end
    end

    integer i;

    //Reset memory
    always @(posedge CLK)
    begin
        if (RESET)
        begin
            for (i=0;i<256; i=i+1)
                mem_array[i] = 0;
            
            BUSYWAIT = 0;
            readaccess = 0;
            writeaccess = 0;
        end
    end
endmodule
