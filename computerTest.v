// RAM module testbench
// 2/23/2022
// peter Sanchez

  
module Computer;
  reg CLK, StopPC, ResetPC, PRAMWrite;
  reg [0:3] Input, PRAMAddress;
  reg [0:7] PRAMData;
  wire [0:3] Output;
  FourBitComputer Computer(CLK, StopPC, ResetPC, PRAMAddress, PRAMData, PRAMWrite, Input, Output);
   always #5 CLK = ~CLK;
 // reg Accumulator;

	initial begin
      CLK = 1'b0;
      StopPC = 1;
      
      ResetPC = 1;
      #5
      ResetPC = 0;
      
       /*
        This program will add 2 numbers and output the sum in an infinite loop
    PC ROM Value  ASM 
    
    0  0b10000000 INP 0000   ; Input a value, 0000 because INP doesnt need an operand
    1  0b00110001 STO 0001   ; Store the value inputed to address 1
    2  0b10000000 INP 0000   ; Input another value
    3  0b00010001 ADD 0001   ; sum of 2 inputed values, the first value entered is in address 1 and the second is still in the accumulator
    4  0b10010000 OUT 0000   ; output the sum, the accumulator is written to the output register, 0000 because OUT doesnt need an operand
    5  0b01010000 B   0000   ; unconditional branch to set program counter to ROM address 0 to create infinite loop
    */
      
      PRAMAddress = 4'b0000; PRAMData = 8'b10000000; #5 PRAMWrite = 1; #1 PRAMWrite = 0;
      PRAMAddress = 4'b0001; PRAMData = 8'b00110001; #5 PRAMWrite = 1; #1 PRAMWrite = 0;
      PRAMAddress = 4'b0010; PRAMData = 8'b10000000; #5 PRAMWrite = 1; #1 PRAMWrite = 0;
      PRAMAddress = 4'b0011; PRAMData = 8'b00010001; #5 PRAMWrite = 1; #1 PRAMWrite = 0;
      PRAMAddress = 4'b0100; PRAMData = 8'b10010000; #5 PRAMWrite = 1; #1 PRAMWrite = 0;
      PRAMAddress = 4'b0101; PRAMData = 8'b01010000; #5 PRAMWrite = 1; #1 PRAMWrite = 0;
      PRAMAddress = 4'b0110; #5
      PRAMAddress = 4'b0111; #5
      PRAMAddress = 4'b1000; #5
      PRAMAddress = 4'b1001; #5
      PRAMAddress = 4'b1010; #5
      PRAMAddress = 4'b1011; #5
      PRAMAddress = 4'b1100; #5
      PRAMAddress = 4'b1101; #5
      PRAMAddress = 4'b1110; #5
      PRAMAddress = 4'b1111; #5
      
      
      StopPC = 0;
      
      ResetPC = 1;
      #5
      ResetPC = 0;
      PRAMAddress= 4'b0000;
      Input = 4'b0110;
      #50
      $display ("value is %d", Output);  // initial value
      #300
     Input = 4'b0010;
	#200
	
      $finish;
	end


	//always@(posedge clk)
	//begin
     // $display ("value is %d", q);
     
	//end
  
  	

    
  //enabling the wave dump
  initial begin 
    $dumpfile("dump.vcd"); $dumpvars;
  end

endmodule
