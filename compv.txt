// Code your design here

// counter
module counter(clk,reset, A, STOP, LD,count);
  input clk,reset,LD, STOP;
  input [3:0] A;
  output reg [3:0] count;
  // counter wants rising clock edge
  initial count = 4'b0000;
  always@(posedge clk) begin
    if(reset)
      count <= 4'b0000;
    else if(LD)
      count <= A;   // load count for jump    
    else if (!STOP)
      count <= count + 1;
    end
endmodule



module Pcounter(clk,reset, A, STOP, LD,count);
  input clk,reset,LD, STOP;
  input [3:0] A;
  output reg [3:0] count;
  // counter wants rising clock edge
  initial count = 4'b0000;
  always@(posedge clk) begin
    $display("here");
    if(reset)
      count <= 4'b0000;
    else if(LD)
      count <= A;   // load count for jump    
    else if (!STOP)
      count <= count + 1;
    end
endmodule


// SR latch


module SRLatch(S, R, Q, Qprime);
  input S, R;
  output reg Q, Qprime;
  always @( S or R) begin  // the sr latch operates continuously
    // ignore cases where S == R
  	if (S == 1 & R == 0) begin
         Q <= 1;
         Qprime <= 0;
    end
    else if((S == 0) & (R == 1)) begin
       Q <= 0;
       Qprime <= 1;
    end
  end
endmodule




// ram module
module RAM(Write, Address, DataIn, DataOut);
  input Write;
  input [0:3] Address;
  input [0:3] DataIn;
  output reg [0:3] DataOut;
  reg [0:3] RamArr[0:15];     // 4 bit by 16 array
  always @(Write or Address) begin
    if ( Write == 1) begin     // write on high
      RamArr[Address] = DataIn; // write operation
      DataOut = 4'b1111;        // values shown during write operation
    end
    else begin // read
      DataOut = RamArr[Address];
    end
  end
endmodule


//
module InstructionDecoder(DataIn, HLT, ADD, SUB, STO, LD, B, BZ, LDV, INP, OUT, AND, OR, NOT);
  input [0:3] DataIn;
  output reg HLT, ADD,SUB, STO, LD, B, BZ, LDV, INP, OUT, AND, OR, NOT;
  
  always @(DataIn) begin
    case (DataIn) 
      4'b0000: begin // HLT
        HLT = 1;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0;
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b0001: begin // ADD
        HLT = 0;
        ADD = 1;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0;
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b0010: begin // SUB
        HLT = 0;
        ADD = 0;
        SUB = 1;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0;
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b0011: begin // STO
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 1;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0;
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b0100: begin // LD
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 1;
        B = 0;
        BZ = 0;
        LDV = 0;
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b0101: begin // B
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 1;
        BZ = 0;
        LDV = 0;
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b0110: begin // BZ
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 1;
        LDV = 0;
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b0111: begin // LDV
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 1;  
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b1000: begin // INP
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0; 
        INP = 1;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b1001: begin // OUT
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0; 
        INP = 0;
        OUT = 1;
        AND = 0;
        OR = 0;
        NOT = 0;
      end
      4'b1010: begin // AND
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0; 
        INP = 0;
        OUT = 0;
        AND = 1;
        OR = 0;
        NOT = 0;
      end
      4'b1011: begin // OR
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0; 
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 1;
        NOT = 0;
      end
      4'b1100: begin // NOT
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0; 
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 1;
      end
      4'b1101: begin
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0; 
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
        // free
      end
      4'b1110: begin
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0; 
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
        // free
      end
      4'b1111: begin
        HLT = 0;
        ADD = 0;
        SUB = 0;
        STO = 0;
        LD = 0;
        B = 0;
        BZ = 0;
        LDV = 0; 
        INP = 0;
        OUT = 0;
        AND = 0;
        OR = 0;
        NOT = 0;
        // free
      end
    endcase
  end
endmodule



module ALU(A, B, S, F, M, Cn);
  input M,Cn;
  input[0:3] A, B, S;
  output reg[0:3] F;
  always @(S) begin
      case (S) 
        4'b0000: begin
          if (M == 1) 
            F = ~A;  // ~ is bitwise,
          else if (Cn == 0) 
            F = A - 1;     
          else if (Cn == 1) 
            F = A;
        end

        4'b0001: begin
          if (M == 1) 
            F = ~(A & B);
          else if (Cn == 0) 
            F = ((A & B) - 1);
          else if (Cn == 1) 
            F = (A & B);
        end

        4'b0010: begin
          if (M == 1)
            F = ((~A) | B);
          else if (Cn == 0) 
            F = ((A & (~B)) - 1);
          else if (Cn == 1) 
            F = (A & (~B));
        end

        4'b0011: begin
          if (M == 1) 
            F = 1;
          else if (Cn == 0) 
            F = -1; // 2s comp
          else if (Cn == 1) 
            F = 0;
        end

        4'b0100: begin
          if (M == 1) 
            F = ~(A | B);
          else if (Cn == 0) 
            F = (A + (A | (~B)));
          else if (Cn == 1) 
            F = (A + (A | (~B)) + 1);
        end

        4'b0101: begin
          if (M == 1) 
            F = ~B;
          else if (Cn == 0) 
            F = ((A & B) + (A | (~B)));
          else if (Cn == 1) 
            F = ((A & B) + (A | (~B)) +1);
        end

        4'b0110: begin
          if (M == 1) 
            F = ~(A ^ B);
          else if (Cn == 0) 
            F = (A - B - 1);
          else if (Cn == 1) 
            F = (A - B);
        end
          
        4'b0111: begin
          if (M == 1) 
            F = (A | (~B));
          else if (Cn == 0) 
            F = (A | (~B));
          else if (Cn == 1) 
            F = ((A | (~B)) + 1);
        end

        4'b1000: begin
          if (M == 1) 
            F = ((~A) & B);
          else if (Cn == 0) 
            F = (A + (A | B));
          else if (Cn == 1) 
            F = (A + (A | B) + 1);
        end

        4'b1001: begin
          if (M == 1) 
            F = A ^ B;
          else if (Cn == 0) 
            F = (A + B);
          else if (Cn == 1) 
            F = (A + B + 1);
        end

        4'b1010: begin
          if (M == 1) 
            F = B;
          else if (Cn == 0) 
            F = ((A & ~(B)) + (A | B));
          else if (Cn == 1) 
            F = ((A & ~(B)) + (A | B) + 1);
        end

        4'b1011: begin
          if (M == 1) 
            F = (A | B);
          else if (Cn == 0) 
            F = (A | B);
          else if (Cn == 1) 
            F = ((A | B) + 1);
        end

        4'b1100: begin
          if (M == 1)
            F = 0;
          else if (Cn == 0) 
            F = (A + (A << 1));
          else if (Cn == 1) 
            F = (A + A + 1);
        end

        4'b1101: begin
          if (M == 1) 
            F = (A & ~B);
          else if (Cn == 0) 
            F = ((A & B) + A);
          else if (Cn == 1) 
            F = ((A & B) + A + 1);
        end

        4'b1110: begin
          if (M == 1) 
            F = (A & B);
          else if (Cn == 0) 
            F = ((A & B) + A);
          else if (Cn == 1) 
            F = ((A & B) + A + 1);
        end
        4'b1111: begin
          if (M == 1) 
            F = A;
          else if (Cn == 0) 
            F = A;
          else if (Cn == 1) 
            F = A + 1;
        end
    endcase
  end
endmodule


module ALUEncoder(ADD, SUB, STO, LD, LDV, INP, OUT, AND, OR, NOT, ALUOut);
  input ADD, SUB, STO, LD, LDV, INP, OUT, AND, OR, NOT;
  output reg [0:5] ALUOut;
  always @(ADD or SUB or STO or LD or LDV or INP or OUT or AND or OR or NOT) begin
    if(ADD) 
      ALUOut = 6'b100100;
   
    else if (SUB) 
      ALUOut = 6'b011001;
    
    else if (STO || OUT ) 
      ALUOut = 6'b000001;
  
    else if (LD || LDV || INP) 
      ALUOut = 6'b101010;
  
    else if (AND) 
      ALUOut = 6'b111010;
  
    else if (OR) 
      ALUOut = 6'b101110;
  
    else if (NOT) 
      ALUOut = 6'b000010;
  end
endmodule


module FourBitComputer(CLK, StopPC, ResetPC, PRAMAddress, PRAMData, PRAMWrite, Input, Output);
  input CLK, StopPC, ResetPC, PRAMWrite;
  input [0:3] Input, PRAMAddress;
  input [0:7] PRAMData;
  output reg [0:3] Output;
  
 // reg Accumulator;
  
  reg Branch;
  
  always @ ( BZ or B) begin
    if(B || (BZ && (Accumulator == 0)))
      Branch = 1;
    else
      Branch = 0;
  end
  
  
  //assign Branch = (B || BranchZero);
  //counter(clk,reset, A , STOP  , LD,count);
 
  
  //SRLatch(S, R, Q, Qprime);  
  wire Q, Qprime;
  wire subCRST;
  
  
  assign subCRST = ((SubCountOut & 4'b0100) >> 2);
  wire[0:3] PCAddress;
  Pcounter PC(Qprime, ResetPC, RamOut, StopPC, Branch, PCAddress);
  SRLatch latch(CLK,subCRST , Q, Qprime);
  
  reg QQ;
  
  initial QQ = 1'b0;
  
  always@(posedge Qprime) begin
    $display ("hhhhh");
   
  end
  wire QQQ;
  assign QQQ = QQ;
  
  
  
  
  
   wire [0:3] SubCountOut;
  counter SubCounter(CLK, Qprime | ResetPC, 4'b0000, StopPC, Branch, SubCountOut);
    
  wire [0:3] Instruction;
  wire [0:3] Data;
  wire [0:3] PInstruction;
  wire [0:3] Pdata;
  assign PInstruction = PRAMData & 8'b00001111;
  assign Pdata = (PRAMData & 8'b11110000) >> 4;
    RAM InstructionRAM(PRAMWrite, PRAMAddress | PCAddress,PInstruction , Instruction);
  RAM DataRAM(PRAMWrite, PRAMAddress | PCAddress, Pdata , Data);
    

  wire HLT, ADD, SUB, STO, LD, B, BZ, LDV, INP, OUT, AND, OR, NOT;
  InstructionDecoder ID(Instruction, HLT, ADD, SUB, STO, LD, B, BZ, LDV, INP, OUT, AND, OR, NOT);


  reg [0:3] ALUB;
  always @ (ADD or SUB or LD or AND or OR or LDV or INP) begin
    if(ADD || SUB || LD || AND || OR) 
      ALUB = RamOut;
    else if(LDV) 
      ALUB = Data;
    else if(INP) 
      ALUB = Input;
  end 
     
     
 
   reg EncoderVal;
  //ALUEncoder(ADD, SUB, STO, LD, LDV, INP, OUT, AND, OR, NOT, EncoderVal);
  //ALU alu1(A, B, S, F, M, Cn);
  wire [0:3] ALUOut;
  ALU alu1(Accumulator, ALUB, ((EncoderVal & 6'b111100) >> 2),ALUOut,((EncoderVal & 6'b000010) >> 1), (EncoderVal & 6'b000001));
   //


  wire writeAccumALUreg;
  assign writeAccumALUreg = (INP || ADD || SUB || LD || LDV || AND || OR || NOT);
  wire writeAccum;
  assign writeAccum = ((SubCountOut == 3) && writeAccumALUreg);

  reg [0:3] Accumulator; 
  always @ (posedge CLK) begin
    if(writeAccum)
      Accumulator = ALU_Register;
  end
  
  
  wire writeALUreg;
  assign writeALUreg = ((SubCountOut) && writeAccumALUreg);
  reg [0:3] ALU_Register;
  always @ (posedge CLK) begin
    if(writeALUreg)
      ALU_Register = ALUOut;
  end
  
  
  
  wire writeOUTreg;
  assign writeOUTreg = ((SubCountOut) && OUT);
 
  always @(posedge CLK) begin
    if (writeOUTreg)
      Output = ALUOut;
  end
  
  
  //module RAM(Write, Address, DataIn, DataOut);
  wire writeRam;
  assign writeRam = ((SubCountOut) && STO);
  wire [0:3] RamOut;
  RAM SystemRam(!(CLK && writeRam), Data, ALUOut, RamOut);
endmodule
