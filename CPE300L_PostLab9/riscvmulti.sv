// riscvmulticycle processor.sv



module testbench_MultiCycle();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 25) begin
          $display("Simulation succeeded");
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, Adr, 
           output logic        MemWrite);

  logic [31:0] PC, Instr, ReadData, SrcA, SrcB, ALUResult;
  
  // instantiate processor and memories
  riscvmulti rvmulti(clk, reset, PC, SrcA, SrcB, Instr, MemWrite, Adr,
                       WriteData, ALUResult, ReadData);

  //imem imem(Adr, );
  dmem dmem(clk, MemWrite, Adr, WriteData, ReadData);
endmodule

module riscvmulti(input  logic        clk, reset,
                   output logic [31:0] PC, SrcA, SrcB,
                   output  logic [31:0] Instr,
                   output logic        MemWrite,
                   output logic [31:0] adr, WriteData, ALUResult,
                   input  logic [31:0] ReadData);

  logic       RegWrite, PCWrite, Zero, AdrSrc, IRWrite;
  logic [1:0] ResultSrc, ALUSrcA, ALUSrcB;
  logic [2:0] ImmSrc;
  logic [2:0] ALUControl;

  controller c( clk, reset, Instr[6:0], Instr[14:12], Instr[30], Zero,
             ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl, IRWrite,
             PCWrite, RegWrite, MemWrite);

  datapath dp(clk, reset, ResultSrc, AdrSrc,
              ALUSrcB, ALUSrcA, RegWrite, IRWrite, PCWrite,
              ImmSrc, ALUControl,
              Zero, PC, SrcA, SrcB, Instr,
              adr, WriteData, ALUResult, ReadData);
endmodule

/*======================================================
=============== Controller Module ======================
========================================================*/

module controller(input  logic       clk,
                  input  logic       reset,
                  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [2:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc,
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IRWrite, PCWrite,
                  output logic       RegWrite, MemWrite);

  logic [1:0] ALUOp;
  logic       Branch;

  MainFSM mf(clk, reset, op, Branch, PCUpdate, RegWrite, MemWrite, IRWrite, ResultSrc, 
              ALUSrcA, ALUSrcB, AdrSrc, ALUOp);

  aludec  ad(op[5], funct3, funct7b5, ALUOp, ALUControl);
  InstrDec Id(op, ImmSrc);

  assign PCWrite = Branch & Zero | PCUpdate;
endmodule



/*==================================================
============= Main FSM Module ======================
====================================================*/

module MainFSM(input  logic       clk,
               input  logic       reset,
               input  logic [6:0] op,
               output logic       Branch,
               output logic       PCUpdate,
               output logic       RegWrite,
               output logic       MemWrite,
               output logic       IRWrite,
               output logic [1:0] ResultSrc,
               output logic [1:0] ALUSrcA, ALUSrcB,
               output logic       AdrSrc,
               output logic [1:0] ALUOp);




 typedef enum logic [3:0] { S0_FETCH, S1_DECODE, S2_MemAdr, S3_MemRead,
                            S4_MemWB, S5_MemWrite, S6_ExecuteR, S7_ALUWB,
                            S8_ExecuteI, S9_JAL, S10_BEQ} statetype;

    statetype current_state, next_state;

    // State register
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= S0_FETCH;
        end else begin
            current_state <= next_state;
        end
    end

    // Next state logic and control signal generation
    always_comb begin
    

        case (current_state)
            S0_FETCH: begin
                AdrSrc      = 1'b0;
                IRWrite     = 1'b1; // Write instruction to IR
                ALUSrcA     = 2'b00;
                ALUSrcB     = 2'b10;
                ALUOp       = 2'b00;
                ResultSrc   = 2'b10;
                PCUpdate    = 1'b1; // Increment PC
                MemWrite    = 1'b0;
                RegWrite    = 1'b0;
                next_state  = S1_DECODE;
            end

            S1_DECODE: begin
                // Decode instruction and determine next state based on opcode
                ALUSrcA     = 2'b01;
                ALUSrcB     = 2'b01;
                ALUOp       = 2'b00;
                ResultSrc   = 2'b00;
                IRWrite     = 1'b0; // Write instruction to IR
                PCUpdate    = 1'b0; // Increment PC

                case (op)
                    7'b0000011: begin // LW (Load Word)
                        next_state = S2_MemAdr;
                    end
                    7'b0100011: begin //SW (Store Word)
                        next_state = S2_MemAdr;
                    end
                    7'b0110011: begin // R-Type
                        next_state = S6_ExecuteR;
                    end
                    7'b0010011: begin // I-Type ALU
                        next_state = S8_ExecuteI;
                    end
                    7'b1101111: begin // JAL
                        next_state = S9_JAL;
                    end
                    7'b1100011: begin // beq
                        next_state = S10_BEQ;
                    end
                    default: begin // Handle unsupported opcodes
                        next_state = S0_FETCH; // Or an error state
                    end
                endcase
            end

            S2_MemAdr: begin
                ALUSrcA = 2'b10;
                ALUSrcB = 2'b01;
                ALUOp   = 2'b00;

                case (op)
                    7'b0000011: begin // LW (Load Word)
                        next_state = S3_MemRead;
                    end
                    7'b0100011: begin //SW (Store Word)
                        next_state = S5_MemWrite;
                    end
                    default: begin // Handle unsupported opcodes
                        next_state = S0_FETCH; // Or an error state
                    end
                endcase

                //next_state       = S_FETCH_0;
            end

            S3_MemRead: begin
                ResultSrc = 2'b00; 
                AdrSrc    = 1'b1; 
                ALUSrcA   = 2'b00;
                ALUSrcB   = 2'b00;

                next_state      = S4_MemWB;      
            end

            S4_MemWB: begin
                ResultSrc       = 2'b01; //Update result source
                RegWrite        = 1'b1;  //Update address source
                AdrSrc          = 1'b0; 

                next_state      = S0_FETCH;      
            end

            S5_MemWrite: begin
                ResultSrc = 2'b00; // Read_data1 from RegFile
                AdrSrc    = 1'b1; // Sign-extended immediate
                MemWrite  = 1'b1;
                ALUSrcA   = 2'b00;
                ALUSrcB   = 2'b00; // Data from memory

                next_state       = S0_FETCH;      
            end

            S6_ExecuteR: begin
                ALUSrcA  = 2'b10;
                ALUSrcB  = 2'b00; // Data from memory
                ALUOp    = 2'b10;

                next_state       = S7_ALUWB;
            end

            S7_ALUWB: begin
                ResultSrc = 2'b00;
                RegWrite  = 1'b1;
                ALUSrcA   = 2'b00;
                ALUSrcB   = 2'b00; // Data from memory
                ALUOp     = 2'b00;
                PCUpdate  = 1'b0; // Increment PC

                next_state       = S0_FETCH;
            end

            S8_ExecuteI: begin
                ALUSrcA  = 2'b10;
                ALUSrcB  = 2'b01; 
                ALUOp    = 2'b10; 

                next_state       = S7_ALUWB;
            end

            S9_JAL: begin
                ALUSrcA   = 2'b01;
                ALUSrcB   = 2'b10; 
                ALUOp     = 2'b00; 
                ResultSrc = 2'b00;
                PCUpdate  = 1'b1;

                next_state       = S7_ALUWB;
            end

            S10_BEQ: begin
                ALUSrcA   = 2'b10;
                ALUSrcB   = 2'b00; 
                ALUOp     = 2'b01; 
                ResultSrc = 2'b00;
                Branch    = 1'b1;

                next_state       = S0_FETCH;
            end

            default: begin
                next_state = S0_FETCH;
            end
        endcase
    end


endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // addition
      2'b01:                ALUControl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 3'b001; // sub
                          else          
                            ALUControl = 3'b000; // add, addi
                 3'b010:    ALUControl = 3'b101; // slt, slti
                 3'b110:    ALUControl = 3'b011; // or, ori
                 3'b100:    ALUControl = 3'b100; // xor
                 3'b111:    ALUControl = 3'b010; // and, andi
                 default:   ALUControl = 3'bxxx; // ???
               endcase
    endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc,  
                input  logic        AdrSrc, 
                input  logic [1:0]  ALUSrcB, ALUSrcA, 
                input  logic        RegWrite, IRWrite, PCWrite,
                input  logic [2:0]  ImmSrc,
                input  logic [2:0]  ALUControl,
                output logic        Zero,
                output logic [31:0] PC, SrcA, SrcB,
                output  logic [31:0] Instr, 
                output logic [31:0] Adr, WriteData, ALUResult,
                input  logic [31:0] ReadData);

  logic [31:0] PCTarget, rd1, rd2, A;
  logic [31:0] ImmExt;
  logic [31:0] ALUOut, Data;
  logic [31:0] Result, OldPC;

  // next PC logic
  flopenr #(32) pcreg(clk, reset, PCWrite, Result, PC); 
  mux2 #(32)  pcmux(PC, Result, AdrSrc, Adr);
  flopenr #(32) InstraDFlopPC(clk, reset, IRWrite, PC, OldPC); 
  flopenr #(32) InstraDFlopRD(clk, reset, IRWrite, ReadData, Instr); 
  flopr #(32) InstrDflop(clk, reset, ReadData, Data);
  
 
  // register file logic
  regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20], 
                 Instr[11:7], Result, rd1, rd2);
  extend      ext(Instr[31:7], ImmSrc, ImmExt);

  flopr #(32) alufloprd1(clk, reset, rd1, A);
  flopr #(32) alufloprd2(clk, reset, rd2, WriteData);
  
  // ALU logic
  mux3 #(32)  srcamux(PC, OldPC, A, ALUSrcA, SrcA);
  mux3 #(32)  srcbmux(WriteData, ImmExt, 32'd4, ALUSrcB, SrcB);
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, Zero);
  flopr #(32) aluflop(clk, reset, ALUResult, ALUOut);
  mux3 #(32)  resultmux(ALUOut, Data, ALUResult, ResultSrc, Result);
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [2:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      3'b000:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      3'b001:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      3'b010:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      3'b011:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 

      3'b100:   immext = {instr[31:12], 12'b0}; 
               // U-type (lui)
      default: immext = 32'bx; // undefined
    endcase             
endmodule


module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule


module flopenr #(parameter WIDTH = 8)
              (input  logic             clk, reset, en,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

   // asynchronous reset 
  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else if (en)    q <= d;
endmodule

/* module flopenr2x2 #(parameter WIDTH = 8)
              (input  logic             clk, reset, en,
               input  logic [WIDTH-1:0] d, c,  
               output logic [WIDTH-1:0] q, r);

   // asynchronous reset 
  always_ff @(posedge clk, posedge reset)
    if (reset)
        q <= 0,
        r <= 0;
   
    else if (en){ 
        q <= d, 
        r <= c;
endmodule */

/* module flopr2x2 #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, c,  
               output logic [WIDTH-1:0] q, r);

   // asynchronous reset 
  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0, r <= 0;
    else       q <= d, r <= c;
endmodule */


module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000:  result = sum;         // add
      3'b001:  result = sum;         // subtract
      3'b010:  result = a & b;       // and
      3'b011:  result = a | b;       // or
      3'b100:  result = a ^ b;       // xor
      3'b101:  result = sum[31] ^ v; // slt
      3'b110:  result = a << b[4:0]; // sll
      3'b111:  result = a >> b[4:0]; // srl
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule

/*====================================================
============== Instruction Decoder Module ============
======================================================*/

module InstrDec(input  logic  [6:0] op,
                output logic  [2:0] ImmSrc);

logic [10:0] controls;

    assign ImmSrc = controls; 

  always_comb
    case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_1; // sw
      7'b0110011: controls = 11'b1_00_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_01_1; // jal
      default:    controls = 11'b0_00_0_0_00_0_00_0; // non-implemented instruction
    endcase 


endmodule
