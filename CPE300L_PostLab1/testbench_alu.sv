//ALU testbench 

module testbench_alu();
  logic        clk, reset;
  logic [31:0] A, B, Result, ResultExpected;
  logic [2:0]  ALUControl;
  logic [3:0]  ALUFlags;
  logic [31:0] vectornum, errors;
  logic [250:0]  testvectors[10000:0];

  // instantiate device under test
  alu dut(.A(A), .B(B), .ALUControl(ALUControl), .Result(Result), .ALUFlags(ALUFlags));

  // generate clock
  always 
    begin
      clk = 1; #5; clk = 0; #5;
    end

  // at start of test, load vectors
  // and pulse reset
  initial
    begin
      $readmemh("alu.txt", testvectors);
      vectornum = 0; errors = 0;
      reset = 1; #15; reset = 0; //#30;
    end

  // apply test vectors on rising edge of clk
  always @(posedge clk)
    begin
      #1; {ALUControl, A, B, ResultExpected, ALUFlags} = testvectors[vectornum];
    end

  // check results on falling edge of clk
  always @(negedge clk)
    if (~reset) begin // skip during reset
      if (Result !== ResultExpected) begin  // check result
        $display("Error: inputs = %h", {ALUControl, A, B});
        $display("  outputs = %h (%h expected)",Result, ResultExpected);
        errors = errors + 1;
      end


      vectornum = vectornum + 1;

      if (testvectors[vectornum] === 27'hx) begin 
        $display("%h tests completed with %h errors", 
	           vectornum, errors);
        $finish;
      end
end    
endmodule

