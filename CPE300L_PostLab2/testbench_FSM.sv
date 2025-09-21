// 4.39: testbench example 3

module testbench_fsm();
  logic        clk, reset;
  logic        ss;
  logic [31:0] cnt;
  logic go, capture;

  // instantiate device under test
  FSM dut(clk, reset, ss, cnt, go, capture);

  // generate clock
  always 
    begin
      clk = 1; #5; clk = 0; #5;
      cnt = $urandom%5000; #10; 
    end

  // at start of test, load vectors
  // and pulse reset
  initial
    begin
    reset = 1; #10; reset = 0;
    ss = 1; #10;
    
    ss = 0; #10;
    ss = 1; #10;
    end

endmodule
