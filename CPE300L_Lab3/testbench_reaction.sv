//reaction timer testbench (lab 3)

module testbench_reaction();
  logic        clk, reset;
  logic        ss, go;
  logic [10:0] reactionTime;

  // instantiate device under test
  reaction dut(clk, reset, ss, go, reactionTime);

  // generate clock
  always 
    begin
      clk = 1; #5; clk = 0; #5;
      //ss = 1; #5;  ss = 0; #5;
    //cnt = $urandom%5000; #10; 
    end

  // at start of test, load vectors
  // and pulse reset
  initial
    begin
    reset = 1; #5; reset = 0;
    ss = 1; #10;
    
   
    
    /* ss = 0; #10;
    ss = 1; #10;
    ss = 0; #10;
    ss = 1; #10;
    ss = 1; #10;
    
    ss = 0; #10;
    ss = 1; #10;
    ss = 0; #10;
    ss = 1; #10;
    ss = 1; #10;
    
    ss = 0; #10;
    ss = 1; #10;
    ss = 1; #10; */
    
    
    end

endmodule

