//1kHz testbench 

module testbench_clk1kHz();
  logic        clk, reset;
  logic        clkout;


  // instantiate device under test
  clk1kHz dut(clk, reset, clkout);


  // generate clock
  always 
    begin 
      clk = 1; #5 clk = 0;
    end


 initial 
    begin
      reset = 1; #5 reset = 0;  
    end

  //toggle clock at positive clock edge
  always @(posedge clk)
    begin
    //reset = 1; #5 reset = 0;
    clk = 1; #5; clk = 0; #5;

    end

endmodule
