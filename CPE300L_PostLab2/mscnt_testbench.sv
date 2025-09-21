//millisecond counter testbench 

module testbench_mscnt();
  logic        clk, reset;
  logic        [31:0] cnt;


  // instantiate device under test
  mscnt dut(clk, reset, cnt);


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

