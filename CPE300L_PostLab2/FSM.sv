//CPE 300L, Lab2 FSM

module FSM(input  logic clk, reset, 
           input  logic ss, 
	   input  logic [31:0] cnt,	  
           output logic go, capture);

  typedef enum logic [1:0] {S0, S1, S2} statetype;
  statetype state, nextstate;

  // state register
  always_ff @(posedge clk, posedge reset)
    if (reset) begin
      state <= S0;
      $display("reset pressed moving to state S0");
    end

    else       state <= nextstate;

  // next state logic
  always_comb
    case (state)
      S0:      if (ss) begin
                nextstate = S1;    //if ss pressed then move on to next state and generate random value 0-3000;
                $display("ss pressed moving to state S1");
      end
	       else begin
        nextstate = S0;	  //if ss not pressed then stay in current state. 
        $display("ss not pressed staying in state S0");
        end

      S1:      if (cnt > 32'h0 && cnt < 32'hBB8) begin
                nextstate = S2;
                $display("cnt between 0-3000 asserting go and moving to state S2");
      end
	       else begin
           nextstate = S1;
           $display("Error! cnt not between 0-3000 staying in S1");
         end

      S2:      if (ss) begin
         nextstate = S0;
         $display("ss pressed asserting capture and moving to state S0"); 
      end

		else begin
      nextstate = S2;
      $display("ss not pressed staying in state S2");
    end

      default: nextstate = S0;
    endcase

  // output logic
  assign capture = (state == S0);
  assign go = (state == S2);
endmodule



//use $urandom%3000 to produce random values that are unsigned and between 0 - 3000. 
//use $random%3000 to produce random values that are signed and between 0 - 3000. 