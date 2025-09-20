//CPE 300L, Lab3 reaction_FSM

module reaction_FSM(input  logic clk, reset, 
           input  logic ss,
	   input  logic [10:0] cnt,	  
           output logic go, capture);


  typedef enum logic [4:0] {S0, S1, S2, S3, S4} statetype;
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

    ////////////S0 press ss to start system /////////////////////////////
      S0:      if (ss) begin
                capture = 1'b0;
                nextstate = S1;    //if ss pressed then move on to next state to start clock divider;
                $display("ss pressed moving to state S1");
      end
	       else begin
        nextstate = S0;	  //if ss not pressed then stay in current state. 
        $display("ss not pressed staying in state S0");
        end

    /////////// S1 start clock divider (50mhz to 1khz) in this state /////
      S1:      if (!ss) begin
                nextstate = S2;
                $display("initializing random counter");
      end
	       else begin
           nextstate = S1;
           $display("waiting for clk divider to complete");
         end

    /////////// S2 start random counter (loops from 0-2000) /////
      S2:      if (cnt == 1'd0) begin
                nextstate = S3;
                $display("cnt equal to 0 asserting go and moving to state S3");
      end
	       else begin
           nextstate = S2;
           $display("waiting for cnt to reach zero!");
         end

    ////////// S3 assert go (board LED gives signal) start reaction timer ///////

      S3:  begin
         nextstate = S4;
         $display("starting reaction timer and moving to state S4"); 
      end


    ////////// S4 capture reaction time, set go = 0, capture = 1 ////////////

      S4:      if (ss) begin
         capture = 1'b1;
         //go <= ~go;
         nextstate = S0;
         $display("ss pressed asserting capture, outputting reaction time and moving to state S0"); 
      end

		else begin
      nextstate = S4;
      $display("ss not pressed staying in state S4");
    end



      default: nextstate = S0;
    endcase

  // output logic
  assign go = (state == S3);

endmodule
