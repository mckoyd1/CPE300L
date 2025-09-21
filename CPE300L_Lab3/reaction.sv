//Structural system verilog reaction timer Code (Lab 3)
module reaction(	input logic clk, reset,
			input logic ss, 
			output logic go, 
			output logic [10:0] reactionTime
);

	//internal signals
	logic clk1k;
	logic [31:0] msCnt;
	logic [10:0] reactionCnt;
	logic [10:0] rndmCnt;
	logic capture;

	//1 kHz clock submodule
	clk1kHz u_div (
	.clk(clk),
	.reset(reset),
	.clkout(clk1k)
);

//millisecond counter submodule
random_cnt u_reaction (
	.random_clk(clk1k),
	.reset(reset),
	.cnt(reactionCnt)
);

//secondary counter for random time
random_cnt u_rand (
	.random_clk(clk1k),
	.reset(reset),
	.cnt(rndmCnt)
);

//FSM submodule
reaction_FSM u_fsm (
	.clk(clk1k),
	.reset(reset),
	.ss(ss),
	.cnt(rndmCnt),
	.go(go),
	.capture(capture)
);

//reaction time register
always_ff @(posedge clk1k or posedge reset) begin
	if (reset) begin
		reactionTime <= 0;
	end
	else if (capture) reactionTime <= reactionCnt;
end

endmodule

