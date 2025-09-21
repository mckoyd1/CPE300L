////////////////////////////////////////////////////
//32-bit ALU
//Darryll Mckoy
//CPE300L 1 ALU and testbenches


module alu(input logic [31:0] A, B,
input logic [2:0] ALUControl,
output logic [31:0] Result,
output logic [3:0] ALUFlags);

logic [32:0] Carry;
//initial begin

//ALUFlags = 0;
//end

////////////////////////////////////////////////////
always @(*) //include all assignments as combinational logic


case (ALUControl)
3'b000: begin
	Carry = A + B; 			//Carry is 1 bit larger than A and B so MSB will change to 1 if overflow.
	Result = Carry[31:0];		//We only want the first 31 bits. 
	ALUFlags[1] = Carry[32];	//sets overflow flag 
	ALUFlags[3] = Result[31]; 	//Will set negative flag if MSB is 1.

if ( A[31] == B[31] && A[31] != Carry[31]) begin
	ALUFlags[0] = 1'b1;		//sets carry flag to 1. 
end

else begin
	ALUFlags[0] = 1'b0;	//sets carry flag to zero. 
end

end


/////////////////////////////////////////////////
3'b001: begin
	Carry = A - B; 			//Carry is 1 bit larger than A and B so MSB will change to 1 if overflow.
	Result = Carry[31:0];		//We only want the first 31 bits. 
	//Result = A - B;
	ALUFlags[3] = Result[31]; 	//Will set negative flag if MSB is 1.

if ( A[31] == B[31] && A[31] != Carry[31]) begin
	ALUFlags[1] = 1'b1;		//sets carry flag to 1. 
end

else begin
	ALUFlags[1] = 1'b0;	//sets carry flag to zero. 
end

end

/////////////////////////////////////////////
3'b010: Result = A & B;			//AND
3'b011: Result = A | B;			//OR
3'b100: Result = A ^ B;			//XOR
3'b101: Result = (A < B) ? A : B;	//SLT

default: Result = 32'b0;

endcase

//////////////////////////////////////////////
always_comb begin

if (Result == 32'b0) begin   //Zero flag
	ALUFlags[2] = 1'b1;		//sets zero flag to 1 if result is zero. 
end

else begin
	ALUFlags[2] = 1'b0;	//sets zero flag to zero if > zero. 
end


end
endmodule



