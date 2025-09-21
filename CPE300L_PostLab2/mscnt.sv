module mscnt(input  logic clk, reset, 
output logic [31:0] cnt);


logic [15:0] count = 0;   // Counter to count clock cycles 
  int value = 1000; // The number of clock cycles for 1 kHz
  int clk_20kHz = 20000;

/////////////////////////////////////////////////////////////

    
    always @(posedge clk or posedge reset) begin

        if (reset) begin
            count <= 16'd0;
            //clkout <= 1'b0;
            cnt <= 32'b0;
        end 

        
        if (count == value) begin
                count <= 16'd0;
                //clkout <= ~clkout;
                cnt <= cnt + 1;
            end 
        
            else begin
                count <= count + 1;

            end   
        
    end


endmodule
