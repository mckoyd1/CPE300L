
module clk1kHz(input  logic clk, reset, output logic clkout);
    
  logic [15:0] count = 0;   // Counter to count clock cycles (e.g., 20 bits for 1M counts)
  int value = 1000; // The number of clock cycles for 1 kHz
  int clk_20kHz = 20000;

/////////////////////////////////////////////////////////////

    // Frequency divider logic
    always @(posedge clk or posedge reset) begin

        if (reset) begin
            count <= 16'd0;
            clkout <= 1'b0;
        end 

        
        if (count == value) begin
                count <= 16'd0;
                clkout <= ~clkout;
            end 
        
            else begin
                count <= count + 1;

            end   
        
    end


endmodule