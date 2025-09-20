/* random counter
This loops from 0 - 2000 */


module random_cnt(input  logic random_clk, reset, 
output logic [10:0] cnt);

  logic [10:0] rnd_count = 0;
  int value = 2000; // The number of clock cycles for 2000ms


/////////////////////////////////////////////////////////////

    
    always @(posedge random_clk or posedge reset) begin

        if (reset) begin
            rnd_count <= 11'b0;
            cnt <= 11'b0;
        end 

        
        if (rnd_count == value) begin
            cnt <= 0;
            rnd_count <= 11'b0;
        end 
        
        else begin
            rnd_count <= rnd_count + 1;
            cnt <= rnd_count;
        end   
        
    end


endmodule
