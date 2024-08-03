module cache_table( input         clk, 
                    input         rst,
                    input         RFWr, 
                    input  [7:0]  Addr, 
                    input         WD, 
                    output reg    RD);

  reg [255:0] rf;

  always @(posedge clk, posedge rst)begin
    if (rst) begin    //  reset
      rf <= 256'b0;
    end     
    else begin
      if (RFWr) begin
        rf[Addr] <= WD;
      end
      else begin
        RD <= rf[Addr];
      end
    end
  end

  //assign RD = rf[Addr];
endmodule 
