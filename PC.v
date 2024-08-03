module PC( clk, rst, NPC, PC, write, flush);

  input              clk;
  input              rst;
  input              write;
  input              flush;
  input       [31:0] NPC;
  output reg  [31:0] PC;

//warning!!
  //always @(posedge clk, posedge rst)begin
   always @(posedge clk)begin
    if (rst) 
//      PC <= 32'h0000_0000;
      PC <= 32'h1c000000;
      //PC <= 32'h1BFFFFFC;
    else
    begin
        if(write) begin 
          if(flush) begin PC <= 32'b0; end
          else begin PC <= NPC; end
        end
        //PC <= NPC;
    end
  end
  
endmodule

