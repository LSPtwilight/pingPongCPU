module PC( clk, rst, NPC, PC );

  input              clk;
  input              rst;
  //input              write;
  input       [31:0] NPC;
  output reg  [31:0] PC;

//warning!!
  //always @(posedge clk, posedge rst)begin
   always @(posedge clk)begin
    if (rst) 
//      PC <= 32'h0000_0000;
      PC <= 32'h1c000000;
    else
    begin
        //if(write) begin PC <= NPC; end
        PC <= NPC;
    end
  end
  
endmodule

