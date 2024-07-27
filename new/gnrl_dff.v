module gnrl_dff #(parameter WIDTH = 32)(
    input wire clk, 
    input wire rst, 
    input wire write_enable, 
    input wire flush,
    input wire [WIDTH-1:0] in,
    output reg [WIDTH-1:0] out
    );
    //warning
    always@(posedge clk, posedge rst)//?
    begin
        if(rst)
            out <= 0;
        else if(write_enable)
        begin
            if(flush)
                out <= out;
            else
                out <= in;
        end
    end
    
    /*always@(posedge rst)
    begin
        out = 0;
    end*/
    
endmodule
