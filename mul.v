
//`include "defines.v"

`define MulIdle     2'b00
//`define MulByZero   2'b01
`define MulOn       2'b10
`define MulEnd      2'b11
`define ZeroWord    32'd0

module mul(
	input clk, resetn,
	
	input 	[31:0] 	    opdata1_i,
	input 	[31:0] 	    opdata2_i,
	input 				signed_mul_i,
	input 				start_i,
	input 				annul_i,	// cancel division
	
	output reg 	[63:0]  result_o,
	output reg 			ready_o
);

	//reg 	[64:0] 	dividend;
	//wire 	[32:0] 	div_temp;
	//reg 	[31:0] 	divisor;
    reg     [63:0]  product;
    reg     [63:0]  mul_temp;
    reg     [31:0]  shift;
	reg 	[5:0] 	cnt;
	reg 	[1:0] 	state;

	reg     sign_1;
	reg     sign_2;

	//assign div_temp = {1'b0, dividend[63:32]} - {1'b0, divisor};

	always @ (posedge clk) begin
		if (!resetn) begin
			state 	<= `MulIdle;
			ready_o <= 1'b0;
			result_o <= {`ZeroWord,`ZeroWord};
		end else begin
		    case (state)
		  	`MulIdle: begin               //MulIdle
		  		if (start_i == 1'b1 && annul_i == 1'b0) begin
		  			state <= `MulOn;
		  			cnt <= 6'b000000;
		  			product <= 64'b0;
                    mul_temp[63:32] <= 32'b0;
					sign_1 <= opdata1_i[31];
					sign_2 <= opdata2_i[31];
		  			if (signed_mul_i == 1'b1 && opdata1_i[31] == 1'b1 ) begin
		  				mul_temp[31:0] <= ~opdata1_i + 1;
		  			end else begin
		  				mul_temp[31:0] <= opdata1_i;
		  			end
		  			if (signed_mul_i == 1'b1 && opdata2_i[31] == 1'b1 ) begin
		  				shift <= ~opdata2_i + 1;
		  			end else begin
		  				shift <= opdata2_i;
		  			end
                end else begin
				     ready_o <= 1'b0;
					 result_o <= {`ZeroWord,`ZeroWord};
				end          	
		  	end
		  	`MulOn:	begin               //MulOn
		  		if (annul_i == 1'b0) begin
		  			if (cnt != 6'b100000) begin 	// cnt reach 8 32
                        /*product <= product + (
                        (({64{shift[0]}} & (mul_temp << 0)) + ({64{shift[1]}} & (mul_temp << 1))) +
                        (({64{shift[2]}} & (mul_temp << 2)) + ({64{shift[3]}} & (mul_temp << 3))) );
                        shift    <= shift    >> 4;
                        mul_temp <= mul_temp << 4;*/
						product <= product + ({64{shift[0]}} & mul_temp);
						shift    <= shift    >> 1;
						mul_temp <= mul_temp << 1;
                    	cnt <= cnt + 1;
                    end else begin
                        if ((signed_mul_i == 1'b1) && ((/*opdata1_i[31] ^ opdata2_i[31]*/sign_1 ^ sign_2) == 1'b1)) begin
                            product <= ~product + 1;
                        end
                    	state <= `MulEnd;
                    	cnt <= 6'b000000;
                	end
		    	end else begin
		  			state <= `MulIdle;
		  		end	
		  	end
		  	`MulEnd: begin               //MulEnd
        		result_o 	<= product;  	// �?32为余数，�?32为商
          		ready_o 	<= 1'b1;
          		if (start_i == 1'b0) begin
          			state 	<= `MulIdle;
					ready_o <= 1'b0;
					result_o <= {`ZeroWord,`ZeroWord};       	
          		end		  	
		  	end
		  endcase
		end
	end
endmodule
