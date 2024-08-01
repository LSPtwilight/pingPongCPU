// op
`define WRITE 1'b1
`define READ  1'b0

// states
`define IDLE    3'b000
`define LOOKUP  3'b001
`define MISS    3'b010
`define REPLACE 3'b011
`define REFILL  3'b100

// write states
`define W_IDLE  1'b0
`define W_WRITE 1'b1
