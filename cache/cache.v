`include "cache_def.vh"

module cache(
    input wire clk,
    input wire resetn,
    // with CPU
    input  wire         valid,
    input  wire         op,    // 1'b1 WRITE, 1'b0 READ
    input  wire [ 7: 0] index, // addr [11:4]
    input  wire [19: 0] tag,
    input  wire [ 3: 0] offset,
    input  wire [ 3: 0] wstrb,
    input  wire [31: 0] wdata,

    output wire         addr_ok,
    output wire         data_ok,
    output wire [31: 0] rdata,

    // with AXI
    output wire         rd_req,
    output wire [ 2: 0] rd_type,
    output wire [31: 0] rd_addr,
    input  wire         rd_rdy,
    input  wire         ret_valid,
    input  wire         ret_last,
    input  wire [31: 0] ret_data,

    output wire         wr_req,
    output wire [ 2: 0] wr_type,
    output wire [31: 0] wr_addr,
    output wire [ 3: 0] wr_wstrb,
    output wire [127:0] wr_data,
    input  wire         wr_rdy
);

/*-------Request Buffer--------*/
reg         reg_op;
reg [ 7: 0] reg_index;
reg [19: 0] reg_tag;
reg [ 3: 0] reg_offset;
reg [ 3: 0] reg_wstrb;
reg [31: 0] reg_wdata;
/*-----------------------------*/
/*-------Write Buffer----------*/
reg [ 7: 0] write_index;
reg [19: 0] write_tag;
reg [ 3: 0] write_offset;
reg         write_way;
reg [ 3: 0] write_wstrb;
reg [31: 0] write_wdata;
/*-------Replace Buffer--------*/
reg         wr_req_;
reg [ 7: 0] reg_replace_index;
reg [19: 0] reg_replace_tag;
//reg [ 3: 0] replace_offset;
reg         reg_replace_way;
reg [127:0] reg_replace_data;
reg         reg_replace_dirty;
//reg [ 3: 0] replace_wstrb;
//reg [31: 0] replace_wdata;
/*-------Refill Buffer---------*/
reg [ 1: 0] rd_cnt;
wire [31: 0] refill_wdata;
/*reg [ 7: 0] refill_index;
reg [19: 0] refill_tag;
reg [ 3: 0] refill_offset;
reg         refill_way;
reg [ 3: 0] refill_wstrb;
reg [31: 0] refill_wdata;*/
/*-----------------------------*/

reg [2:0] state;
reg       w_state;

wire [127:0] way0_data;
wire [127:0] way1_data;

wire [20: 0] way0_tag_v;
wire [20: 0] way1_tag_v;

wire [19: 0] way0_tag = way0_tag_v[20:1];
wire [19: 0] way1_tag = way1_tag_v[20:1];
wire         way0_v   = way0_tag_v[0];
wire         way1_v   = way1_tag_v[0];

wire way0_hit  = way0_v && (way0_tag==reg_tag);
wire way1_hit  = way1_v && (way1_tag==reg_tag);
wire cache_hit = way0_hit | way1_hit;

wire [31:0] way0_load_word = way0_data[reg_offset[3:2]*32 +: 32];
wire [31:0] way1_load_word = way1_data[reg_offset[3:2]*32 +: 32];
wire [31:0] load_res       = {32{way0_hit}} & way0_load_word
                           | {32{way1_hit}} & way1_load_word
                           | {32{state == `REFILL && ret_valid==1'b1 && rd_cnt == reg_offset[3:2] && reg_op==`READ}} & ret_data;

wire         replace_way = !way0_v ? 1'b0 : 
                           !way1_v ? 1'b1 :
                                     1'b0/*LSFR*/;
wire [127:0] replace_data  = replace_way ? way1_data  : way0_data ;
wire         replace_dirty = replace_way ? way1_d_out : way0_d_out; 
wire [19: 0] replace_tag   = replace_way ? way1_tag   : way0_tag  ;
wire [ 7: 0] replace_index = reg_index;


/*--------------------------------*/
wire look_up;
wire hit_write;
wire replace;
wire refill;

wire [3:0] way0_data_banks_ena;
wire [3:0] way1_data_banks_ena;

wire [3:0] way0_data_bank0_wea;
wire [3:0] way1_data_bank0_wea;

wire [3:0] way0_data_bank1_wea;
wire [3:0] way1_data_bank1_wea;

wire [3:0] way0_data_bank2_wea;
wire [3:0] way1_data_bank2_wea;

wire [3:0] way0_data_bank3_wea;
wire [3:0] way1_data_bank3_wea;

wire       way0_tag_v_ena;
wire       way1_tag_v_ena;
wire       way0_tag_v_wea;
wire       way1_tag_v_wea;
wire [20:0] way0_tag_v_in;
wire [20:0] way1_tag_v_in;

wire       way0_d_write;
wire [7:0] way0_d_addr;
wire       way0_d_in;
wire       way0_d_out;

wire       way1_d_write;
wire [7:0] way1_d_addr;
wire       way1_d_in;
wire       way1_d_out;

/*--------------------------------*/

wire      hit_write_conflict;

/*--------way0 data rams----------*/

data_bank_ram way0_data_bank_0(
    .addra (
        look_up ? index : // read
        w_state == `W_WRITE ? write_index : // write
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b0 ? reg_replace_index : 
        8'b0
    ), //[7:0]
    .clka  (clk                    ),
    .dina  (
        w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd0 ? write_wdata :
        state == `REFILL && reg_replace_way == 1'b0 ? ret_data :
        32'b0
    ), // [31:0]
    .douta (way0_data[31: 0]       ), // [31:0]
    .ena   (
        look_up && offset[3:2] == 2'd0
    ||  w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd0
    ||  state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */
    ||  state == `REFILL && reg_replace_way == 1'b0
    ),
    .wea   (
        w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd0 ? write_wstrb :
        state == `REFILL && reg_replace_way == 1'b0 && rd_cnt == 2'd0 ? 4'b1111:
        4'b0
    )
);

data_bank_ram way0_data_bank_1(
    .addra (
        look_up ? index : // read
        w_state == `W_WRITE ? write_index : // write
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b0 ? reg_replace_index : 
        8'b0
    ), //[7:0]
    .clka  (clk                    ),
    .dina  (
        w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd1 ? write_wdata :
        state == `REFILL && reg_replace_way == 1'b0 ? refill_wdata :
        32'b0
    ), // [31:0]
    .douta (way0_data[63: 32]       ), // [31:0]
    .ena   (
        look_up && offset[3:2] == 2'd1
    ||  w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd1
    ||  state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */
    ||  state == `REFILL && reg_replace_way == 1'b0
    ),
    .wea   (
        w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd1 ? write_wstrb :
        state == `REFILL && reg_replace_way == 1'b0 && rd_cnt == 2'd1 ? 4'b1111:
        4'b0
    )
);

data_bank_ram way0_data_bank_2(
    .addra (
        look_up ? index : // read
        w_state == `W_WRITE ? write_index : // write
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b0 ? reg_replace_index : 
        8'b0
    ), //[7:0]
    .clka  (clk                    ),
    .dina  (
        w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd2 ? write_wdata :
        state == `REFILL && reg_replace_way == 1'b0 ? refill_wdata :
        32'b0
    ), // [31:0]
    .douta (way0_data[95: 64]       ), // [31:0]
    .ena   (
        look_up && offset[3:2] == 2'd2
    ||  w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd2
    ||  state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */
    ||  state == `REFILL && reg_replace_way == 1'b0
    ),
    .wea   (
        w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd2 ? write_wstrb :
        state == `REFILL && reg_replace_way == 1'b0 && rd_cnt == 2'd2 ? 4'b1111:
        4'b0
    )
);

data_bank_ram way0_data_bank_3(
    .addra (
        look_up ? index : // read
        w_state == `W_WRITE ? write_index : // write
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b0 ? reg_replace_index : 
        8'b0
    ), //[7:0]
    .clka  (clk                    ),
    .dina  (
        w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd3 ? write_wdata :
        state == `REFILL && reg_replace_way == 1'b0 ? refill_wdata :
        32'b0
    ), // [31:0]
    .douta (way0_data[127: 96]       ), // [31:0]
    .ena   (
        look_up && offset[3:2] == 2'd3
    ||  w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd3
    ||  state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */
    ||  state == `REFILL && reg_replace_way == 1'b0
    ),
    .wea   (
        w_state == `W_WRITE && write_way == 1'b0 && write_offset[3:2] == 2'd3 ? write_wstrb :
        state == `REFILL && reg_replace_way == 1'b0 && rd_cnt == 2'd3 ? 4'b1111:
        4'b0
    )
);

/*data_bank_ram way0_data_bank_0(
    .addra (index                  ), //[7:0]
    .clka  (clk                    ),
    .dina  (wdata                  ), // [31:0]
    .douta (way0_data[31: 0]       ), // [31:0]
    .ena   (way0_data_banks_ena[0] ),
    .wea   (way0_data_bank0_wea    )
);

data_bank_ram way0_data_bank_1(
    .addra (index                  ), //[7:0]
    .clka  (clk                    ),
    .dina  (wdata                  ), // [31:0]
    .douta (way0_data[63:32]       ), // [31:0]
    .ena   (way0_data_banks_ena[1] ),
    .wea   (way0_data_bank1_wea    )
);

data_bank_ram way0_data_bank_2(
    .addra (index                  ), //[7:0]
    .clka  (clk                    ),
    .dina  (wdata                  ), // [31:0]
    .douta (way0_data[95:64]       ), // [31:0]
    .ena   (way0_data_banks_ena[2] ),
    .wea   (way0_data_bank2_wea    )
);

data_bank_ram way0_data_bank_3(
    .addra (index                  ), //[7:0]
    .clka  (clk                    ),
    .dina  (wdata                  ), // [31:0]
    .douta (way0_data[127:96]      ), // [31:0]
    .ena   (way0_data_banks_ena[3] ),
    .wea   (way0_data_bank3_wea    )
);*/

/*--------way1 data rams----------*/
data_bank_ram way1_data_bank_0(
    .addra (
        look_up ? index : // read
        w_state == `W_WRITE ? write_index : // write
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b1 ? reg_replace_index : 
        8'b0
    ), //[7:0]
    .clka  (clk                    ),
    .dina  (
        w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd0 ? write_wdata :
        state == `REFILL && reg_replace_way == 1'b1 ? refill_wdata :
        32'b0
    ), // [31:0]
    .douta (way1_data[31: 0]       ), // [31:0]
    .ena   (
        look_up && offset[3:2] == 2'd0
    ||  w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd0
    ||  state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */
    ||  state == `REFILL && reg_replace_way == 1'b1
    ),
    .wea   (
        w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd0 ? write_wstrb :
        state == `REFILL && reg_replace_way == 1'b1 && rd_cnt == 2'd0 ? 4'b1111:
        4'b0
    )
);

data_bank_ram way1_data_bank_1(
    .addra (
        look_up ? index : // read
        w_state == `W_WRITE ? write_index : // write
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b1 ? reg_replace_index : 
        8'b0
    ), //[7:0]
    .clka  (clk                    ),
    .dina  (
        w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd1 ? write_wdata :
        state == `REFILL && reg_replace_way == 1'b1 ? refill_wdata :
        32'b0
    ), // [31:0]
    .douta (way1_data[63: 32]       ), // [31:0]
    .ena   (
        look_up && offset[3:2] == 2'd1
    ||  w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd1
    ||  state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */
    ||  state == `REFILL && reg_replace_way == 1'b1
    ),
    .wea   (
        w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd1 ? write_wstrb :
        state == `REFILL && reg_replace_way == 1'b1 && rd_cnt == 2'd1 ? 4'b1111:
        4'b0
    )
);

data_bank_ram way1_data_bank_2(
    .addra (
        look_up ? index : // read
        w_state == `W_WRITE ? write_index : // write
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b1 ? reg_replace_index : 
        8'b0
    ), //[7:0]
    .clka  (clk                    ),
    .dina  (
        w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd2 ? write_wdata :
        state == `REFILL && reg_replace_way == 1'b1 ? refill_wdata :
        32'b0
    ), // [31:0]
    .douta (way1_data[95: 64]       ), // [31:0]
    .ena   (
        look_up && offset[3:2] == 2'd2
    ||  w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd2
    ||  state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */
    ||  state == `REFILL && reg_replace_way == 1'b1
    ),
    .wea   (
        w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd2 ? write_wstrb :
        state == `REFILL && reg_replace_way == 1'b1 && rd_cnt == 2'd2 ? 4'b1111:
        4'b0
    )
);

data_bank_ram way1_data_bank_3(
    .addra (
        look_up ? index : // read
        w_state == `W_WRITE ? write_index : // write
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b1 ? reg_replace_index : 
        8'b0
    ), //[7:0]
    .clka  (clk                    ),
    .dina  (
        w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd3 ? write_wdata :
        state == `REFILL && reg_replace_way == 1'b1 ? refill_wdata :
        32'b0
    ), // [31:0]
    .douta (way1_data[127: 96]       ), // [31:0]
    .ena   (
        look_up && offset[3:2] == 2'd3
    ||  w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd3
    ||  state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */
    ||  state == `REFILL && reg_replace_way == 1'b1
    ),
    .wea   (
        w_state == `W_WRITE && write_way == 1'b1 && write_offset[3:2] == 2'd3 ? write_wstrb :
        state == `REFILL && reg_replace_way == 1'b1 && rd_cnt == 2'd3 ? 4'b1111:
        4'b0
    )
);

/*data_bank_ram way1_data_bank_0(
    .addra (index                  ), //[7:0]
    .clka  (clk                    ),
    .dina  (wdata                  ), // [31:0]
    .douta (way1_data[31: 0]       ), // [31:0]
    .ena   (way1_data_banks_ena[0] ),
    .wea   (way1_data_bank0_wea    )
);

data_bank_ram way1_data_bank_1(
    .addra (index                  ), //[7:0]
    .clka  (clk                    ),
    .dina  (wdata                  ), // [31:0]
    .douta (way1_data[63:32]       ), // [31:0]
    .ena   (way1_data_banks_ena[1] ),
    .wea   (way1_data_bank1_wea    )
);

data_bank_ram way1_data_bank_2(
    .addra (index                  ), //[7:0]
    .clka  (clk                    ),
    .dina  (wdata                  ), // [31:0]
    .douta (way1_data[95:64]       ), // [31:0]
    .ena   (way1_data_banks_ena[2] ),
    .wea   (way1_data_bank2_wea    )
);

data_bank_ram way1_data_bank_3(
    .addra (index                  ), //[7:0]
    .clka  (clk                    ),
    .dina  (wdata                  ), // [31:0]
    .douta (way1_data[127:96]      ), // [31:0]
    .ena   (way1_data_banks_ena[3] ),
    .wea   (way1_data_bank3_wea    )  // [ 3:0]
);
*/
/*----------tag_v---------------*/
tag_v_ram way0_tag_v_ram(
    .addra (
        look_up ? index :
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b0 ? reg_replace_index :
        8'b0
    ), //[7:0]
    .clka  (clk              ),
    .dina  (
        state == `REFILL && reg_replace_way == 1'b0 ? {reg_tag, 1'b1} :
        21'b0
    ), // [20:0]
    .douta (way0_tag_v       ), // [20:0]
    .ena   (
        look_up
    ||  state == `MISS && wr_rdy == 1'b1
    ||  state == `REFILL && reg_replace_way == 1'b0
    ),
    .wea   (
        state == `REFILL && reg_replace_way == 1'b0
    )  // [ 0:0]
);

tag_v_ram way1_tag_v_ram(
    .addra (
        look_up ? index :
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b1 ? reg_replace_index :
        8'b0
    ), //[7:0]
    .clka  (clk              ),
    .dina  (
        state == `REFILL && reg_replace_way == 1'b1 ? {reg_tag, 1'b1} :
        21'b0
    ), // [20:0]
    .douta (way1_tag_v       ), // [20:0]
    .ena   (
        look_up
    ||  state == `MISS && wr_rdy == 1'b1
    ||  state == `REFILL && reg_replace_way == 1'b1
    ),
    .wea   (
        state == `REFILL && reg_replace_way == 1'b1
    )  // [ 0:0]
);
/*----------Dirty--------------------*/
cache_table way0_d_table(
    .clk (clk), 
    .rst (~resetn),
    .RFWr(
        w_state == `W_WRITE && write_way == 1'b0
    //||  state == `MISS && wr_rdy == 1'b1
    ||  state == `REFILL && reg_replace_way == 1'b0
    ), 
    .Addr(
        w_state == `W_WRITE && write_way == 1'b0 ? write_index :
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b0 ? reg_replace_index :
        8'b0
    ), 
    .WD  (
        w_state == `W_WRITE && write_way == 1'b0 ? 1'b1 :
        state == `REFILL && reg_replace_way == 1'b0 ? (reg_op == `WRITE) :
        1'b0
    ), 
    .RD  (way0_d_out)
);

cache_table way1_d_table(
    .clk (clk), 
    .rst (~resetn),
    .RFWr(
        w_state == `W_WRITE && write_way == 1'b1
    //||  state == `MISS && wr_rdy == 1'b1
    ||  state == `REFILL && reg_replace_way == 1'b1
    ), 
    .Addr(
        w_state == `W_WRITE && write_way == 1'b1 ? write_index :
        state == `MISS && wr_rdy == 1'b1 /* MISS -> REPLACE */ ? reg_index :
        state == `REFILL && reg_replace_way == 1'b1 ? reg_replace_index :
        8'b0
    ), 
    .WD  (
        w_state == `W_WRITE && write_way == 1'b1 ? 1'b1 :
        state == `REFILL && reg_replace_way == 1'b1 ? (reg_op == `WRITE) :
        1'b0
    ), 
    .RD  (way1_d_out)
);

/*--------------------------DFA----------------------------*/

assign hit_write_conflict = state == `LOOKUP  && 1'b1 /*cache_hit*/ && reg_op == `WRITE && valid == 1'b1 && tag==reg_tag && index==reg_index && offset[3:2]==reg_offset[3:2]
                         || w_state == `W_WRITE && valid == 1'b1 && tag==write_tag && index==write_index && offset[3:2]==write_offset[3:2];

assign look_up   = state == `IDLE    && valid && !hit_write_conflict /* IDLE -> LOOKUP */
                 ||state == `LOOKUP  && 1'b1 /*cache_hit*/ && valid && !hit_write_conflict /* LOOKUP -> LOOKUP */;
assign hit_write = state == `LOOKUP  && cache_hit && reg_op == `WRITE;
assign replace   = state == `MISS    && wr_rdy == 1'b1 /* MISS -> REPLACE */
                 ||state == `REPLACE && rd_rdy == 1'b0 /* REPLACE -> REPLACE */;
assign refill    = state == `REPLACE && rd_rdy == 1'b1 /* REPLACE -> REFILL */
                 ||state == `REFILL  && ret_valid==1'b1 && ret_last==1'b0 /*??*/ /* REFILL -> REFILL*/;

wire [31: 0] write_mask = {{8{reg_wstrb[3]}}, {8{reg_wstrb[2]}}, {8{reg_wstrb[1]}}, {8{reg_wstrb[0]}}};
assign refill_wdata = reg_op == `WRITE && reg_offset[3:2] == rd_cnt ? 
                    (write_mask & reg_wdata) | (~write_mask & ret_data) : ret_data;

always@(posedge clk, negedge resetn/*?*/)
begin
    if(~resetn)
    begin
    /*-------buffers--------*/
        reg_op <= 1'b0;
        reg_index <= 8'b0;
        reg_tag <= 20'b0;
        reg_offset <= 4'b0;
        reg_wstrb <= 4'b0;
        reg_wdata <= 32'b0;
        write_index <= 8'b0;
        write_tag <= 20'b0;
        write_offset <= 4'b0;
        write_way <= 1'b0;
        write_wstrb <= 4'b0;
        write_wdata <= 32'b0;
        reg_replace_index <= 8'b0;
        reg_replace_tag <= 20'b0;
        reg_replace_way <= 1'b0;
        reg_replace_data <= 128'b0;
        reg_replace_dirty <= 1'b0;
        rd_cnt <= 2'b0;
    /*---------------------*/
        state   <= `IDLE;
        w_state <= `W_IDLE;
        wr_req_  <= 1'b0;
    end
    else
    begin
        /*-----------------main states---------------------*/
        case(state)
            `IDLE:
            begin/*didn't consider hit write*/
                if(valid && !hit_write_conflict)
                begin /* IDLE -> LOOKUP */
                    reg_op     <= op    ;
                    reg_index  <= index ;
                    reg_tag    <= tag   ;
                    reg_offset <= offset;
                    reg_wstrb  <= wstrb ;
                    reg_wdata  <= wdata ;

                    state <= `LOOKUP;
                end
            end
            `LOOKUP:
            begin
                if(cache_hit && !valid || valid && hit_write_conflict)
                begin /* LOOKUP -> IDLE */
                    state <= `IDLE;
                end
                else if(cache_hit && valid && !hit_write_conflict)
                begin /* LOOKUP -> LOOKUP */
                    reg_op     <= op    ;
                    reg_index  <= index ;
                    reg_tag    <= tag   ;
                    reg_offset <= offset;
                    reg_wstrb  <= wstrb ;
                    reg_wdata  <= wdata ;
                    // transfer into LOOKUP
                    state <= `LOOKUP;
                end
                else if(!cache_hit)
                begin /* LOOKUP -> MISS */
                    state <= `MISS;
                end
            end
            `MISS:
            begin
                /*if(wr_rdy == 1'b0)
                begin // MISS -> MISS 
                    ;
                end
                else */if(wr_rdy == 1'b1)
                begin /* MISS -> REPLACE */
                    wr_req_ <= 1'b1;
                    state <= `REPLACE;
                end
            end
            `REPLACE:
            begin
                if(wr_req_ == 1'b1)
                begin
                    wr_req_ <= 1'b0;// ?
                    reg_replace_index <= replace_index;
                    reg_replace_tag   <= replace_tag;
                    reg_replace_way   <= replace_way;
                    reg_replace_data  <= replace_data;
                    reg_replace_dirty <= replace_dirty;
                end
                /*if(rd_rdy == 1'b0)
                begin // REPLACE -> REPLACE 
                    ;
                end
                else */if(rd_rdy == 1'b1)
                begin /* REPLACE -> REFILL */
                    rd_cnt <= 2'b00;
                    state <= `REFILL;
                end
            end
            `REFILL:
            begin
                if(ret_valid==1'b1 && ret_last==1'b0/*?????????*/)
                begin /* REFILL -> REFILL */
                    rd_cnt <= rd_cnt + 1;
                end
                else if(ret_valid==1'b1 && ret_last==1'b1)
                begin /* REFILL -> IDLE */
                    state <= `IDLE;
                end
            end
        endcase
        /*---------------write states-------------------*/
        case(w_state)
            `W_IDLE:
            begin
                if(hit_write)
                begin /* W_IDLE -> W_WRITE */
                    write_index  <= reg_index;
                    write_tag    <= reg_tag;
                    write_offset <= reg_offset;
                    write_way    <= way0_hit ? 1'b0 : 1'b1;
                    write_wstrb  <= reg_wstrb;
                    write_wdata  <= reg_wdata;
                    w_state <= `W_WRITE;
                end
                else // !hit_write
                begin /* W_IDLE -> W_IDLE */
                    w_state <= `W_IDLE;
                end
            end
            `W_WRITE:
            begin
                if(hit_write)
                begin /* W_WRITE -> W_WRITE */
                    write_index  <= reg_index;
                    write_tag    <= reg_tag;
                    write_offset <= reg_offset;
                    write_way    <= way0_hit ? 1'b0 : 1'b1;
                    write_wstrb  <= reg_wstrb;
                    write_wdata  <= reg_wdata;
                    w_state <= `W_WRITE;
                end
                else // !hit_write
                begin /* W_WRITE -> W_IDLE */
                    w_state <= `W_IDLE;
                end
            end
        endcase
    end
end

/*-----------outputs--------------*/
assign addr_ok = state == `IDLE
               || state == `LOOKUP && cache_hit && valid && !hit_write_conflict; /* LOOKUP -> LOOKUP */
assign data_ok = state == `LOOKUP && cache_hit
               || state == `LOOKUP && reg_op == `WRITE
               || state == `REFILL && ret_valid==1'b1 && rd_cnt == reg_offset[3:2] && reg_op==`READ;
assign rdata   = load_res;
                
/*
    // with AXI
    output wire         rd_req,
    output wire [ 2: 0] rd_type,
    output wire [31: 0] rd_addr,
    input  wire         rd_rdy,
    input  wire         ret_valid,
    input  wire         ret_last,
    input  wire [31: 0] ret_data,

    output wire         wr_req,
    output wire [ 2: 0] wr_type,
    output wire [31: 0] wr_addr,
    output wire [ 3: 0] wr_wstrb,
    output wire [127:0] wr_data,
    input  wire         wr_rdy
*/
assign rd_req  = state == `REPLACE;
assign rd_type = 3'b100; //?
assign rd_addr = {reg_tag, reg_index, 4'b0}; 

assign wr_req   = wr_req_ & replace_dirty;
assign wr_type  = 3'b100;
assign wr_addr  = {replace_tag, replace_index, 4'b0};
assign wr_wstrb = replace_dirty ? 4'b1111 : 4'b0000; //??
assign wr_data  = replace_data;

endmodule
