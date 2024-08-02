module cache2axi (

    input     wire      clk,
    input     wire      resetn,


    //AXI接口
    output    wire    [ 3 : 0 ]     arid    ,
    output    wire    [ 31: 0 ]     araddr  ,
    output    wire    [ 7 : 0 ]     arlen   ,
    output    wire    [ 2 : 0 ]     arsize  ,
    output    wire    [ 1 : 0 ]     arburst ,
    output    wire    [ 1 : 0 ]     arlock  ,
    output    wire    [ 3 : 0 ]     arcache ,
    output    wire    [ 2 : 0 ]     arprot  ,
    output    wire                  arvalid ,
    input     wire                  arready ,
                
    input     wire    [ 3 : 0 ]     rid     ,
    input     wire    [ 31: 0 ]     rdata   ,
    input     wire    [ 1 : 0 ]     rresp   ,
    input     wire                  rlast   ,
    input     wire                  rvalid  ,
    output    wire                  rready  ,
               
    output    wire    [ 3 : 0 ]    awid    ,
    output    wire    [ 31: 0 ]    awaddr  ,
    output    wire    [ 7 : 0 ]    awlen   ,
    output    wire    [ 2 : 0 ]    awsize  ,
    output    wire    [ 1 : 0 ]    awburst ,
    output    wire    [ 1 : 0 ]    awlock  ,
    output    wire    [ 3 : 0 ]    awcache ,
    output    wire    [ 2 : 0 ]    awprot  ,
    output    wire                 awvalid ,
    input     wire                 awready ,
    
    output    wire    [ 3 : 0 ]    wid     ,
    output    wire    [ 31: 0 ]    wdata   ,
    output    wire    [ 1 : 0 ]    wstrb   ,
    output    wire                 wlast   ,
    output    wire                 wvalid  ,
    input     wire                 wready  ,
    
    input     wire    [ 3 : 0 ]    bid     ,
    input     wire    [ 1 : 0 ]    bresp   ,
    input     wire                 bvalid  ,
    output    wire                 bready  ,

    // 类sram接口
    /*
    input   wire             req_data                    ,
    input   wire              wr_data                    ,
    input   wire [1:0]      size_data                    ,
    input   wire [31:0]     addr_data                    ,
    input   wire [3:0]     wstrb_data                    ,
    input   wire [31:0]    wdata_data                    ,
    output  wire         addr_ok_data                    ,
    output  wire         data_ok_data                    ,
    output  wire [31:0]    rdata_data                    ,
    */
    // with AXI data cache
    input  wire         rd_req_data,
    input  wire [ 2: 0] rd_type_data, // 3'b100 cache line
    input  wire [31: 0] rd_addr_data,
    output wire         rd_rdy_data,
    output wire         ret_valid_data,
    output wire         ret_last_data,
    output wire [31: 0] ret_data_data,

    input  wire         wr_req_data,  
    input  wire [ 2: 0] wr_type_data, // 3'b100 cache line
    input  wire [31: 0] wr_addr_data, 
    input  wire [ 3: 0] wr_wstrb_data,
    input  wire [127:0] wr_data_data, 
    output wire         wr_rdy_data,

    // with AXI inst cache
    input  wire         rd_req_inst,
    input  wire [ 2: 0] rd_type_inst,
    input  wire [31: 0] rd_addr_inst,
    output wire         rd_rdy_inst,
    output wire         ret_valid_inst,
    output wire         ret_last_inst,
    output wire [31: 0] ret_data_inst,

    input  wire         wr_req_inst,  // 1'b0
    input  wire [ 2: 0] wr_type_inst, // 3'b100 cache line
    input  wire [31: 0] wr_addr_inst, 
    input  wire [ 3: 0] wr_wstrb_inst,// 4'b0000
    input  wire [127:0] wr_data_inst, // ignore
    output wire         wr_rdy_inst
/*
    input   wire             req_inst                    ,
    input   wire              wr_inst                    ,
    input   wire [1:0]      size_inst                    ,
    input   wire [31:0]     addr_inst                    ,
    input   wire [3:0]     wstrb_inst                    ,
    input   wire [31:0]    wdata_inst                    ,
    output  wire         addr_ok_inst                    ,
    output  wire         data_ok_inst                    ,
    output  wire [31:0]    rdata_inst                    */

);

reg [127:0] write_buffer;
reg [1:0] cnt;

always@(posedge clk)begin
    cnt <= !resetn        ? 2'b00 :
           wvalid&&wready ? cnt+1 : cnt;
    write_buffer <= !resetn          ? 128'b0 :
                    awvalid&&awready ? wr_data_data : write_buffer;
end

//ar
assign arid    = 4'd0;
assign araddr  = rd_req_data ? rd_addr_data : rd_addr_inst;
assign arlen   = 8'd3;   // 3+1 //8'd0;
assign arsize  = 3'b010; // 4 bytes per transfer
assign arburst = 2'b01;  // INCR transaction
assign arlock  = 2'd0;
assign arcache = 4'd0;
assign arprot  = 3'd0;
assign arvalid = rd_req_data | rd_req_inst;//doing_req && (!doing_wr_r) && (!addr_receive);
//r
//assign rready  = cache_rreq ? cache_rready : 1'b1;
assign rready = 1'b1;

//aw
assign awid    = 4'd0;
assign awaddr  = wr_addr_data;
assign awlen   = 8'd3; // 3+1 //8'd0;
assign awsize  = 3'b010;
assign awburst = 2'b01; // INCR transaction
assign awlock  = 2'd0;
assign awcache = 4'd0;
assign awprot  = 3'd0;
assign awvalid = wr_req_data;
//w
assign wid    = 4'd0;
assign wdata  = write_buffer[cnt * 32 +: 32];
assign wstrb  = 4'b1111;
assign wlast  = cnt==2'b11;//1'd1;
assign wvalid = 1'b1;
//b
assign bready = 1'b1;

// with cache
// data
assign rd_rdy_data    = arready;
assign ret_valid_data = rvalid;
assign ret_last_data  = rlast;
assign ret_data_data  = rdata;

assign wr_rdy_data    = awready;
//inst
assign rd_rdy_inst    = arready & !rd_req_data;
assign ret_valid_inst = rvalid;
assign ret_last_inst  = rlast;
assign ret_data_inst  = rdata;

assign wr_rdy_inst    = 1'b1;


endmodule