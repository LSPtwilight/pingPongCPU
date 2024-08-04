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

);

reg [127:0] write_buffer;
reg [  2:0] cnt;
reg       writing;
reg       wvalid_r;

reg [31: 0] wr_addr_data_r;

always@(posedge clk)begin
    writing <= !resetn                     ? 1'b0 :
/*awvalid&&awready*/wr_req_data            ? 1'b1 :
               /*wlast&&*/bvalid&&bready   ? 1'b0 : writing;
    wr_addr_data_r <= !resetn              ? 32'b0 :
                      wr_req_data          ? wr_addr_data : wr_addr_data_r;
    cnt     <= !resetn                     ? 3'b0 :
               writing&&wvalid&&wready     ? cnt+1 : 
               /*wlast&&*/bvalid&&bready   ? 3'b0  : cnt;
    wvalid_r <= !resetn                    ? 1'b0 :
                awvalid&&awready           ? 1'b1 :
                /*wlast&&*/bvalid&&bready  ? 1'b0 : wvalid_r;
    write_buffer <= !resetn          ? 128'b0  :
                    wr_req_data ? wr_data_data : write_buffer;
end

reg  [31: 0] wdata_reg;
/*always@(posedge clk)begin
    if(wr_req_data)begin
        wdata_reg <= wr_data_data[31: 0];
    end else begin
        case(cnt)
            //3'd0: wdata_reg <= write_buffer[ 31:  0];
            3'd1: wdata_reg <= write_buffer[ 63: 32];
            3'd2: wdata_reg <= write_buffer[ 95: 64];
            3'd3: wdata_reg <= write_buffer[127: 96];
        endcase
    end
    //wdata_reg <= write_buffer[cnt*32 + 31 : cnt*32];
end*/

always@(*)begin
  case(cnt)
    3'd0: wdata_reg <= write_buffer[ 31:  0];
    3'd1: wdata_reg <= write_buffer[ 63: 32];
    3'd2: wdata_reg <= write_buffer[ 95: 64];
    3'd3: wdata_reg <= write_buffer[127: 96];
  endcase
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
assign awaddr  = wr_addr_data_r;
assign awlen   = 8'd3; // 3+1 //8'd0;
assign awsize  = 3'b010;
assign awburst = 2'b01; // INCR transaction
assign awlock  = 2'd0;
assign awcache = 4'd0;
assign awprot  = 3'd0;
reg awvalid_prepare;
always@(posedge clk) begin
    if(~resetn) begin
        awvalid_prepare <= 1'b0;
    end
    else if(wr_req_data) begin
        awvalid_prepare <= 1'b1;
    end
    else if(awready) begin
        awvalid_prepare <= 1'b0;
    end
    else begin
        awvalid_prepare <= awvalid_prepare;
    end
end
assign awvalid = awvalid_prepare;

//wr_req_data;
//w
assign wid    = 4'd0;
assign wdata  = wdata_reg; //write_buffer[cnt * 32 +: 32];
assign wstrb  = 4'b1111;
assign wlast  = (cnt==3'd3)&&(writing);//1'd1;
assign wvalid = wvalid_r;//1'b1;
//b
assign bready = 1'b1;

// with cache
// data
assign rd_rdy_data    = arready;
assign ret_valid_data = rvalid;
assign ret_last_data  = rlast;
assign ret_data_data  = rdata;

assign wr_rdy_data    = ~writing;
//inst
assign rd_rdy_inst    = arready & ~rd_req_data;
assign ret_valid_inst = rvalid;
assign ret_last_inst  = rlast;
assign ret_data_inst  = rdata;

assign wr_rdy_inst    = 1'b1;


endmodule