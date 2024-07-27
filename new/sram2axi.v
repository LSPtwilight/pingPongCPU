module sram2axi (

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
    input   wire             req_data                    ,
    input   wire              wr_data                    ,
    input   wire [1:0]      size_data                    ,
    input   wire [31:0]     addr_data                    ,
    input   wire [3:0]     wstrb_data                    ,
    input   wire [31:0]    wdata_data                    ,
    output  wire         addr_ok_data                    ,
    output  wire         data_ok_data                    ,
    output  wire [31:0]    rdata_data                    ,

    input   wire             req_inst                    ,
    input   wire              wr_inst                    ,
    input   wire [1:0]      size_inst                    ,
    input   wire [31:0]     addr_inst                    ,
    input   wire [3:0]     wstrb_inst                    ,
    input   wire [31:0]    wdata_inst                    ,
    output  wire         addr_ok_inst                    ,
    output  wire         data_ok_inst                    ,
    output  wire [31:0]    rdata_inst                    

);

reg doing_req               ;
reg doing_req_or            ;//req is inst or data;1:data,0:inst
reg        doing_wr_r       ;
reg [1 :0] doing_size_r     ;
reg [31:0] doing_addr_r     ;
reg [31:0] doing_wdata_r    ;
wire data_back              ;

assign addr_ok_inst = !doing_req&&!req_data&&resetn;
assign addr_ok_data = !doing_req&&resetn;


always@(posedge clk)begin
    doing_req <=    !resetn                             ? 1'b0 :
                    (req_data||req_inst)&&(!doing_req)  ? 1'b1 :
                    data_back                           ? 1'b0 :
                    doing_req                                  ;
    doing_req_or <= !resetn     ? 1'b0 : 
                    !doing_req  ? req_data : doing_req_or;
    doing_wr_r    <=    (req_data&&addr_ok_data) ? wr_data   :
                        (req_inst&&addr_ok_inst) ? wr_inst   : 
                        //cache_rreq               ? 1'b0      :
                        doing_req ? doing_wr_r : 1'b0;
    doing_size_r  <=    (req_data&&addr_ok_data) ? size_data :
                        //cache_rreq               ? cache_rtype:
                        (req_inst&&addr_ok_inst) ? size_inst : doing_size_r;
    doing_addr_r  <=    (req_data&&addr_ok_data) ? addr_data :
                        //cache_rreq               ? cache_raddr:
                        (req_inst&&addr_ok_inst) ? addr_inst : doing_addr_r;
    doing_wdata_r <=    (req_data&&addr_ok_data) ? wdata_data :
                        //cache_rreq               ? 1'b0       :
                        (req_inst&&addr_ok_inst) ? wdata_inst : 
                        doing_req ? doing_wdata_r : 32'b0;
end

assign data_ok_inst = doing_req &&(!doing_req_or)&& data_back;
assign data_ok_data = doing_req &&  doing_req_or && data_back;
assign rdata_inst   = rdata;
assign rdata_data   = rdata;


//axi<-->类sram
reg addr_receive;
reg wdata_receive;

assign data_back = addr_receive && (rvalid&&rready||bvalid&&bready);
always @(posedge clk)
begin
    addr_receive  <= !resetn            ? 1'b0 :
                 arvalid&&arready       ? 1'b1 :
                 awvalid&&awready       ? 1'b1 :
                 data_back              ? 1'b0 : addr_receive;
    wdata_receive <= !resetn            ? 1'b0 :
                 wvalid&&wready         ? 1'b1 :
                 data_back              ? 1'b0 : wdata_receive;
end

//ar
assign arid    = 4'd0;
assign araddr  = doing_addr_r;
assign arlen   = 8'd0;
assign arsize  = doing_size_r;
assign arburst = 2'd0;
assign arlock  = 2'd0;
assign arcache = 4'd0;
assign arprot  = 3'd0;
assign arvalid = doing_req && (!doing_wr_r) && (!addr_receive);
//r
//assign rready  = cache_rreq ? cache_rready : 1'b1;
assign rready = 1'b1;

//aw
assign awid    = 4'd0;
assign awaddr  = doing_addr_r;
assign awlen   = 8'd0;
assign awsize  = doing_size_r;
assign awburst = 2'd0;
assign awlock  = 2'd0;
assign awcache = 4'd0;
assign awprot  = 3'd0;
assign awvalid = doing_req && doing_wr_r && (!addr_receive);
//w
assign wid    = 4'd0;
assign wdata  = doing_wdata_r;
assign wstrb  = (doing_size_r==2'd0) ? 4'b0001<<doing_addr_r[1:0] :
                (doing_size_r==2'd1) ? 4'b0011<<doing_addr_r[1:0] : 4'b1111;
assign wlast  = 1'd1;
assign wvalid = doing_req && doing_wr_r && (!wdata_receive);
//b
assign bready = 1'b1;



endmodule