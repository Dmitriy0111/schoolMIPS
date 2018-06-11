/*
 * schoolMIPS - small MIPS CPU for "Young Russian Chip Architects" 
 *              summer school ( yrca@googlegroups.com )
 *
 * AHB-Lite eth 
 * 
 * Copyright(c) 2017-2018 Stanislav Zhelnio
 *                        Dmitriy Vlasov
 */

`include "ahb_lite.vh"
`include "sm_settings.vh"

`define SM_ETH_REG_DATA   4'h0
`define SM_ETH_REG_COM    4'h4


module ahb_eth
(
    //bus side
    input            HCLK,
    input            HRESETn,
    input            HSEL,
    input            HWRITE,
    input            HREADY,
    input     [ 1:0] HTRANS,
    input     [31:0] HADDR,
    output    [31:0] HRDATA,
    input     [31:0] HWDATA,
    output           HREADYOUT,
    output           HRESP,

    //eth side
    input            eth_clk,
    input            eth_rstn,
    output           Txp,
    output           Txn,
    output           Led_Tx
);

    // bus input decode
    wire request   = HREADY & HSEL & HTRANS != `HTRANS_IDLE;
    wire request_r = request & !HWRITE;

    wire request_w;
    wire request_w_new = request & HWRITE;
    sm_register_c r_request_w (HCLK, HRESETn, request_w_new, request_w);

    wire [31:0] addr_w;
    wire [31:0] addr_r = HADDR;
    sm_register_we #(32) r_addr_w (HCLK, HRESETn, request, HADDR, addr_w);

    // peripheral module interface
    wire        pm_we    = request_w;
    wire [31:0] pm_wd    = HWDATA;
    wire [31:0] pm_addr  = request_w ? addr_w : addr_r;
    wire        pm_valid = request_r | request_w;
    wire [31:0] pm_rd;


    // read after write hazard
    wire hz_raw;
    wire hz_raw_new = (request_r & request_w) | request_w_new;
    sm_register_c r_hz_raw (HCLK, HRESETn, hz_raw_new, hz_raw );

    // bus output
    assign HREADYOUT = ~hz_raw;
    assign HRDATA    = pm_rd;
    assign HRESP     = 1'b0;

    sm_eth eth
    (
        .eth_clk    ( eth_clk       ),
        .eth_rstn   ( HRESETn       ),
        .Txp        ( Txp           ),
        .Txn        ( Txn           ),
        .Led_Tx     ( Led_Tx        )
    );

    
    sm_eth_mem sm_eth_mem_0
    (
        .HCLK       ( HCLK          ),
        .HRESETn    ( HRESETn       ),
        .bSel       ( pm_valid      ),
        .bAddr      ( pm_addr       ),
        .bWrite     ( pm_we         ),
        .bWData     ( pm_wd         ),
        .bRData     ( pm_rd         )
    );

endmodule


/*module sm_gpio
(
    //bus side
    input             clk,
    input             rst_n,
    input             bSel,
    input      [31:0] bAddr,
    input             bWrite,
    input      [31:0] bWData,
    output reg [31:0] bRData,

    //pin side
    input  [`SM_GPIO_WIDTH - 1:0] gpioInput,
    output [`SM_GPIO_WIDTH - 1:0] gpioOutput
);
    wire   [`SM_GPIO_WIDTH - 1:0] gpioIn;    // debounced input signals
    wire                          gpioOutWe; // output Pin value write enable
    wire   [`SM_GPIO_WIDTH - 1:0] gpioOut;   // output Pin next value

    assign gpioOut   = bWData [`SM_GPIO_WIDTH - 1:0];
    assign gpioOutWe = bSel & bWrite & (bAddr[3:0] == `SM_GPIO_REG_OUTPUT);

    sm_debouncer   #(`SM_GPIO_WIDTH) debounce(clk, gpioInput, gpioIn);
    sm_register_we #(`SM_GPIO_WIDTH) r_output(clk, rst_n, gpioOutWe, gpioOut, gpioOutput);

    localparam BLANK_WIDTH = 32 - `SM_GPIO_WIDTH;

    always @ (*)
        case(bAddr[3:0])
            default              : bRData = { { BLANK_WIDTH {1'b0}}, gpioIn  };
            `SM_GPIO_REG_INPUT   : bRData = { { BLANK_WIDTH {1'b0}}, gpioIn  };
            `SM_GPIO_REG_OUTPUT  : bRData = { { BLANK_WIDTH {1'b0}}, gpioOut };
        endcase

endmodule*/


module sm_eth_mem
(
    input             HCLK,
    input             HRESETn,
    input             bSel,
    input      [31:0] bAddr,
    input             bWrite,
    input      [31:0] bWData,
    output     [31:0] bRData
);
    reg        c_d;
    reg  [7:0] eth_mem [127:0];
    reg  [7:0] com_reg ;
    wire mem_wr;
    wire reg_wr;

    assign mem_wr = bWrite & ( ~ c_d ) & bSel ;
    assign reg_wr = bWrite & (   c_d ) & bSel ;
    assign bRData = c_d ? eth_mem[ bAddr ] : com_reg ;

    always @ ( posedge HCLK )
    begin
        if ( mem_wr )
            eth_mem[ bAddr[8:2] ] <= bWData[7:0] ;
        if ( reg_wr )
            com_reg <= bWData[7:0] ;
    end

    always @ (*)
        case(bAddr[3:0])
            default           : c_d = 1'b0 ;
            `SM_ETH_REG_COM   : c_d = 1'b0 ;
            `SM_ETH_REG_DATA  : c_d = 1'b1 ;
        endcase
    
    initial
    begin
        com_reg = 8'h00 ;
        c_d = 1'b0 ;
        $readmemh("../eth_frame.hex",eth_mem) ;
    end

endmodule

module sm_eth
(
    input             eth_clk,
    input             eth_rstn,
    /*input             bSel,
    input      [31:0] bAddr,
    input             bWrite,
    input      [31:0] bWData,
    output reg [31:0] bRData,*/
    output            Txp,
    output            Txn,
    output            Led_Tx
);

wire clk_20m;
wire eth_data_s;
wire Tx_nlp;
wire go;
wire Txn_nlp;
wire Txp_nlp;
wire Txn_TxD;
wire Txp_TxD;
wire Tx_w;

/*eth_pll eth_pll_0
(
    .inclk0 (   clk     ),
    .c0     (   clk_20m )
);*/

assign clk_20m = eth_clk;

assign Txp_nlp = Tx_nlp == 1'b1 ? 1'b1 : 1'b0 ;
assign Txn_nlp = Tx_nlp == 1'b1 ? 1'b0 : 1'b0 ;

assign Txp_TxD = Tx_w ?   eth_data_s : 1'b0 ;
assign Txn_TxD = Tx_w ? ~ eth_data_s : 1'b0 ;

assign Txn = Txn_nlp | Txn_TxD ;
assign Txp = Txp_nlp | Txp_TxD ;

assign Led_Tx = ~ ( Tx_w ) ;

nlp nlp_0
    (
        .clk      ( clk_20m   ),
        .go       ( go        ),
        .Tx       ( Tx_nlp    )
    );

eth_frame eth_frame_0
(
    .transmit   ( go            ),
    .clk        ( clk_20m       ),
    .rst_n      ( eth_rstn      ),
    .eth_data_s ( eth_data_s    ),
    .Tx_w       ( Tx_w          )
);

endmodule

module eth_frame
(
    input        transmit,
    input        clk,
    input        rst_n,
    output       eth_data_s,
	output		 Tx_w
);

localparam eth_frame_length = 'd128 ;
localparam addr_max         = {$clog2(eth_frame_length){1'b1}} ;
localparam addr_zero        = {$clog2(eth_frame_length){1'b0}} ;

//assign TxD = eth_data_s | Tx_idle;

reg [7:0]                           eth_data    [ eth_frame_length-1 : 0 ] ;
reg [7:0]                           eth_data_reg ;
reg [$clog2(eth_frame_length)-1:0]  addr ;
reg [1:0]                           state ;
reg [2:0]                           count ;
reg                                 manch ;
reg   										Tx		;
reg   										Tx_idle ;

assign Tx_w			= Tx | Tx_idle;
assign eth_data_s = ( ~ ( manch ^ eth_data_reg[count] ) ) | Tx_idle ;

localparam  WAIS_S      = 2'b00 ,
            BROADCAST_S = 2'b01 ,
            TP_IDLE     = 2'b10 ;

always @(posedge clk or negedge rst_n) 
begin
    if( ~ rst_n )
        begin
            state   <= WAIS_S ;
            addr    <= addr_zero  ;
            Tx_idle <= 1'b0 ;
            Tx      <= 1'b0 ;
            manch   <= 1'b0 ;
        end
    else
        case( state )
        WAIS_S:
            begin
                Tx      <= 1'b0 ;
                manch   <= 1'b0 ;
					 Tx_idle <= 1'b0 ;
                if( transmit == 1'b1 )
                begin
                    state        <= BROADCAST_S ;
                    eth_data_reg <= eth_data[0] ;
                    Tx           <=1'b1 ;
                end
            end
        BROADCAST_S:
            begin
                count <= count + manch ;
                manch <= ~ manch ;
                if( (count == 3'h7) && ( manch ) )
                begin
                    addr         <= addr + 1'b1 ;
                    eth_data_reg <= eth_data[addr + 1'b1];
                end
                if( (addr == addr_max) && (count == 3'h7) )
                begin
                    addr    <= addr_zero ;
                    count   <= 3'd0 ;
                    state   <= TP_IDLE ;
                    Tx_idle <= 1'b1 ;
                    Tx      <= 1'b0 ;
						  eth_data_reg <= 8'b0;
                end
            end
        TP_IDLE:
            begin
                count <= count + 1'b1 ;
                if( count == 3'h5 )
                begin
                    Tx_idle <= 1'b0 ;
                    count   <= 3'd0 ;
                    state   <= WAIS_S ;
                end
            end
        endcase
end

initial
begin
    state        = WAIS_S ;
    addr         = addr_zero ;
    count        = 3'd0 ;
    eth_data_reg = 8'h00 ;
    Tx           = 1'b0 ;
    Tx_idle      = 1'b0 ;
    $readmemh("../eth_frame.hex",eth_data) ;
end

endmodule

module nlp 
(
    input               clk,
    output  reg         go,
    output  reg         Tx
);

reg [31:0]  counter ;

always @(posedge clk) 
begin
    counter <= counter + 1'b1 ;
    if ( counter == 160000*2 )
        Tx <= 1'b1 ;
    if ( counter == 160000*2 + 1*2)
    begin
        Tx <= 1'b0 ;
    end 
    if (counter == 160000*2+5*2+5*2+30000)
    begin
        go <= 1'b1 ;
    end
    if (counter == 160000*2+5*2+5*2+5*2+30000)
    begin
        counter <= 32'h00000 ;
        go <= 1'b0 ;
    end
end

initial
begin
    counter = 32'b0 ;
    Tx      = 1'b0 ;
    go      = 1'b0 ;
end

endmodule
