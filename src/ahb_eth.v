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

    sm_eth sm_eth_0
	(
		// AHB-side
		.HCLK 		( HCLK 		),
    	.HRESETn 	( HRESETn 	),
    	.bSel 		( pm_valid  ),
    	.bAddr  	( pm_addr 	),
    	.bWrite 	( pm_we 	),
    	.bWData 	( pm_wd 	),
    	.bRData 	( pm_rd 	),
    	// ETH-side
    	.eth_clk 	( eth_clk   ),
    	.eth_rstn 	( eth_rstn 	),
    	.Txp 		( Txp 		),
    	.Txn 		( Txn 		),
    	.Led_Tx 	( Led_Tx 	)
	);
    
endmodule

module sm_eth
(
	// AHB-side
	input             HCLK,
    input             HRESETn,
    input             bSel,
    input      [31:0] bAddr,
    input             bWrite,
    input      [31:0] bWData,
    output     [31:0] bRData,
    // ETH-side
    input             eth_clk,
    input             eth_rstn,
    output            Txp,
    output            Txn,
    output            Led_Tx
);

	reg         c_d;
    wire 		mem_wr;
    wire 		reg_wr;
    wire [7:0]  q_out;
    wire [7:0]	mem2eth;
    wire        t_complete;
    wire        b_end;
    wire [7:0]  addr_count_out;
    wire        mem_rd;
    wire        a_clr_com;

    assign mem_wr = bWrite & (   c_d ) & bSel ;
    assign reg_wr = bWrite & ( ~ c_d ) & bSel ;
    assign bRData = q_out ;

    wire [7:0] rst_p_reg;
    assign rst_p_reg = {6'h3f, ~ a_clr_com, ~ t_complete } ;

    p_reg p_reg_0
    (
        .clk    ( HCLK        ),
        .rst    ( rst_p_reg   ),
        .data   ( bWData[7:0] ),
        .wr     ( reg_wr      ),
        .q      ( q_out       )
    );

    eth_mem 
    #(
        .data_width(8),
        .addr_width(8)
    ) 
    eth_mem_0
    (
    // Port A
        .a_clk          ( HCLK           ),
        .a_wr           ( mem_wr         ),
        .a_din          ( bWData[7:0]    ),  
        .a_clr          ( q_out[1]       ),  //TX_FLUSH  
        .a_clr_com      ( a_clr_com      ),
    // Port B
        .b_clk          ( eth_clk        ),
        .b_rd           ( mem_rd         ),
        .b_dout         ( mem2eth        ),
        .b_end          ( b_end          ),
        .addr_count_out ( addr_count_out )
    );

    wire eth_data_s;
    wire Tx_nlp;
    wire Txn_nlp;
    wire Txp_nlp;
    wire Txn_TxD;
    wire Txp_TxD;
    wire Tx_w;

    assign Txp_nlp = Tx_nlp == 1'b1 ? 1'b1 : 1'b0 ;
    assign Txn_nlp = Tx_nlp == 1'b1 ? 1'b0 : 1'b0 ;

    assign Txp_TxD = Tx_w ?   eth_data_s : 1'b0 ;
    assign Txn_TxD = Tx_w ? ~ eth_data_s : 1'b0 ;

    assign Txn = Txn_nlp | Txn_TxD ;
    assign Txp = Txp_nlp | Txp_TxD ;

    assign Led_Tx = ~ ( Tx_w ) ;
    wire [31:0]     c_out;
    wire transmit;
    assign transmit = q_out[0] & ( c_out[31:2] > ( addr_count_out + 1000 ) ) & ( ( (1440000 >> 2) - c_out[31:2] ) > ( {addr_count_out,3'b0} + 1000 ) );

    nlp nlp_0
    (
        .clk      ( eth_clk   ),
        .Tx       ( Tx_nlp    ),
        .c_out    ( c_out     )
    );

    eth_frame eth_frame_0
    (
        .transmit   ( transmit      ),
        .clk        ( eth_clk       ),
        .rst_n      ( eth_rstn      ),
        .eth_data_s ( eth_data_s    ),
        .Tx_w       ( Tx_w          ),
        .mem_rd     ( mem_rd        ),
        .eth_in     ( mem2eth       ),
        .t_complete ( t_complete    ),
        .b_end      ( b_end         )
    );
    
    always @ (*)
        case(bAddr[3:0])
            default           : c_d = 1'b0 ;
            `SM_ETH_REG_COM   : c_d = 1'b0 ;
            `SM_ETH_REG_DATA  : c_d = 1'b1 ;
        endcase
    
    initial
    begin
        c_d = 1'b0 ;
    end

endmodule


module eth_mem #(
    parameter data_width = 8,
    parameter addr_width = 8
)(
    // Port A
    input   wire                      a_clk,
    input   wire                      a_wr,
    input   wire    [data_width-1:0]  a_din,
    input   wire                      a_clr,
    output  reg                       a_clr_com,
    
    // Port B
    input   wire                      b_clk,
    input   wire                      b_rd,
    output       [data_width-1:0]     b_dout,
    output  reg                       b_end,
    output       [addr_width-1:0]     addr_count_out
);

    reg [addr_width-1:0] addr_count;
    reg [addr_width-1:0] b_addr ;
    reg [addr_width-1:0] a_addr ;
    // Shared memory
    reg [data_width-1:0] mem [(2**addr_width)-1:0];

    assign addr_count_out = addr_count ;
    assign b_dout = mem[b_addr];
    
    // Port A
    always @(posedge a_clk) 
    begin
        a_clr_com <= 1'b0 ;
        if( a_clr )
        begin
            a_addr <= 8'h08 ;
            a_clr_com <= 1'b1 ;
            addr_count <= 8'h08 ;
        end
        if(a_wr) begin
            a_addr <= a_addr + 1'b1;
            if(a_addr > addr_count)
                a_addr <= 8'h08;
            mem[a_addr] <= a_din;
            addr_count <= addr_count + 1'b1;
        end
    end
 
    // Port B
    always @(posedge b_clk)
    begin
        if( b_rd )
        begin
            b_end <= 1'b0 ;
            b_addr <= b_addr + 1'b1;
            if (b_addr == addr_count )
            begin
                b_addr <= 8'h0;
                //b_end <= 1'b1 ;
            end
            if (b_addr == addr_count -1 )
            begin
                b_end <= 1'b1;
            end
        end
    
    end
    
    initial
    begin
        a_clr_com = 1'b0 ;
        b_end = 1'b0;
        addr_count = 8'h8;
        b_addr = 8'h0;
        a_addr = 8'h8;
        $readmemh("../eth_frame.hex",mem) ;
        mem[0] = 8'h55;
        mem[1] = 8'h55;
        mem[2] = 8'h55;
        mem[3] = 8'h55;
        mem[4] = 8'h55;
        mem[5] = 8'h55;
        mem[6] = 8'h55;
        mem[7] = 8'hD5;
    end
 
endmodule

module p_reg
#(
    parameter data_width=8
)(
    input                       clk,
    input   [data_width-1:0]    rst,
    input   [data_width-1:0]    data,
    input                       wr,
    output  [data_width-1:0]    q
);

    genvar k;
    generate 
    for ( k = 0 ; k < data_width ; k = k + 1 )
    begin : generate_p_reg
        p_one_ff p_one_ff_
        (
            .clk    ( clk     ),
            .rst    ( rst[k]  ),
            .data   ( data[k] ),
            .wr     ( wr      ),
            .q      ( q[k]    )
        );
    end
    endgenerate

endmodule

module p_one_ff
(
    input       clk,
    input       rst,
    input       data,
    input       wr,
    output reg  q
);

    always @( posedge clk or negedge rst )
    begin
        if ( ~ rst )
            q <= 1'b0;
        else if ( wr )
            q <= data;
    end

    initial
    begin
        q = 1'b0;
    end

endmodule

module eth_frame
(
    input             transmit,
    input             clk,
    input             rst_n,
    output            eth_data_s,
	output	     	  Tx_w,
    output  reg       mem_rd,
    input [7:0]       eth_in,
    output  reg       t_complete,
    input             b_end
);

    localparam eth_frame_length = 'd128 ;
    localparam addr_max         = {$clog2(eth_frame_length){1'b1}} ;
    localparam addr_zero        = {$clog2(eth_frame_length){1'b0}} ;

    reg [1:0]   divider;

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
                t_complete <= 1'b0 ;
                divider <= 2'b0;
            end
        else
            case( state )
            WAIS_S:
                begin
                    Tx      <= 1'b0 ;
                    manch   <= 1'b0 ;
                    Tx_idle <= 1'b0 ;
                    t_complete <= 1'b0 ;
                    divider <= 2'b0;
                    if( transmit == 1'b1 )
                    begin
                        state        <= BROADCAST_S ;
                        eth_data_reg <= eth_in ;
                        Tx           <=1'b1 ;
                        mem_rd       <= 1'b1;
                    end
                end
            BROADCAST_S:
                begin
                    divider <= divider + 1'b1;
                    mem_rd <= 1'b0;
                    if( divider == 2'b11 )
                    begin
                        divider <= 2'b0;
                    
                        count <= count + manch ;
                        manch <= ~ manch ;
                        if( (count == 3'h7) && ( manch ) )
                        begin
                            mem_rd       <= 1'b1;
                            addr         <= addr + 1'b1 ;
                            eth_data_reg <= eth_in;
                        end
                    
                        if( (count == 3'h7) && ( b_end ) && ( manch ) )
                        begin
                            addr    <= addr_zero ;
                            count   <= 3'd0 ;
                            state   <= TP_IDLE ;
                            Tx_idle <= 1'b1 ;
                            Tx      <= 1'b0 ;
                            eth_data_reg <= 8'b0;
                            divider <= 2'b0 ;
                        end
                    end
                end
            TP_IDLE:
                begin
                    mem_rd <= 1'b0;
                    divider <= divider + 1'b1 ;
                    if (divider == 2'b11 )
                    begin
                        divider <= 2'b0;
                        count <= count + 1'b1 ;
                        
                        if( count == 3'h5 )
                        begin
                            Tx_idle <= 1'b0 ;
                            count   <= 3'd0 ;
                            state   <= WAIS_S ;
                            t_complete <= 1'b1 ;
                        end
                    end
                end
            endcase
    end

    initial
    begin
        divider      = 2'b0 ;
        mem_rd       = 1'b0 ;
        state        = WAIS_S ;
        addr         = addr_zero ;
        count        = 3'd0 ;
        eth_data_reg = 8'h00 ;
        Tx           = 1'b0 ;
        Tx_idle      = 1'b0 ;
        t_complete   = 1'b0 ;
        $readmemh("../eth_frame.hex",eth_data) ;
    end

endmodule

module nlp 
(
    input               clk,
    output  reg         go,
    output  reg         Tx,
    output  [31:0]      c_out
);

    reg [31:0]  counter ;

    assign c_out = counter ;

    always @(posedge clk) 
    begin
        counter <= counter + 1'b1 ;
        if ( counter == 0 )
            Tx <= 1'b1 ;
        if ( counter == 8)
        begin
            Tx <= 1'b0 ;
        end 
        if (counter == 1440000)
        begin
            counter <= 32'h00000 ;
        end
    end

    initial
    begin
        counter = 32'b0 ;
        Tx      = 1'b0 ;
        go      = 1'b0 ;
    end

endmodule
