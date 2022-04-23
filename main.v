// project iCECapture. iCE40 based logic analyzer with SUMP OLS compatible. Based on iCE40LP1K CM36. 
// Coded by TinLethax 2021/11/26 +7

// Nicey UART IP from Piotr Esden-Tempski
`include "uart_rx.v"
`include "uart_tx.v"

// Block RAM module, We use ALL bits of BRAM as STORAGE.
module BRAM(
	input RWCLK,
	
	input [7:0]BRAM_IN,
	output reg [7:0]BRAM_OUT,
	
	input [12:0] BRAM_ADDR_R,
	input [12:0] BRAM_ADDR_W,
	
	input B_CE_W,
	input B_CE_R);
	
reg [7:0] mem1 [8191:0];// 8192 Bytes worth of BRAM as logic CAP mem.
integer i;

initial begin
	for(i = 0; i < 8192; i++)// start with blank memory with 0 instead of x so that we can infer Yosys for BRAM.
		mem1[i] <= 'd0;
end

always@(posedge RWCLK) begin
	if(B_CE_R) 
		BRAM_OUT <= mem1[BRAM_ADDR_R];	
		
	if(B_CE_W) 
		mem1[BRAM_ADDR_W] <= BRAM_IN;

end	

	
endmodule// BRAM


module main (
	
	input CAP_CLK, //48MHz sampling clock as system clock.
	input [7:0]CAP,// 8 Input pins used for data capturing 
	
	// UART
	input RX,
  	output wire TX,
	
	//Interrupt for host.
	output wire INT0);
	
		
// Memory stuffs.
// From input channels 
reg [12:0]cap_wr_addr = 0;
reg cap_wr_ce = 0;
// to HOST
wire [7:0]cap2host;
reg [12:0]cap_rd_addr = 0;
reg cap_rd_ce = 0;

// Command buffer.
reg [7:0]cmdbuf[4:0];// 1 or 5 bytes format 
reg [1:0]cmd_byte_n = 0; 

// reading from BRAM require a careful steps. FSM is involved.
reg [2:0]bram_steps = 0; 

// params for command and flags
parameter cap_size = 8192 - 1;
reg cmd_max = 0;
// commands 
// 1 byte cmd 
parameter cmd_arm = 8'h01;
parameter cmd_reset = 8'h00;
parameter cmd_id = 8'h02;

// 5 bytes cmd
parameter cmd_settrigmask = 8'hc0;
parameter cmd_settrigsens = 8'hc1;
parameter cmd_setdiv = 8'h80;

reg [23:0]cap_div = 0;

parameter cap_48m = 8'd3;
parameter cap_24m = 8'd7;
parameter cap_12m = 8'd15;
parameter cap_6m = 8'd32;
parameter cap_2m = 8'd99;

// capture counter limit for each sample rate.
reg [3:0]cap_counter = 0;
reg cap_limit = 0;

// Trigger mask and sensitivity.
// Trigger mask is to set which will be use as trigger.
// mask : [7:0,CH7, CH6, CH5, CH4, CH3, CH2, CH1, CH0].
// Sensitivity is use for rising or falling detection.
// sensi : [7:0, CH7, CH6, CH5, CH4, CH3, CH2, CH1, CH0].
reg [7:0]trig_mask = 0;
reg [7:0]trig_sens = 0;

// INT flag
// Interrupt flag set to 1 after capture is done.
reg INT_flag = 0;
assign INT0 = INT_flag;

BRAM capRam(
	.RWCLK(CAP_CLK),
	//.RCLK(CAP_CLK),
	.BRAM_IN(CAP),
	.BRAM_OUT(cap2host),
	
	.BRAM_ADDR_R(cap_rd_addr),
	.BRAM_ADDR_W(cap_wr_addr),
	
	.B_CE_W(cap_wr_ce),
	.B_CE_R(cap_rd_ce)
	);

localparam clk_freq = 48_000_000; // 48MHz
localparam baud = 115200;

/* instantiate the rx1 module */
wire rxready;
wire [7:0] rx1_data;
uart_rx #(clk_freq, baud) urx1 (
	.clk(CAP_CLK),
	.rx(RX),
	.rx_ready(rxready),
	.rx_data(rx1_data)
);

/* instantiate the tx1 module */
reg tx1_start;
wire txbusy;
wire tx1_data;
reg tx_id;
assign tx1_data = tx_id;
uart_tx #(clk_freq, baud) utx1 (
	.clk(CAP_CLK),
	.tx_start(tx1_start),
	.tx_data(TX_sel ? tx1_data : cap2host),
	.tx(TX),
	.tx_busy(txbusy)
);	
	

// report id.
reg [7:0] id [31:0];
reg [4:0] id_cnt = 0;
reg TX_sel = 0;// select between 0 Captured data to TX or 1 ID report to TX.

initial begin
	// Name report
	id[0] <= 'h01;
	// Name : iCECapture
	id[1] <= 'h69;// i
	id[2] <= 'h43;// C
	id[3] <= 'h45;// E
	id[4] <= 'h43;// C
	id[5] <= 'h61;// a
	id[6] <= 'h70;// p
	id[7] <= 'h74;// t
	id[8] <= 'h75;// u
	id[9] <= 'h72;// r
	id[10] <= 'h65;// e
	
	id[11] <= 'h00;// end marker 
	
	// Version report
	id[12] <= 'h02;
	// Version : 0.1
	id[13] <= 'h30;// 0
	id[14] <= 'h2e;// .
	id[15] <= 'h31;// 1
	
	id[16] <= 'h00;// end marker 
	
	// Sample memory depth (8KiB).
	id[17] <= 'h21;
	// Sample mem depth (uint32_t) MSB first
	id[18] <= 'h00;
	id[19] <= 'h00;
	id[20] <= 'h20;
	id[21] <= 'h00;
	
	// Sample rate, Maximum is 48MSa/s.
	id[22] <= 'h23;
	id[23] <= 'h02;
	id[24] <= 'hdc;
	id[25] <= 'h6c;
	id[26] <= 'h00;
	
	// Input channel number report (8 channels)
	id[27] <= 'h40;
	id[28] <= 'd8;// we have 8 channels 
	
	// Protocol version (2)
	id[29] <= 'h41;
	id[30] <= 'h02;
	
	// End marking
	id[31] <= 'h00;

end
	
// main FSM.

reg [1:0]main_fsm_cnt = 0;

always@(posedge CAP_CLK)begin

	case(main_fsm_cnt)
	0: begin// Wait for RX
	
		if(rxready) begin
			// copy data from UART to command buffer.
			if(cmd_byte_n <= cmd_max) 
				cmd_byte_n <= 0;
			else
				cmd_byte_n <= cmd_byte_n + 1;
				
			cmdbuf[cmd_byte_n] <= rx1_data;// store all 4 bytes into command buffer, auto-increment.

			// command length 
			case(cmdbuf[0])
				// 1 byte command variants.
				cmd_arm,
				cmd_reset,
				cmd_id:	cmd_max <= 0;

				// 5 bytes command variants. 
				cmd_setdiv,
				cmd_settrigmask,
				cmd_settrigsens: cmd_max <= 4;
				
				default: cmd_max <= 0;
			endcase 
			
			if (cmd_byte_n == cmd_max) begin // received all 5 bytes, then decoder begins. 

				case(cmdbuf[0])
						// Flag: Capture. Capture start immediately after this command is decoded.
						// Format [0:cmd]
						cmd_arm: begin
							case(cap_div)
								cap_48m,
								cap_24m,
								cap_12m,
								cap_6m,
								cap_2m: begin 
									main_fsm_cnt <= 1; // move to Start Capture stage.
									cap_wr_ce <= 1;// Enable Mem write.
								end
								default: begin
									main_fsm_cnt <= main_fsm_cnt;
									cap_wr_ce <= 0;// Not Enable Mem write.
								end
							endcase
							
							case(cap_div)
								cap_48m: begin 
									cap_limit <= 0;
									
									end
								cap_24m: begin
									cap_limit <= 1;
									
									end
								cap_12m: begin
									cap_limit <= 2;
									
									end
								cap_6m: begin 
									cap_limit <= 4;
									
									end
								cap_2m: begin 
									cap_limit <= 12;
									
									end
							endcase//cmd_div case 
							
						end
							
						cmd_id: begin // Report SUMP ID to OLS.
							TX_sel <= 1;
							main_fsm_cnt <= 4;
						end
							
						// Flag: Trigger mask setup, this will select which pin we want to be used as trigger
						// Format [0:cmd][1:LSB][2:LWRMID][3:HGRMID][4:MSB] but we only care [1:LSB] since we only have 8 channel inputs
						cmd_settrigmask: begin
								trig_mask <= cmdbuf[1];
							end
						
						// Flag: Trigger sensitivity setup, whether triggers when falling or rising edge.
						// Format [0:cmd][1:LSB][2:LWRMID][3:HGRMID][4:MSB], again we only care [1:LSB] since we only have 8 channel inputs
						cmd_settrigsens: begin
								trig_sens <= cmdbuf[1];
							end
						
						// Flag: set divider, this will select sample rate.
						// Format [0:cmd][1:LSB][2:MID][3:MSB][4:IGNORED]
						cmd_setdiv: begin
								cap_div[7:0] <= cmdbuf[1];
								cap_div[15:8] <= cmdbuf[2];
								cap_div[23:16] <= cmdbuf[3];
							end
					
					
				endcase
			
			end // cmd decoder if 
		
		end
	
	end
	
	1: begin// Start Capture
	
		cap_counter <= cap_counter + 1;
		if(cap_counter == cap_limit) 
			cap_counter <= 0;

		if(cap_wr_addr == cap_size)begin// Capture done. move to TX stage.
			cap_wr_ce <= 0;
			main_fsm_cnt <= 2;
		end
		else begin// Still capturing. 
		
			if(cap_limit == 0) begin // 48MSa/s running as System clock.
				cap_wr_addr <= cap_wr_addr + 1;
			end
			else begin// sample rate less than 48MSa/s capture depends on free-running counter.
				if(cap_counter == 0) begin 
					cap_wr_addr <= cap_wr_addr + 1;
				end

			end
			
		main_fsm_cnt <= main_fsm_cnt;
		
		end
			
	end
	
	2: begin// Start TX to host.
		
		INT_flag <= 1;
		
		if(txbusy)begin // if busy, just wait until transmit done.
			tx1_start <= 0;
		end
		else begin // send data back to PC
		
			if(cap_rd_addr == cap_size) begin
				cap_rd_ce <= 0;// diable BRAM read 
				main_fsm_cnt <= 3;
				INT_flag <= 0;
				bram_steps <= 0;
			end
			else begin
				
				main_fsm_cnt <= main_fsm_cnt;
				
				// enable BRAM read back 
				cap_rd_ce <= 1; 
			
				// finite state machine for BRAM read back.
				case(bram_steps)
				0:	begin// write address to ram
					cap_rd_addr <= cap_rd_addr + 1;
					bram_steps <= 1;// move to next step, wait
					end
				1: /*do nothing*/ bram_steps <= 2;
				2: begin // read from ram and write to UART TX
					//tx1_data <= cap2host;
					tx1_start <= 1;
					bram_steps <= 0;
					end
				endcase
				
			end
		end//if(tx1_busy)
	
	end
	
	3: main_fsm_cnt <= 0;
	
	4:begin// Special stage, used for id command 
	
		if(txbusy)begin // if busy, just wait until transmit done.
			tx1_start <= 0;
		end
		else begin // send data back to PC
			id_cnt <= id_cnt + 1;
			if(id_cnt == 31)begin
				id_cnt <= 0;
				main_fsm_cnt <= 0;
				TX_sel <= 0;
				tx1_start <= 0;
			end
			else begin
				tx1_start <= 1;
				tx_id <= id[id_cnt];
				main_fsm_cnt <= main_fsm_cnt;
			end
			
		end//if(tx1_busy)
	
	end
	
	endcase

end	

endmodule
/* report ID example 

reg [7:0]id[32];

initial begin
	// Name report
	id[0] <= 'h01;
	// Name : iCECapture
	id[1] <= 'h69;// i
	id[2] <= 'h43;// C
	id[3] <= 'h45;// E
	id[4] <= 'h43;// C
	id[5] <= 'h61;// a
	id[6] <= 'h70;// p
	id[7] <= 'h74;// t
	id[8] <= 'h75;// u
	id[9] <= 'h72;// r
	id[10] <= 'h65;// e
	
	id[11] <= 'h00;// end marker 
	
	// Version report
	id[12] <= 'h02;
	// Version : 0.1
	id[13] <= 'h30;// 0
	id[14] <= 'h2e;// .
	id[15] <= 'h31;// 1
	
	id[16] <= 'h00;// end marker 
	
	// Sample memory depth (8KiB).
	id[17] <= 'h21;
	// Sample mem depth (uint32_t) MSB first
	id[18] <= 'h00;
	id[19] <= 'h00;
	id[20] <= 'h20;
	id[21] <= 'h00;
	
	// Sample rate, Maximum is 48MSa/s.
	id[22] <= 'h23;
	id[23] <= 'h02;
	id[24] <= 'hdc;
	id[25] <= 'h6c;
	id[26] <= 'h00;
	
	// Input channel number report (8 channels)
	id[27] <= 'h40;
	id[28] <= 'd8;// we have 8 channels 
	
	// Protocol version (2)
	id[29] <= 'h41;
	id[30] <= 'h02;
	
	// End marking
	id[31] <= 'h00;

end


	usart_write(0x01);// name report
	prntf("ComeNCapture");
	usart_write(0x00);// end marker

	usart_write(0x02);// version report
	prntf("2.0");
	usart_write(0x00);// end marker

	// Sample memory size (16Kbytes)
	usart_write(0x21);// Sample size report in uint32_t format 
	usart_write(0x00);
	usart_write(0x00);
	usart_write(0x3F);
	usart_write(0xFF);

	// Sample rate, assuming it's 8MHz
	usart_write(0x23);// Sample rate report in uint32_t format 
	usart_write(0x00);
	usart_write(0x7A);
	usart_write(0x12);
	usart_write(0x00);

	// we have 5 usable channels 
	usart_write(0x40);// report channel number
	usart_write(0x05);// 5 channels
	
	// protocol version is  2 
	usart_write(0x41);// report protocol version
	usart_write(0x02);

	usart_write(0x00);// end marking

*/