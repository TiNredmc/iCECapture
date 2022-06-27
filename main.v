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
parameter cap_size = 8192 - 1;// capture size is 8192 bytes, stop the capture when address is reached 'h8191.
reg cmd_max = 0;
// commands 
// 1 byte cmd 
parameter cmd_arm = 8'h01;
parameter cmd_reset = 8'h00;
parameter cmd_id = 8'h02;
parameter cmd_metadat = 8'h04;

// 5 bytes cmd
parameter cmd_settrigmask = 8'hc0;
parameter cmd_settrigsens = 8'hc1;
parameter cmd_settrigconf = 8'hc2;
parameter cmd_setdiv = 8'h80;
parameter cmd_setsamcnt = 8'h81;// set read count and delay count (in unit of sample/4), this probably used for setting pre-post trigger ratio.
reg [23:0]cap_div = 0;

// Dividers for each sample rate
// Calculated with Div = (100,000,000 / Sample_rate) - 1 
// Dividing by discard the decimal point. i.e. 1.9 = 1, 1.2 = 1
// Sample_rate is in Hertz unit.

// Mhz range
parameter cap_48m = 8'd1;
parameter cap_24m = 8'd3;
parameter cap_12m = 8'd7;
parameter cap_6m = 8'd15;
parameter cap_4m = 8'd24;
parameter cap_2m = 8'd49;
parameter cap_1m = 8'd99;

// kHz range
parameter cap_500k = 8'd199;
parameter cap_200k = 16'd499;
parameter cap_100k = 16'd999;

// capture counter limit for each sample rate.
reg [7:0]cap_counter = 0;
reg [7:0]cap_limit = 0;

// Trigger mask and sensitivity.
// Trigger mask is to set which will be use as trigger.
// mask : [7:0,CH7, CH6, CH5, CH4, CH3, CH2, CH1, CH0].
// Sensitivity is use for rising or falling detection.
// sensi : [7:0, CH7, CH6, CH5, CH4, CH3, CH2, CH1, CH0].
reg [7:0]trig_mask = 0;
reg [7:0]trig_sens = 0;
// trigger address register stores the memory address of the input data that match trigger setting.
reg [12:0]trig_addr = 0;
// trigger end address store the address of "what suppose to be" last input data captured. This make sure that we don't override previous capture that
// was a part of pre-trigger. 
reg [12:0]trig_end_addr = 0;

// check for input to match trigger condition.
wire trig_match;
assign trig_match = !((CAP ^ trig_sens) & trig_mask);

// store how many sample to read back from memory and how many sample to capture after trigger is fired.
reg [15:0]read_size = 0;
reg [15:0]delay_size = 0;
// from 2 regs above. read_size - delay_size == pre trigger sample count (size).
reg [15:0]trig_pre_size = 0;

// state machine for trigger
reg [1:0]trig_fsm = 0;

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
reg TX_sel = 0;// select between 0 Captured data to TX or 1 ID report to TX.
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
	
// metadat and id reports.
reg [7:0] mdtd [31:0];
reg [7:0] id [3:0];
reg [4:0] id_cnt = 0;
	
// main FSM.

reg [2:0]main_fsm_cnt = 0;

always@(posedge CAP_CLK)begin

	case(main_fsm_cnt)
	0: begin // Initialize default values.
		// "1ALS" report
		id[0] <= 'h31;// 1
		id[1] <= 'h41;// A
		id[2] <= 'h4C;// L
		id[3] <= 'h53;// S
		
		// Name report
		mdtd[0] <= 'h01;
		// Name : iCECapture
		mdtd[1] <= 'h69;// i
		mdtd[2] <= 'h43;// C
		mdtd[3] <= 'h45;// E
		mdtd[4] <= 'h43;// C
		mdtd[5] <= 'h61;// a
		mdtd[6] <= 'h70;// p
		mdtd[7] <= 'h74;// t
		mdtd[8] <= 'h75;// u
		mdtd[9] <= 'h72;// r
		mdtd[10] <= 'h65;// e
		
		mdtd[11] <= 'h00;// end marker 
		
		// Version report
		mdtd[12] <= 'h02;
		// Version : 0.1
		mdtd[13] <= 'h30;// 0
		mdtd[14] <= 'h2e;// .
		mdtd[15] <= 'h31;// 1
		
		mdtd[16] <= 'h00;// end marker 
		
		// Sample memory depth (8KiB).
		mdtd[17] <= 'h21;
		// Sample mem depth (uint32_t) MSB first
		mdtd[18] <= 'h00;
		mdtd[19] <= 'h00;
		mdtd[20] <= 'h20;
		mdtd[21] <= 'h00;
		
		// Sample rate, Maximum is 48MSa/s.
		mdtd[22] <= 'h23;
		mdtd[23] <= 'h02;
		mdtd[24] <= 'hdc;
		mdtd[25] <= 'h6c;
		mdtd[26] <= 'h00;
		
		// Input channel number report (8 channels)
		mdtd[27] <= 'h40;
		mdtd[28] <= 'd8;// we have 8 channels 
		
		// Protocol version (2)
		mdtd[29] <= 'h41;
		mdtd[30] <= 'h02;
		
		// End marking
		mdtd[31] <= 'h00;
		
		main_fsm_cnt <= 1;
	end
	
	1: begin// Wait for RX
	
		if(rxready) begin
			// copy data from UART to command buffer.
			if(cmd_byte_n == cmd_max) 
				cmd_byte_n <= 0;
			else
				cmd_byte_n <= cmd_byte_n + 1;
				
			cmdbuf[cmd_byte_n] <= rx1_data;// store all 1 or 5 bytes into command buffer, auto-increment.

			// command length 
			case(cmdbuf[0])
				// 1 byte command variants.
				cmd_arm,
				cmd_reset,
				cmd_id:	cmd_max <= 0;

				// 5 bytes command variants. 
				cmd_setdiv,
				cmd_settrigmask,
				cmd_settrigsens,
				cmd_setsamcnt: cmd_max <= 4;
				
				default: cmd_max <= 0;
			endcase 
			
			if (cmd_byte_n == cmd_max) begin // received 1 or 5 bytes, then decoder begins. 

				case(cmdbuf[0])
						// Flag: Capture. Capture start immediately after this command is decoded.
						// Format [0:cmd]
						cmd_arm: begin
							case(cap_div)
								cap_48m,
								cap_24m,
								cap_12m,
								cap_6m,
								cap_2m,
								cap_500k,
								cap_200k,
								cap_100k: begin 
									main_fsm_cnt <= 2; // move to Start Capture stage.
									cap_wr_ce <= 1;// Enable Mem write.
									cap_wr_addr <= 0;
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
								cap_4m: begin
									cap_limit <= 6;
									
									end									
								cap_2m: begin 
									cap_limit <= 12;
								
									end
								cap_1m: begin
									cap_limit <= 24;
									
									end
								cap_500k: begin
									cap_limit <= 48;
								
									end
								cap_200k: begin
									cap_limit <= 120;
									
									end
								cap_100k: begin
									cap_limit <= 240;
									
									end
									
							endcase//cmd_div case 
							
						end
							
						cmd_metadat: begin // Report SUMP Metadata to OLS.
							TX_sel <= 1;
							main_fsm_cnt <= 5;
						end
							
						cmd_id: begin // OLS expect "1ALS" send over UART.
							TX_sel <= 1;
							main_fsm_cnt <= 6;
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
						
						// Flag: Set Read count (how many sample to read back) and Delay count (how many sample to capture after the trigger is fired). All in unit of samples/4
						// Format [0:cmd][1:readcnt_LSB][2:readcnt_MSB][3:dlycnt_LSB][4:dlycnt_MSB]
						cmd_setsamcnt: begin
								// convert data back from sample/4 to sample unit.
								read_size <= {cmdbuf[2], cmdbuf[1]} << 2; 
								delay_size <= {cmdbuf[4], cmdbuf[3]} << 2;
								trig_pre_size <= read_size - delay_size;// calculate sample size of pre trigger.
						end
						
						// Flag: set divider, this will select sample rate.
						// Format [0:cmd][1:LSB][2:MID][3:MSB][4:IGNORED]
						cmd_setdiv: begin
								cap_div[7:0] <= cmdbuf[1];
								cap_div[15:8] <= cmdbuf[2];
								cap_div[23:16] <= cmdbuf[3];
						end
					
				endcase
			
			end //if (cmd_byte_n == cmd_max) cmd decoder.
		
		end// if(rxready)
	
	end
	
	2: begin// Start Capture
	
		cap_counter <= cap_counter + 1;
		if(cap_counter == cap_limit) 
			cap_counter <= 0;
			
		if(trig_mask == 8'h00) begin// if somehow entered trigger mode but with trigger mask set all input to 0, goes back to normal capture mode.
		
			if(cap_wr_addr == cap_size)begin// Capture done. move to TX stage.
				cap_wr_ce <= 0;
				cap_wr_addr <= 0;
				main_fsm_cnt <= 3;
				trig_end_addr <= cap_size;
			end
			
		end
		else begin
		
			case(trig_fsm)// Capture with trigger.
			0: begin// wait for trigger to acquire trigger address.
				if(trig_match) begin
					trig_addr <= cap_wr_addr;
					trig_end_addr <= cap_wr_addr + delay_size;
					trig_fsm <= 1;
				end
					
			end
			
			1: begin// wait until post trigger input data is written to the memory. 
				if(cap_wr_addr == trig_end_addr)begin
					// reached the end of "trigger" memory. Stop capture.
					cap_wr_ce <= 0;
					cap_wr_addr <= 0;
					main_fsm_cnt <= 3;
					trig_fsm <= 0;
					cap_rd_addr <= trig_end_addr + 1;// start sending the pre capture. Then counter rolls over back to 0 and end at trig_end_addr.
				end	
				
			end
			
			endcase
			
		end

		case(cap_limit)// BRAM writing address up counter.
		0: cap_wr_addr <= cap_wr_addr + 1;
		default : begin
			if(cap_counter == 0) begin 
				cap_wr_addr <= cap_wr_addr + 1;
				cap_wr_ce <= 1;
			end
			else
				cap_wr_ce <= 0;
		
		end
		
		endcase

			
		main_fsm_cnt <= main_fsm_cnt;
			
	end
	
	3: begin// Start TX to host.
		
		INT_flag <= 1;
		TX_sel <= 0;
		
		if(txbusy)begin // if busy, just wait until transmit done.
			tx1_start <= 0;
		end
		else begin // send data back to PC
		
			if(cap_rd_addr == trig_end_addr) begin
				cap_rd_ce <= 0;// disble BRAM read 
				cap_rd_addr <= 0; // reset BRAM read address to 0.
				main_fsm_cnt <= 4;
				INT_flag <= 0;
				bram_steps <= 0;
			end
			else begin

				// finite state machine for BRAM read back.
				case(bram_steps)
				0:	begin// enable bram write and write first address to bram.
					// enable BRAM read back 
					cap_rd_ce <= 1; 
					cap_rd_addr <= 0;
					bram_steps <= 1;// move to next step, wait
					
				end
				1: begin
					// write next address and read data from previous address on next clock cycle.
					cap_rd_addr <= cap_rd_addr + 1;
					bram_steps <= 2;
				end
				
				2: begin // read from ram and write to UART TX
					//tx1_data <= cap2host;
					tx1_start <= 1;
					bram_steps <= 1;
					end
				endcase
				
				main_fsm_cnt <= main_fsm_cnt;
				
			end
		end//if(tx1_busy)
	
	end
	
	4: main_fsm_cnt <= 1;
	
	5:begin// Special stage, used for metadata report. 
	
		if(txbusy)begin // if busy, just wait until transmit done.
			tx1_start <= 0;
		end
		else begin // send data back to PC
			id_cnt <= id_cnt + 1;
			if(id_cnt == 31)begin
				id_cnt <= 0;
				main_fsm_cnt <= 1;
				TX_sel <= 0;
				tx1_start <= 0;
			end
			else begin
				tx1_start <= 1;
				tx_id <= mdtd[id_cnt];
				main_fsm_cnt <= main_fsm_cnt;
			end
			
		end//if(tx1_busy)
	
	end
	
	6:begin// Special stage, used for ID report "1ALS".
	
		if(txbusy)begin // if busy, just wait until transmit done.
			tx1_start <= 0;
		end
		else begin // send data back to PC
			id_cnt <= id_cnt + 1;
			if(id_cnt == 3)begin
				id_cnt <= 0;
				main_fsm_cnt <= 1;
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


/* Pre / Post trigger calculation algorithm. 
	input from pc -> Read count, Delay count (all in sample/4 unit).
	Example Read count == 1 -> 4 sample.
1. Convert sample/4 into sample by multiplying with 4
	read_size <= read_count << 2;// m << n means m times 2 to the n.
	delay_size <= delay_count << 2;
2. calculate pre trigger sample count
	tirg_pre_size <= read_size - delay_size;
3. calculate the end address of trigger capture.
	trig_addr <= cap_wr_addr;// trig_addr will be equal to writing address when trigger is fired. 
	trig_end_addr <= cap_wr_addr - trig_addr;
4. wait until trig_end_addr equal to the end address calculate with triggered address plus delay_size.
	if(trig_end_addr == (trig_addr + delay_size))
*/