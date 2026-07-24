

module top_fpga (

    input  wire clk_50M,

    // Ultrasonic
    input  wire echo_L,
    input  wire echo_F,
    input  wire echo_R,

    output wire trig_L,
    output wire trig_F,
    output wire trig_R,

    input  wire ENA_1, ENB_1, ENA_2, ENB_2,
    input  wire ir1, ir2, ir3,

    // L298N pins
    output wire IN1, IN2,
    output wire IN3, IN4,
    output wire ENA,
    output wire ENB,

    output reg left, mid, right,

    output wire [16:0] turn_pulse_cnt,
    output wire [15:0] dist_L, dist_F, dist_R,

    inout  wire dht_pin,

    output wire tx,          // HC-05 RX
	 output wire pwm_out,


    output wire [11:0] moisture_raw,

    input  wire adc_dout,
    output wire adc_cs_n,
    output wire adc_din,
    output wire adc_sck,
	 input wire rx ,  //HC-05 TX
	 output wire start,
	 output wire high
);

    // -------------------------------------------------
    // Internal signals
    // -------------------------------------------------
    wire [2:0] move_cmd;
    wire pwm_L, pwm_R;
    wire clk_3125KHz;
    wire reset_n = 1'b1;

    // ---------------- DHT ----------------
    wire [7:0] T_int, T_dec;
    wire [7:0] RH_int, RH_dec;
    wire       dht_valid;

    // Latched DHT values (FIX)
    reg [7:0] T_int_l, T_dec_l;
    reg [7:0] RH_int_l, RH_dec_l;

    // ---------------- UART ----------------
    wire [7:0] tx_data;
    wire       tx_start;
    wire       tx_done;

    // UART trigger (FIX)
    reg [23:0] uart_cnt = 0;
    reg        uart_tick = 0;

    // Moisture (FIX: single driver)
    wire [11:0] moisture_raw_adc;
    assign moisture_raw = moisture_raw_adc;
	 
	 
	 wire servo;
	 
	 wire end_cmd = ((dist_F>900)&&(dist_L>900)&&(dist_R>900));
	 wire [3:0]deadend_count;
	 wire [7:0]mpi_id; 
	 
	 
	 wire [2:0] move_latched;
	 
	 wire [3:0] end_cmd_count; 
	

    // -------------------------------------------------
    // UART periodic trigger (~1 second)
    // -------------------------------------------------
    always @(posedge clk_50M) begin
        if (uart_cnt == 24'd50_000_000) begin
            uart_cnt  <= 0;
            uart_tick <= 1'b1;
        end else begin
            uart_cnt  <= uart_cnt + 1;
            uart_tick <= 1'b0;
        end
    end

    // -------------------------------------------------
    // Latch DHT data (FIX)
    // -------------------------------------------------
    always @(posedge clk_50M) begin
        if (dht_valid) begin
            T_int_l  <= T_int;
            T_dec_l  <= T_dec;
            RH_int_l <= RH_int;
            RH_dec_l <= RH_dec;
        end
    end

    // -------------------------------------------------
    // Clock scaling
    // -------------------------------------------------
    frequency_scaling fl (
        .clk_50M(clk_50M),
        .clk_3125KHz(clk_3125KHz)
    );

    // -------------------------------------------------
    // Ultrasonic Sensors
    // -------------------------------------------------
    t1b_ultrasonic UL (
        .clk_50M(clk_50M),
        .reset(reset_n),
        .echo_rx(echo_L),
        .trig(trig_L),
        .op(),
        .distance_out(dist_L)
    );

    t1b_ultrasonic UF (
        .clk_50M(clk_50M),
        .reset(reset_n),
        .echo_rx(echo_F),
        .trig(trig_F),
        .op(),
        .distance_out(dist_F)
    );

    t1b_ultrasonic UR (
        .clk_50M(clk_50M),
        .reset(reset_n),
        .echo_rx(echo_R),
        .trig(trig_R),
        .op(),
        .distance_out(dist_R)
    );

    // -------------------------------------------------
    // Wall detection
    // -------------------------------------------------
    always @(*) begin
//        left  = (dist_L<285);
        left  = !ir1;
        mid   = (dist_F<80);   //Because front ir is getting used in MPI
//          mid   = !ir2;
        right =(dist_R<285);
    end

    // -------------------------------------------------
    // Maze Solver
    // -------------------------------------------------
    t2c_maze_explorer MAZE (
        .clk(clk_50M),
        .rst_n(reset_n),
		  .start_signal(start),
        .left(left),
        .mid(mid),
        .right(right),
		  .dist_R(dist_R), .dist_L(dist_L),.dist_F(dist_F),
		  .ir2(ir2),
        .move(move_cmd),
		  .end_cmd(end_cmd),
		  .deadend_count(),
		  .mode(mode),
		  .move_latched(move_latched),
		  .one_right_turn(high),
		   .post_fwd_active(active),
			.end_cmd_count(end_cmd_count)

    );

    // -------------------------------------------------
    // PWM
    // -------------------------------------------------
    pwm_generator PWM_L (
        .clk_3125KHz(clk_3125KHz),
        .duty_cycle(4'd8),
        .clk_195KHz(),
        .pwm_signal(pwm_L)
    );
	 
 
    pwm_generator PWM_R (
        .clk_3125KHz(clk_3125KHz),
        .duty_cycle(4'd8),
        .clk_195KHz(),
        .pwm_signal(pwm_R)
    );

    // -------------------------------------------------
    // Motor Driver
    // -------------------------------------------------
	 
	 wire end_cnf;
    t3_motor_driver MOTOR (
        .clk(clk_50M),
        .rst_n(start_rst),
        .move( move_cmd ),
        .pwm_L(pwm_L),
        .pwm_R(pwm_R),
        .ENA_1(ENA_1), .ENB_1(ENB_1),
        .ENA_2(ENA_2), .ENB_2(ENB_2),
        .ir1(ir1), .ir2(ir2), .ir3(ir3),
        .dist_R(dist_R), .dist_L(dist_L),.dist_F(dist_F),
        .IN1(IN1), .IN2(IN2),
        .IN3(IN3), .IN4(IN4),
        .ENA(ENA),
        .ENB(ENB),
        .turn_pulse_cnt(turn_pulse_cnt),
		  .servo_done(servo),
		  .end_cmd(end_cmd),
		  .mode(mode),
		  .move_latched(move_latched),
		  .post_fwd_active(active),
		  .end_cmd_count(end_cmd_count),
		  .one_right_turn(high)

		   
    );
	 wire active;

    // -------------------------------------------------
    // DHT Sensor
    // -------------------------------------------------
    t2a_dht dht_inst (
        .clk_50M(clk_50M),
        .reset(reset_n),
        .sensor(dht_pin),
        .T_integral(T_int),
        .T_decimal(T_dec),
        .RH_integral(RH_int),
        .RH_decimal(RH_dec),
        .Checksum(),
        .data_valid(dht_valid)
    );

    // -------------------------------------------------
    // Moisture Sensor (ONLY driver)
    // -------------------------------------------------
    moisture_sensor moist_inst (
        .dout(adc_dout),
        .clk50(clk_50M),
        .adc_cs_n(adc_cs_n),
        .din(adc_din),
        .adc_sck(adc_sck),
        .d_out_ch0(moisture_raw_adc),
        .led_ind()
    );
	 
	 //---------------------
	 // MPI MAPPER
	 //---------------------
 
   
     wire mode;
	  mpi_mapper mpi_map (
    .clk(clk_3125KHz),
    .reset(start_rst),
    .move(move_cmd),
    .mpi_id(mpi_id),
	 .mode(mode)
);

    // -------------------------------------------------
    // UART FSM
    // -------------------------------------------------
    th_uart_fsm th_fsm (
        .clk(clk_3125KHz),
        .reset(reset_n),
        .data_valid(uart_tick),   // FIXED
        .T_integral(T_int_l),
		  .mpi_id(mpi_id),
        .T_decimal(T_dec_l),
        .RH_integral(RH_int_l),
        .RH_decimal(RH_dec_l),
        .d_out_ch0(moisture_raw_adc),
		  .ir1(ir1), .ir2(ir2), .ir3(ir3),

        .tx_data(tx_data),
        .tx_start(tx_start),
        .tx_done(tx_done),
  		  .end_cmd(end_cmd),

		  .down_wait(down),
		  .one_right_turn(high)

//		  .end_cnf(end_cnf)
//        .end_cmd_count(end_cmd_count)
    );

    // -------------------------------------------------
    // UART TX
    // -------------------------------------------------
    uart_tx uart_inst (
        .clk_3125(clk_3125KHz),
        .parity_type(start_rst),
        .tx_start(tx_start),
        .data(tx_data),
        .tx(tx),
        .tx_done(tx_done)
    );
	 
	 
	  wire [7:0]rx_ms;
	  wire rx_done;
//	  wire start;
	  
	  
	 uart_rx rx_inst(
	 .clk_3125(clk_3125KHz),
    .rx(rx),
    .rx_msg(rx_ms),
    .rx_parity(),
    .rx_complete(rx_done)
	 );
	 
//	
//	  start_command_detector (
//     .clk(clk_3125KHz),
//     .rst_n(1'b1),
//     .rx_msg(rx_ms),        // From your uart_rx
//     .rx_complete(rx_done),       // From your uart_rx
//     .start_signal(start)  // High once START-3-# is received
//); 

reg [7:0] rst_cnt = 0;
reg start_rst = 0;
// FOR PUTTING RESET LOW AND HIGH AGAIN
always @(posedge clk_3125KHz) begin
    if (rst_cnt < 8'd50) begin     // ~50 cycles reset
        rst_cnt   <= rst_cnt + 1'b1;
        start_rst <= 1'b0;         // RESET ACTIVE (LOW)
    end else begin
        start_rst <= 1'b1;         // RELEASE RESET
    end
end

	 
wire down;	 

start_command_detector start_det (
    .clk(clk_3125KHz),
    .rst_n(start_rst),
    .rx_msg(rx_ms),
    .rx_complete(rx_done),
    .start_signal(start)
);

	 
	 servo_180deg servo_mod (
      .clk(clk_50M),        // 50 MHz FPGA clock
      .rst_n(reset_n),      // Active-low reset
      .ir1(ir1),.ir2(ir2),.ir3(ir3),
	   .dist_F(dist_F),
		.dist_L(dist_L),
		.dist_R(dist_R),
	   	// IR sensor inputs
      .pwm_out(pwm_out), // Servo signal output
      .servo_done(servo),
		.turn_pulse_cnt(turn_pulse_cnt),//high when servo task is done
		.down_wait(down)
);



endmodule