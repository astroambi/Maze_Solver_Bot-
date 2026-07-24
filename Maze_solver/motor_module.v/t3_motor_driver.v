module t3_motor_driver (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [2:0] move,

    input  wire [3:0] pwm_L,
    input  wire [3:0] pwm_R,

    input  wire       ENA_1,
    input  wire       ENB_1,
    input  wire       ENA_2,
    input  wire       ENB_2,

    input  wire       ir1, ir2, ir3,
    input  wire [15:0] dist_L, dist_R, dist_F,

    output reg        IN1, IN2,
    output reg        IN3, IN4,

    output wire [3:0] ENA,
    output wire [3:0] ENB,

    output reg [16:0] turn_pulse_cnt,
    input  wire       servo_done,
	 input end_cmd,
	 input mode,
	 output reg [2:0] move_latched,
	 output  reg post_fwd_active,
	 output reg [3:0] end_cmd_count ,
	 input wire one_right_turn

);

    // ================= PARAMETERS =================
//    parameter TURN_90          = 453;
    parameter TURN_90          = 453;
//        parameter TURN_90          = 380;

//    parameter TURN_180         = 880;
//    parameter TURN_180         = 800;
    parameter TURN_180         = 720;
 

//    parameter POST_FWD_PULSES  = 970;
   parameter POST_FWD_PULSES  = 800;


    parameter PWM_MAX     = 4'd15;
    parameter PWM_MIN     = 4'd0;
    parameter PWM_DELTA   = 4'd1;

    parameter TOLERANCE        = 16'd30;
    parameter OPEN_DIST        = 16'd285;
    parameter OPEN_DIST_TURN   = 16'd150;
    parameter SIDE_TARGET      = 16'd80;
    parameter SIDE_TOL         = 16'd10;

    // ================= ULTRASONIC =================
    wire left_wall_present  = (dist_L < OPEN_DIST_TURN);
    wire right_wall_present = (dist_R < OPEN_DIST_TURN);
    wire ultrasonic_stable  = (dist_L < OPEN_DIST) && (dist_R < OPEN_DIST);
    wire mid = (dist_F < 80);

    // ================= MOVE LATCH =================
	  wire ir_left_hit  = !ir1; 
	 
     wire ir_right_hit = !ir3;
 

wire emergency_allowed =
    (move_latched == 3'b001) || post_fwd_active;

wire emergency_left  = !ir3 && emergency_allowed;
 

//    reg [2:0] move_latched;

    // ================= UTURN TYPE LATCH ===========
    // 00 = none, 01 = simple, 10 = mpi
    reg [1:0] uturn_mode_latched;
	 
	 
 wire in_turn =
    (move_latched == 3'b010) ||
    (move_latched == 3'b011) ||
    (move_latched == 3'b100);

//    wire simple_uturn =  !ir1 && !ir3 && ir2;
    wire simple_uturn =  !ir1 && right_wall_present && ir2;
    wire mpi_uturn    =  !ir2 && !ir1 && right_wall_present;

    // ================= ENCODER EDGE ===============
    reg ENA1_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ENA1_d <= 1'b0;
        else
            ENA1_d <= ENA_1;
    end
    wire enc_pulse = ENA_1 & ~ENA1_d;

    // ================= COUNTERS ===================
    reg [16:0] post_fwd_cnt;

    // ================= FLAGS ======================
    reg turning_done;
//    reg post_fwd_active;

    // ================= MAIN SEQ ===================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            turn_pulse_cnt     <= 0;
            post_fwd_cnt       <= 0;
            turning_done       <= 0;
            post_fwd_active    <= 0;
            move_latched       <= 3'b000;
            uturn_mode_latched <= 2'b00;
				end_cmd_count      <= 0;
        end else begin
 


            // ---- LATCH MOVE & UTURN TYPE (IDLE ONLY)
            if (!post_fwd_active && !turning_done && turn_pulse_cnt == 0 && (!end_cmd)) begin
                 move_latched <= move;

                if (move == 3'b100) begin
                    if (simple_uturn)
                        uturn_mode_latched <= 2'b01;
                    else if (mpi_uturn)
                        uturn_mode_latched <= 2'b10;
                    else
                        uturn_mode_latched <= 2'b00;
                end else
                    uturn_mode_latched <= 2'b00;
            end
 
if (end_cmd) begin
    if (!one_right_turn) begin  // uturn from exit if last mpi is not traverse
        move_latched <= 3'b100;
        end_cmd_count <= end_cmd_count + 1;
        uturn_mode_latched <= 2'b01;
    end else begin // stop and exit point if mpi is traversed
        move_latched <= 3'b000;
    end
end
		 
		  

 
				 if ((move_latched == 3'b001)||(move_latched == 3'b000)) begin
					  turn_pulse_cnt <= 0;
				 end else if (!turning_done && !post_fwd_active) begin
					  if (enc_pulse) begin
							if (move_latched == 3'b010 ||
								 move_latched == 3'b011 ||
								(move_latched == 3'b100 &&
								 (uturn_mode_latched == 2'b01 ||
								 (uturn_mode_latched == 2'b10 && servo_done))))
								 turn_pulse_cnt <= turn_pulse_cnt + 1'b1;
					  end
				 end else begin
					  turn_pulse_cnt <= 0;
				 end


            // ---- TURN COMPLETE
            if (!turning_done && !post_fwd_active) begin
                if ((move_latched == 3'b010 && turn_pulse_cnt >= TURN_90-190) ||
                    (move_latched == 3'b011 && turn_pulse_cnt >= TURN_90+20) ||
                    (move_latched == 3'b100 && turn_pulse_cnt >= TURN_180-30))
                    turning_done <= 1'b1;
            end

            // ---- POST TURN FORWARD
            else if (turning_done && !post_fwd_active) begin
                post_fwd_active <= 1'b1;
                post_fwd_cnt    <= 0;
                turning_done    <= 1'b0;
            end

            else if (post_fwd_active) begin
                if (enc_pulse)
                    post_fwd_cnt <= post_fwd_cnt + 1'b1;

                if (post_fwd_cnt >= POST_FWD_PULSES)
                    post_fwd_active <= 1'b0;
            end
        end
    end

    // ================= STRAIGHT MODES =============
    wire straight_cruise_mode;
    wire straight_recovery_mode;

 
    assign straight_cruise_mode =
        ((move_latched == 3'b001) &&
         (!post_fwd_active) &&
         (turn_pulse_cnt == 0));

    assign straight_recovery_mode =
        post_fwd_active &&
        (left_wall_present || right_wall_present) &&
        (post_fwd_cnt > 0);

    // ================= PWM CORRECTION =============
    reg [3:0] pwm_L_corr, pwm_R_corr;


always @(*) begin
    pwm_L_corr = pwm_L;
    pwm_R_corr = pwm_R;

    // ===== Ultrasonic Calibration FIRST =====
    if (straight_cruise_mode || straight_recovery_mode) begin

        if (left_wall_present) begin
                if (dist_L < SIDE_TARGET - SIDE_TOL - 10) begin  //-22
                    if (pwm_L > PWM_MIN) pwm_L_corr = pwm_L - PWM_DELTA;
                    if (pwm_R < PWM_MAX) pwm_R_corr = pwm_R + PWM_DELTA;
                end
                else if (dist_L > SIDE_TARGET + SIDE_TOL) begin
                    if (pwm_R > PWM_MIN) pwm_R_corr = pwm_R - PWM_DELTA;
                    if (pwm_L < PWM_MAX) pwm_L_corr = pwm_L + PWM_DELTA;
                end
            end   

        // (your other ultrasonic cases here)
		    else  if (left_wall_present && right_wall_present) begin
            if (dist_R + TOLERANCE < dist_L) begin
                if (pwm_R_corr > PWM_MIN)
                    pwm_R_corr = pwm_R_corr - PWM_DELTA;
                if (pwm_L_corr < PWM_MAX)
                    pwm_L_corr = pwm_L_corr + PWM_DELTA;
            end
            else if (dist_L + TOLERANCE < dist_R) begin
                if (pwm_L_corr > PWM_MIN)
                    pwm_L_corr = pwm_L_corr - PWM_DELTA;
                if (pwm_R_corr < PWM_MAX)
                    pwm_R_corr = pwm_R_corr + PWM_DELTA;
            end
        end
 
        end
    

    // ===== Emergency reduction MUST BE LAST =====
    if (emergency_left ) begin
        if (pwm_L_corr > 2)
            pwm_L_corr = pwm_L_corr - 2;
        if (pwm_R_corr > 2)
            pwm_R_corr = pwm_R_corr - 2;
    end
end

    assign ENA = pwm_L_corr;
    assign ENB = pwm_R_corr;

    // ================= MOTOR OUTPUT ===============
    always @(*) begin
        IN1 = 0; IN2 = 0;
        IN3 = 0; IN4 = 0;
 
 

    // ===== EMERGENCY IR AVOID =====
	 
    if (emergency_left) begin
        // turn left
        IN1 = 1; IN2 = 0;
        IN3 = 0; IN4 = 0;
    end

 

    // ===== POST TURN FORWARD =====
    else if (post_fwd_active) begin
        IN1 = 1; IN2 = 0;
        IN3 = 1; IN4 = 0;
    end

		  
		  

//        else if (!turning_done) begin
      else if (!turning_done &&
             (move_latched == 3'b010 ||
                 move_latched == 3'b011 ||
                   move_latched == 3'b100)) begin
            case (move_latched)

                3'b010: begin

                    if (turn_pulse_cnt < TURN_90-190) begin
                        IN1 = 1; IN2 = 0;
                        IN3 = 0; IN4 = 1;
                    end
                end

                3'b011: begin
                    if (turn_pulse_cnt < TURN_90+20) begin
                        IN1 = 0; IN2 = 1;
                        IN3 = 1; IN4 = 0;
                    end
                end

                3'b100: begin
                    if (uturn_mode_latched == 2'b01 ||
                       (uturn_mode_latched == 2'b10 && servo_done)) begin
                        if (turn_pulse_cnt < TURN_180-30) begin
                            IN1 = 1; IN2 = 0;
                            IN3 = 0; IN4 = 1;
                        end
                    end
                end
            endcase
        end

        else if (move_latched == 3'b001) begin
            IN1 = 1; IN2 = 0;
            IN3 = 1; IN4 = 0;
        end
    end

endmodule

 