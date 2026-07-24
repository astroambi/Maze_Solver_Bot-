
module servo_180deg (
    input  wire clk,        // 50 MHz FPGA clock
    input  wire rst_n,      // Active-low reset
    input  wire ir1, ir2, ir3,
    output reg  pwm_out,
    input wire[15:0]  dist_F,dist_L,dist_R,
    output reg  servo_done,
	 input wire[16:0]turn_pulse_cnt,
	 output reg down_wait
);

       parameter OPEN_DIST_TURN      = 16'd150;
    wire left_wall_present  = (dist_L < OPEN_DIST_TURN);
    wire right_wall_present = (dist_R < OPEN_DIST_TURN);

    // --------------------------------------------------
    // Timing Parameters
    // --------------------------------------------------
    parameter PERIOD_TICKS  = 1000000;   // 20 ms @ 50 MHz
    parameter PULSE_0_DEG   = 50000;     // 1 ms (UP position)
    parameter PULSE_180_DEG = 100000;    // 2 ms (DOWN position)

    parameter FRAMES_TO_WAIT = 50;      // 5 seconds
    parameter CLEAR_FRAMES   = 25;       // 0.5 seconds
	 
	 reg[32:0] counter;
	 
	 
	 
	   

    // --------------------------------------------------
    // Registers & Counters
    // --------------------------------------------------
    reg [19:0] pwm_counter;
    reg [9:0]  frame_counter;
    reg [7:0]  clear_counter;
    reg [19:0] high_ticks;

    // --------------------------------------------------
    // State Machine
    // --------------------------------------------------
    localparam S_WAIT_CLEAR = 3'b000;
    localparam S_IDLE       = 3'b001;
    localparam S_DOWN_WAIT  = 3'b010;
    localparam S_UP_DONE    = 3'b011;

    reg [2:0] state;

    // --------------------------------------------------
    // Sensor Conditions
    // --------------------------------------------------
    wire all_sensors_low = (!ir1 && !ir2 && right_wall_present  );
//    wire strt_face_raw   = ((dist_F>250));
    wire end_of_frame    = (pwm_counter == PERIOD_TICKS - 1);

    // --------------------------------------------------
    // 🔑 LATCHED EXIT CONDITION (FIX)
    // --------------------------------------------------
//    reg strt_face_latched;
//
//    always @(posedge clk or negedge rst_n) begin
//        if (!rst_n)
//            strt_face_latched <= 1'b0;
//        else if (state == S_UP_DONE && strt_face_raw)
//            strt_face_latched <= 1'b1;
//        else if (state == S_WAIT_CLEAR)
//            strt_face_latched <= 1'b0;
//    end

    // --------------------------------------------------
    // PWM Generator (Free Running)
    // --------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 0;
            pwm_out     <= 0;
        end else begin
            if (pwm_counter == PERIOD_TICKS - 1)
                pwm_counter <= 0;
            else
                pwm_counter <= pwm_counter + 1;

            pwm_out <= (pwm_counter < high_ticks);
        end
    end

    // --------------------------------------------------
    // Main FSM (Frame-Synchronized)
    // --------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_WAIT_CLEAR;
            frame_counter <= 0;
            clear_counter <= 0;
            high_ticks    <= PULSE_0_DEG;
            servo_done    <= 1'b0;
				counter       <= 0;
				down_wait     <=0;
        end else if (end_of_frame) begin

            case (state)

                // ---------------- WAIT_CLEAR ----------------
                S_WAIT_CLEAR: begin
                    high_ticks <= PULSE_0_DEG;
                    servo_done <= 1'b0;
						    down_wait <=0;

                    if (!all_sensors_low) begin
                        if (clear_counter >= CLEAR_FRAMES - 1) begin
                            state         <= S_IDLE;
                            clear_counter <= 0;
                        end else
                            clear_counter <= clear_counter + 1;
                    end else
                        clear_counter <= 0;
                end

                // ---------------- IDLE ----------------
                S_IDLE: begin
                    servo_done    <= 1'b0;
                    high_ticks    <= PULSE_0_DEG;
                    frame_counter <= 0;

                    if (all_sensors_low) begin
                        state      <= S_DOWN_WAIT;
                        high_ticks <= PULSE_180_DEG;
                    end
                end

                // ---------------- DOWN_WAIT ----------------
                S_DOWN_WAIT: begin
                    high_ticks <= PULSE_180_DEG;
						  down_wait <=1;
						  

                    if (frame_counter >= FRAMES_TO_WAIT - 1) begin
                        state         <= S_UP_DONE;
                        high_ticks    <= PULSE_0_DEG;
                        frame_counter <= 0;
                    end else
                        frame_counter <= frame_counter + 1;
                end

                // ---------------- UP_DONE ----------------
               S_UP_DONE: begin
                 high_ticks <= PULSE_0_DEG;

                if (counter  >= 100) begin        // 50 frames = 1 second
                         servo_done    <= 1'b0;
                         state         <= S_WAIT_CLEAR;
                         clear_counter <= 0;
                         counter       <= 0;
               end
               else begin
                    servo_done <= 1'b1;
                    state      <= S_UP_DONE;
                    counter    <= counter + 1;
                end
                end

                default: state <= S_WAIT_CLEAR;

            endcase
        end
    end

endmodule
