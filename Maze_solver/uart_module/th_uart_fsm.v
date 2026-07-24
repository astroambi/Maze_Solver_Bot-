 


module th_uart_fsm (
    input        clk,        // 50MHz
    input        reset,      // Active Low

    // From DHT & Sensors
    input        data_valid,
    input        end_cmd,      
    input  [7:0] T_integral, T_decimal,
    input  [7:0] RH_integral, RH_decimal,
    input [11:0] d_out_ch0,
    input        ir1, ir2, ir3,
    input  [7:0] mpi_id,     
    input wire [16:0] dist_R,	 

    // To UART
    output reg [7:0] tx_data,
    output reg       tx_start,
    input            tx_done,
    input  wire      down_wait,
	 
	  input wire one_right_turn,
	 
	 output reg  end_cnf
);

    parameter CLK_FREQ  = 50_000_000;
    parameter DELAY_MS  = 200;   
    parameter PRE_DELAY_MS = 50; 
    
    localparam DELAY_MAX = (CLK_FREQ/1000) * DELAY_MS;
    localparam PRE_DELAY_MAX = (CLK_FREQ/1000) * PRE_DELAY_MS;
	 
	    parameter OPEN_DIST_TURN      = 16'd150;
//    wire left_wall_present  = (dist_L < OPEN_DIST_TURN);
    wire right_wall_present = (dist_R < OPEN_DIST_TURN);

    // FSM States
    localparam [5:0]
        IDLE              = 6'd0,
        PRE_WAIT          = 6'd21, 
        END_SETTLE        = 6'd22, // NEW: State to validate end_cmd noise
        SEND_MPIM_HDR     = 6'd1,  SEND_MPIM_ID      = 6'd2,  SEND_MPIM_END      = 6'd3,
        SEND_MM_HDR       = 6'd4,  SEND_MM_ID        = 6'd5,  SEND_MM_SEP        = 6'd6, 
        SEND_MM_VAL       = 6'd7,  SEND_MM_END       = 6'd8,
        SEND_TH_HDR       = 6'd9,  SEND_TH_ID        = 6'd10, SEND_TH_SEP1       = 6'd11, 
        SEND_TH_TEMP      = 6'd12, SEND_TH_TEMP_DOT  = 6'd13, SEND_TH_TEMP_DEC   = 6'd14,
        SEND_TH_SEP2      = 6'd15,
        SEND_TH_HUM       = 6'd16, SEND_TH_HUM_DOT   = 6'd17, SEND_TH_HUM_DEC    = 6'd18,
        SEND_TH_END       = 6'd19,
        SEND_MAZE_END     = 6'd20,
        WAIT_TX           = 6'd30,
        DONE_CYCLE        = 6'd31;

    reg [5:0]  state, next_state;
    reg [7:0]  char_ptr;
    reg [28:0] delay_cnt;
    reg [28:0] pre_cnt;      
    reg [28:0] end_settle_cnt; // Counter for end_cmd debouncing
    reg        delay_active;

    // Latched data
    reg [7:0] T_reg, T_dec_reg;
    reg [7:0] H_reg, H_dec_reg;
    reg [7:0] id_latch;
    reg       is_moist;

    // Edge detection
    reg end_cmd_d, mpi_detected_d;
	 
	 // FOR SENDING MESSAGE UNTILL ALL THE MPI IS GET DEADEND GET TRAVERSED
    wire mpi_detected = (!ir1 && !ir2 && right_wall_present && down_wait && !one_right_turn);
    wire end_rise = (end_cmd && !end_cmd_d);

    wire moist_condition = (d_out_ch0 < 12'd1200); 

    always @(posedge clk or negedge reset ) begin
        if (!reset ) begin
            state            <= IDLE;
            tx_start         <= 1'b0;
            char_ptr         <= 8'd0;
            delay_active     <= 1'b0;
            delay_cnt        <= 29'd0;
            pre_cnt          <= 29'd0;
            end_settle_cnt   <= 29'd0;
            end_cmd_d        <= 1'b0;
            mpi_detected_d   <= 1'b0;
				end_cnf          <= 0;

        end else begin
            tx_start       <= 1'b0; 
            end_cmd_d      <= end_cmd;
            mpi_detected_d <= mpi_detected;

            if (delay_active) begin
                if (delay_cnt < DELAY_MAX)
                    delay_cnt <= delay_cnt + 1'b1;
                else begin
                    delay_active <= 1'b0;
                    delay_cnt    <= 29'd0;
                end
            end

            case (state)
                IDLE: begin
                    if (!delay_active) begin
                        if (end_rise) begin
                            end_settle_cnt <= 29'd0;
                            state          <= END_SETTLE;
                        end 
                        else if (mpi_detected) begin
                            // Latch data immediately
                       
                            pre_cnt    <= 29'd0;
                            state      <= PRE_WAIT;
                        end
                    end
                end

                // NEW: End Command Settling/Debounce State
                END_SETTLE: begin
                    if (!end_cmd) begin
                        // If signal drops, it was noise
                        state <= IDLE;
                    end else if (end_settle_cnt < PRE_DELAY_MAX) begin
                        end_settle_cnt <= end_settle_cnt + 1'b1;
                    end else begin
                        // Valid signal confirmed
								
								end_cnf <= 1;
                        char_ptr   <= 0;
                        tx_data    <= "E";
                        tx_start   <= 1'b1;
                        next_state <= SEND_MAZE_END;
                        state      <= WAIT_TX;
                    end
                end

                PRE_WAIT: begin
                    if (pre_cnt < PRE_DELAY_MAX) begin
                        pre_cnt <= pre_cnt + 1'b1;
                    end else begin
								T_reg     <= T_integral;
                            T_dec_reg <= T_decimal;
                            H_reg     <= RH_integral;
                            H_dec_reg <= RH_decimal;
                            id_latch  <= mpi_id;
                            is_moist  <= moist_condition;
                        char_ptr   <= 0;
                        tx_data    <= "M";
                        tx_start   <= 1'b1;
                        next_state <= SEND_MPIM_HDR;
                        state      <= WAIT_TX;
                    end
                end

                WAIT_TX: if (tx_done) state <= next_state;

                SEND_MAZE_END: begin
                    char_ptr <= char_ptr + 1;
                    case(char_ptr)
                        0: tx_data <= "N";
                        1: tx_data <= "D";
                        2: tx_data <= "-";
                        3: tx_data <= "#";
                        4: tx_data <= 8'h0A;
                    endcase
						  end_cnf <= 0;
                    tx_start <= 1'b1;
                    if (char_ptr == 4) next_state <= DONE_CYCLE;
                    else               next_state <= SEND_MAZE_END;
                    state <= WAIT_TX;
                end

                // ... [Rest of the MPI and TH states remain identical to your working code] ...

                SEND_MPIM_HDR: begin
                    char_ptr <= char_ptr + 1;
                    case(char_ptr)
                        0: tx_data <= "P"; 1: tx_data <= "I"; 2: tx_data <= "M"; 3: tx_data <= "-";
                    endcase
                    tx_start <= 1'b1;
                    if (char_ptr == 3) next_state <= SEND_MPIM_ID;
                    else               next_state <= SEND_MPIM_HDR;
                    state <= WAIT_TX;
                end

                SEND_MPIM_ID: begin
                    tx_data    <= id_latch + 8'd48;
                    tx_start   <= 1'b1;
                    char_ptr   <= 0;
                    next_state <= SEND_MPIM_END;
                    state      <= WAIT_TX;
                end

                SEND_MPIM_END: begin
                    char_ptr <= char_ptr + 1;
                    case(char_ptr)
                        0: tx_data <= "-"; 1: tx_data <= "#"; 2: tx_data <= 8'h0A;
                    endcase
                    tx_start <= 1'b1;
                    if (char_ptr == 2) begin
                        char_ptr   <= 0;
                        next_state <= SEND_MM_HDR;
                    end else next_state <= SEND_MPIM_END;
                    state <= WAIT_TX;
                end

                SEND_MM_HDR: begin
                    char_ptr <= char_ptr + 1;
                    case(char_ptr)
                        0: tx_data <= "M"; 1: tx_data <= "M"; 2: tx_data <= "-";
                    endcase
                    tx_start <= 1'b1;
                    if (char_ptr == 2) begin
                        char_ptr   <= 0;
                        next_state <= SEND_MM_ID;
                    end else next_state <= SEND_MM_HDR;
                    state <= WAIT_TX;
                end

                SEND_MM_ID: begin
                    tx_data    <= id_latch + 8'd48;
                    tx_start   <= 1'b1;
                    next_state <= SEND_MM_SEP;
                    state      <= WAIT_TX;
                end

                SEND_MM_SEP: begin
                    tx_data    <= "-";
                    tx_start   <= 1'b1;
                    next_state <= SEND_MM_VAL;
                    state      <= WAIT_TX;
                end

                SEND_MM_VAL: begin
                    tx_data    <= is_moist ? "M" : "D";
                    tx_start   <= 1'b1;
                    next_state <= SEND_MM_END;
                    char_ptr   <= 0;
                    state      <= WAIT_TX;
                end

                SEND_MM_END: begin
                    char_ptr <= char_ptr + 1;
                    case(char_ptr)
                        0: tx_data <= "-"; 1: tx_data <= "#"; 2: tx_data <= 8'h0A;
                    endcase
                    tx_start <= 1'b1;
                    if (char_ptr == 2) begin
                        char_ptr   <= 0;
                        next_state <= SEND_TH_HDR;
                    end else next_state <= SEND_MM_END;
                    state <= WAIT_TX;
                end

                SEND_TH_HDR: begin
                    char_ptr <= char_ptr + 1;
                    case(char_ptr)
                        0: tx_data <= "T"; 1: tx_data <= "H"; 2: tx_data <= "-";
                    endcase
                    tx_start <= 1'b1;
                    if (char_ptr == 2) begin
                        char_ptr   <= 0;
                        next_state <= SEND_TH_ID;
                    end else next_state <= SEND_TH_HDR;
                    state <= WAIT_TX;
                end

                SEND_TH_ID: begin
                    tx_data    <= id_latch + 8'd48;
                    tx_start   <= 1'b1;
                    next_state <= SEND_TH_SEP1;
                    state      <= WAIT_TX;
                end

                SEND_TH_SEP1: begin
                    tx_data    <= "-";
                    tx_start   <= 1'b1;
                    next_state <= SEND_TH_TEMP;
                    char_ptr   <= 0;
                    state      <= WAIT_TX;
                end

                SEND_TH_TEMP: begin
                    char_ptr <= char_ptr + 1;
                    if (char_ptr == 0) tx_data <= (T_reg / 10) + 8'd48;
                    else               tx_data <= (T_reg % 10) + 8'd48;
                    tx_start <= 1'b1;
                    if (char_ptr == 1) begin
                        char_ptr   <= 0;
                        next_state <= SEND_TH_TEMP_DOT;
                    end else next_state <= SEND_TH_TEMP;
                    state <= WAIT_TX;
                end

                SEND_TH_TEMP_DOT: begin
                    tx_data    <= ".";
                    tx_start   <= 1'b1;
                    next_state <= SEND_TH_TEMP_DEC;
                    state      <= WAIT_TX;
                end

                SEND_TH_TEMP_DEC: begin
                    tx_data    <= T_dec_reg + 8'd48;
                    tx_start   <= 1'b1;
                    next_state <= SEND_TH_SEP2;
                    state      <= WAIT_TX;
                end

                SEND_TH_SEP2: begin
                    tx_data    <= "-";
                    tx_start   <= 1'b1;
                    next_state <= SEND_TH_HUM;
                    char_ptr   <= 0;
                    state      <= WAIT_TX;
                end

                SEND_TH_HUM: begin
                    char_ptr <= char_ptr + 1;
                    if (char_ptr == 0) tx_data <= (H_reg / 10) + 8'd48;
                    else               tx_data <= (H_reg % 10) + 8'd48;
                    tx_start <= 1'b1;
                    if (char_ptr == 1) begin
                        char_ptr   <= 0;
                        next_state <= SEND_TH_HUM_DOT;
                    end else next_state <= SEND_TH_HUM;
                    state <= WAIT_TX;
                end

                SEND_TH_HUM_DOT: begin
                    tx_data    <= ".";
                    tx_start   <= 1'b1;
                    next_state <= SEND_TH_HUM_DEC;
                    state      <= WAIT_TX;
                end

                SEND_TH_HUM_DEC: begin
                    tx_data    <= H_dec_reg + 8'd48;
                    tx_start   <= 1'b1;
                    next_state <= SEND_TH_END;
                    state      <= WAIT_TX;
                end

                SEND_TH_END: begin
                    char_ptr <= char_ptr + 1;
                    case(char_ptr)
                        0: tx_data <= "-";
                        1: tx_data <= "#";
                        2: tx_data <= 8'h0A;
                    endcase
                    tx_start <= 1'b1;
                    if (char_ptr == 2) begin
                        char_ptr   <= 0;
                        next_state <= DONE_CYCLE;
                    end else next_state <= SEND_TH_END;
                    state <= WAIT_TX;
                end

                DONE_CYCLE: begin
                    delay_active <= 1'b1;
                    delay_cnt    <= 29'd0;
                    state        <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule