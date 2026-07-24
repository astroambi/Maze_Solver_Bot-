 

module start_command_detector (
    input  wire       clk,
    input  wire       rst_n,        // Active-low reset
    input  wire [7:0] rx_msg,       // ASCII byte from UART
    input  wire       rx_complete,  // 1-cycle pulse when byte valid
    output reg        start_signal  // 1-cycle pulse on START-3-#
);

    // FSM states
    localparam IDLE  = 4'd0,
               S1    = 4'd1,  // S
               S2    = 4'd2,  // T
               S3    = 4'd3,  // A
               S4    = 4'd4,  // R
               S5    = 4'd5,  // T
               S6    = 4'd6,  // -
               S7    = 4'd7,  // 3
               S8    = 4'd8,  // -
               DONE  = 4'd9;  // #

    reg [3:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            start_signal <= 1'b0;
        end else begin
//            start_signal <= 1'b0;  // default (pulse only)

            if (rx_complete) begin
                case (state)

                    IDLE:  state <= (rx_msg == 8'h53) ? S1 : IDLE; // S
                    S1:    state <= (rx_msg == 8'h54) ? S2 : IDLE; // T
                    S2:    state <= (rx_msg == 8'h41) ? S3 : IDLE; // A
                    S3:    state <= (rx_msg == 8'h52) ? S4 : IDLE; // R
                    S4:    state <= (rx_msg == 8'h54) ? S5 : IDLE; // T
                    S5:    state <= (rx_msg == 8'h2D) ? S6 : IDLE; // -
                    S6:    state <= (rx_msg == 8'h35) ? S7 : IDLE; // 3
                    S7:    state <= (rx_msg == 8'h2D) ? S8 : IDLE; // -
                    S8: begin
                        if (rx_msg == 8'h23) begin                 // #
                            start_signal <= 1'b1;                  // 🎯 COMMAND MATCH
                            state <= IDLE;
                        end else begin
                            state <= IDLE;
                        end
                    end

                    default: state <= IDLE;
                endcase
            end
        end
    end

endmodule
