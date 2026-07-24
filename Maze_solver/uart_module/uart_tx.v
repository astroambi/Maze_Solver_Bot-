// MazeSolver Bot: Task 2B - UART Transmitter
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.

This file is used to generate UART Tx data packet to transmit the messages based on the input data.

Recommended Quartus Version : 20.1
The submitted project file must be 20.1 compatible as the evaluation will be done on Quartus Prime Lite 20.1.

Warning: The error due to compatibility will not be entertained.
-------------------
*/

/*
Module UART Transmitter

Input:  clk_3125 - 3125 KHz clock
        parity_type - even(0)/odd(1) parity type
        tx_start - signal to start the communication.
        data    - 8-bit data line to transmit

Output: tx      - UART Transmission Line
        tx_done - message transmitted flag


        Baudrate : 115200 bps
*/

// module declaration

module uart_tx(
    input clk_3125,
    input parity_type, tx_start,
    input [7:0] data,
    output reg tx, tx_done
);

//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE//////////////////

// FSM States
localparam IDLE   = 3'd0,
           START  = 3'd1,
           DATA   = 3'd2,
           PARITY = 3'd3,
           STOP   = 3'd4;

// Registers
reg [4:0] bit_cnt = 0;   // Enough to count 0–26
reg [2:0] bit_idx = 0;   // Data bit counter
reg [2:0] state   = IDLE;

// Initialization
initial begin
    tx = 1'b1;
    tx_done = 1'b0;
end

// UART FSM
always @(posedge clk_3125) begin
    tx_done <= 0;  // default
    case (state)
        IDLE: begin
            tx <= 1;
             
            bit_idx <= 0;
				bit_cnt <= 1;
            if (tx_start) begin
                tx <= 0;              // start bit
                state <= START;
            end
        end

        START, DATA, PARITY, STOP: begin
            bit_cnt <= bit_cnt + 1;
            if (bit_cnt == 26) begin
                bit_cnt <= 0;
                case (state)
                    START:  begin state <= DATA;   bit_idx <= 0;  end
                    DATA:   if (bit_idx == 7) state <= PARITY;
                            else bit_idx <= bit_idx + 1;
                    PARITY: state <= STOP;
                    STOP:   begin state <= IDLE; tx_done <= 1; end
                endcase
            end

            // TX output assignment per state
            case (state)
                START:  tx <= 0;
                DATA:   tx <= data[ bit_idx]; // MSB-first
                PARITY: tx <= (^data ^ parity_type);
                STOP:   tx <= 1;
            endcase
        end
    endcase
end

endmodule

