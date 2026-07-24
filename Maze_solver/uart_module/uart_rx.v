
module uart_rx(
    input  wire       clk_3125,
    input  wire       rx,
    output reg [7:0]  rx_msg,
    output reg        rx_parity,
    output reg        rx_complete
);

    // 3.125 MHz / 115200 ≈ 27 clocks per bit
    localparam integer CLKS_PER_BIT = 27;
    localparam integer HALF_BIT     = CLKS_PER_BIT / 2;

    // FSM states
    localparam IDLE  = 2'd0,
               START = 2'd1,
               DATA  = 2'd2,
               STOP  = 2'd3;

    reg [1:0] state = IDLE;
    reg [7:0] data_reg;
    reg [3:0] bit_index;
    reg [7:0] clk_count;
    reg rx_sync;

    always @(posedge clk_3125) begin
        // synchronize RX
        rx_sync <= rx;

        rx_complete <= 1'b0;
        rx_parity   <= 1'b0;   // not used (Bluetooth has no parity)

        case (state)

        // ---------------- IDLE ----------------
        IDLE: begin
            clk_count <= 0;
            bit_index <= 0;
            if (!rx_sync) begin              // start bit detected
                state <= START;
                clk_count <= HALF_BIT;       // wait half bit
            end
        end

        // ---------------- START ----------------
        START: begin
            if (clk_count == 0) begin
                state <= DATA;
                clk_count <= CLKS_PER_BIT - 1;
            end else
                clk_count <= clk_count - 1;
        end

        // ---------------- DATA ----------------
        DATA: begin
            if (clk_count == 0) begin
                data_reg[bit_index] <= rx_sync;   // LSB first
                clk_count <= CLKS_PER_BIT - 1;

                if (bit_index == 7)
                    state <= STOP;
                else
                    bit_index <= bit_index + 1;
            end else
                clk_count <= clk_count - 1;
        end

        // ---------------- STOP ----------------
        STOP: begin
            if (clk_count == 0) begin
                rx_msg <= data_reg;
					 
                rx_complete <= 1'b1;          // one-cycle pulse
                state <= IDLE;
            end else
                clk_count <= clk_count - 1;
        end

        endcase
    end

endmodule

 