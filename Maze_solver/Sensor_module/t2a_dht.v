 
module t2a_dht (
    input wire clk_50M,      // 50 MHz Clock
    input wire reset,        // Active Low Reset
    inout wire sensor,       // Bidirectional Sensor Pin
    output reg [7:0] T_integral,
    output reg [7:0] RH_integral,
    output reg [7:0] T_decimal,
    output reg [7:0] RH_decimal,
    output reg [7:0] Checksum,
    output reg data_valid,   // Stays High until next read starts
    output reg error         // Stays High until next read starts
);

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam FREQ_MHZ = 50;

    // Timing Constants
    localparam C_2S_DELAY      = 2000 * 1000 * FREQ_MHZ; // 2 seconds
    localparam C_18MS_START    = 18 * 1000 * FREQ_MHZ;   // 18 ms
    localparam C_20US_RELEASE  = 20 * FREQ_MHZ;          // 20 us
    localparam C_TIMEOUT       = 200 * FREQ_MHZ;         // 200 us safety limit
    localparam C_BIT_THRESHOLD = 50 * FREQ_MHZ;          // 50 us (Threshold)

    // FSM States
    localparam IDLE            = 4'd0,
               START_LOW       = 4'd1,
               START_HIGH      = 4'd2,
               WAIT_SENSOR_LOW = 4'd3,
               WAIT_RESP_LOW   = 4'd4,
               WAIT_RESP_HIGH  = 4'd5,
               READ_BIT_LOW    = 4'd6,
               READ_BIT_HIGH   = 4'd7,
               DONE            = 4'd8;

    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------
    reg [3:0]  state;
    reg [27:0] delay_timer; 
    reg [20:0] counter;      
    reg [5:0]  bit_index;
    reg [39:0] data_reg;

    // Tri-state Control
    reg sensor_out;
    reg sensor_oe;
    assign sensor = sensor_oe ? sensor_out : 1'bz;

    // Synchronizer
    reg sync1, sync2;
    wire sensor_in;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            sync1 <= 1;
            sync2 <= 1;
        end else begin
            sync1 <= sensor;
            sync2 <= sync1;
        end
    end
    assign sensor_in = sync2;

    // -------------------------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------------------------
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            state       <= IDLE;
            T_integral  <= 0; RH_integral <= 0;
            T_decimal   <= 0; RH_decimal  <= 0;
            Checksum    <= 0;
            data_valid  <= 0;
            error       <= 0;
            sensor_oe   <= 0;
            sensor_out  <= 1;
            counter     <= 0;
            delay_timer <= 0;
            bit_index   <= 0;
            data_reg    <= 0;
        end else begin
            
            // -----------------------------------------------------------------
            // PRIORITY 1: TIMEOUT GUARD
            // This structure guarantees Fault Dominance.
            // -----------------------------------------------------------------
            // If we are in a listening state AND the counter exceeds safe limits:
            if ((state > START_HIGH && state < DONE) && (counter > C_TIMEOUT)) begin
                state       <= IDLE;
                error       <= 1;        // Latch error
                data_valid  <= 0;        // Invalidate data
                sensor_oe   <= 0;        // Force bus release
                delay_timer <= 0;        // Restart wait period
            end 
            // -----------------------------------------------------------------
            // PRIORITY 2: NORMAL FSM EXECUTION
            // Only executes if NO timeout exists.
            // -----------------------------------------------------------------
            else begin
                case (state)
                    IDLE: begin
                        sensor_oe <= 0;
                        if (delay_timer < C_2S_DELAY) begin
                            delay_timer <= delay_timer + 1;
                        end else begin
                            delay_timer <= 0;
                            counter     <= 0;
                            bit_index   <= 0;
                            data_reg    <= 0;
                            data_valid  <= 0; 
                            error       <= 0; 
                            state       <= START_LOW;
                        end
                    end

                    START_LOW: begin
                        sensor_oe  <= 1;
                        sensor_out <= 0;
                        if (counter >= C_18MS_START) begin
                            counter <= 0;
                            state   <= START_HIGH;
                        end else begin
                            counter <= counter + 1;
                        end
                    end

                    START_HIGH: begin
                        sensor_out <= 1;
                        if (counter >= C_20US_RELEASE) begin
                            counter   <= 0;
                            sensor_oe <= 0; 
                            state     <= WAIT_SENSOR_LOW;
                        end else begin
                            counter <= counter + 1;
                        end
                    end

                    WAIT_SENSOR_LOW: begin
                        if (sensor_in == 0) begin
                            counter <= 0;
                            state   <= WAIT_RESP_LOW;
                        end else begin
                            counter <= counter + 1;
                        end
                    end

                    WAIT_RESP_LOW: begin
                        if (sensor_in == 1) begin
                            counter <= 0;
                            state   <= WAIT_RESP_HIGH;
                        end else begin
                            counter <= counter + 1;
                        end
                    end

                    WAIT_RESP_HIGH: begin
                        if (sensor_in == 0) begin
                            counter <= 0;
                            state   <= READ_BIT_LOW;
                        end else begin
                            counter <= counter + 1;
                        end
                    end

                    READ_BIT_LOW: begin
                        if (sensor_in == 1) begin
                            counter <= 0;
                            state   <= READ_BIT_HIGH;
                        end else begin
                            counter <= counter + 1;
                        end
                    end

                    READ_BIT_HIGH: begin
                        if (sensor_in == 0) begin
                            if (counter > C_BIT_THRESHOLD)
                                data_reg[39 - bit_index] <= 1;
                            else
                                data_reg[39 - bit_index] <= 0;
                            
                            counter   <= 0;
                            bit_index <= bit_index + 1;

                            if (bit_index == 39)
                                state <= DONE;
                            else
                                state <= READ_BIT_LOW;
                        end else begin
                            counter <= counter + 1;
                        end
                    end

                    DONE: begin
                        RH_integral <= data_reg[39:32];
                        RH_decimal  <= data_reg[31:24];
                        T_integral  <= data_reg[23:16];
                        T_decimal   <= data_reg[15:8];
                        Checksum    <= data_reg[7:0];

                        if (data_reg[39:32] + data_reg[31:24] + data_reg[23:16] + data_reg[15:8] == data_reg[7:0]) begin
                            data_valid <= 1;
                            error      <= 0;
                        end else begin
                            data_valid <= 0;
                            error      <= 1;
                        end
                        state <= IDLE;
                    end

                    default: state <= IDLE;
                endcase
            end
        end
    end

endmodule