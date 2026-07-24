  
module t1b_ultrasonic(
    input  wire        clk_50M,
    input  wire        reset,     // active LOW
    input  wire        echo_rx,
    output reg         trig,
    output reg         op,
    output reg [15:0]  distance_out
);

    // -------------------------------------------------
    // FSM states
    // -------------------------------------------------
    localparam DELAY = 2'd0,
               TRIG  = 2'd1,
               ECHO  = 2'd2,
               DIST  = 2'd3;

    reg [1:0]  state;
    reg [19:0] counter;
    reg [31:0] echo_count;
    reg [31:0] distance_raw;
	 
	 reg [15:0] prev_distance;

    // -------------------------------------------------
    // Echo synchronizer (GOOD, kept)
    // -------------------------------------------------
    reg echo_d1, echo_d2;
    wire echo_sync;

    always @(posedge clk_50M) begin
        echo_d1 <= echo_rx;
        echo_d2 <= echo_d1;
    end

    assign echo_sync = echo_d2;

    // -------------------------------------------------
    // Main FSM
    // -------------------------------------------------
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            trig         <= 1'b0;
            op           <= 1'b0;
            state        <= DELAY;
            counter      <= 20'd0;
            echo_count   <= 32'd0;
            distance_raw <= 32'd0;
        end else begin

            // defaults
            trig <= 1'b0;

            case (state)

                // ---------------------------------
                // Startup / inter-measure delay
                // ---------------------------------
                DELAY: begin
                    if (counter < 20'd500000) begin   // ~10 ms quiet time
                        counter <= counter + 1'b1;
                    end else begin
                        counter    <= 20'd0;
                        echo_count <= 32'd0;
                        state      <= TRIG;
                    end
                end

                // ---------------------------------
                // Trigger pulse (10 us)
                // ---------------------------------
                TRIG: begin
                    trig <= 1'b1;
                    if (counter < 20'd500) begin
                        counter <= counter + 1'b1;
                    end else begin
                        trig       <= 1'b0;
                        counter    <= 20'd0;
                        echo_count <= 32'd0;
                        state      <= ECHO;
                    end
                end

                // ---------------------------------
                // Measure echo width WITH TIMEOUT
                // ---------------------------------
                ECHO: begin
                    if (echo_sync) begin
                        echo_count <= echo_count + 1'b1;
                        counter    <= 20'd0;   // reset timeout while echo high
                    end
                    else if (echo_count != 0) begin
                        // echo finished normally
                        distance_raw <= (echo_count * 16'd34) / 16'd10000;
                        op           <= ((echo_count * 16'd34) / 16'd10000) < 16'd70;
                        echo_count   <= 32'd0;
                        counter      <= 20'd0;
                        state        <= DIST;
                    end
                    else begin
                        // waiting for echo → timeout protection
                        if (counter < 20'd1500000) begin // ~30 ms timeout
                            counter <= counter + 1'b1;
                        end else begin
                            // echo failed → restart cycle safely
                            counter    <= 20'd0;
                            echo_count <= 32'd0;
                            state      <= DELAY;
                        end
                    end
                end

                // ---------------------------------
                // Hold result before next cycle
                // ---------------------------------
                DIST: begin
                    if (counter < 20'd300000) begin   // ~6 ms
                        counter <= counter + 1'b1;
                    end else begin
                        counter <= 20'd0;
                        state   <= DELAY;
                    end
                end

                default: begin
                    state <= DELAY;
                end

            endcase
        end
    end

    // -------------------------------------------------
    // Quantize output (safe, unchanged intent)
    // -------------------------------------------------
    always @(*) begin
        if (distance_raw < 16'd5)
            distance_out = 16'd0;
        else if (distance_raw > 16'd1000)
            distance_out = 16'd1000;
        else
            distance_out = (distance_raw / 16'd5) * 16'd5;
    end

//
//always @(posedge clk_50M or negedge reset) begin
//    if (!reset) begin
//        prev_distance <= 16'd0;
//        distance_out  <= 16'd0;
//    end else if (state == DIST) begin
//        
//        // Quantized value
//        reg [15:0] quantized;
//        if (distance_raw < 16'd5)
//            quantized = 16'd0;
//        else if (distance_raw > 16'd1000)
//            quantized = 16'd1000;
//        else
//            quantized = (distance_raw / 16'd5) * 16'd5;
//
//        // Reject large jump (>150mm change)
//        if ((quantized > prev_distance + 16'd150) ||
//            (quantized + 16'd150 < prev_distance)) begin
//            distance_out <= prev_distance;  // ignore spike
//        end else begin
//            distance_out <= quantized;
//            prev_distance <= quantized;
//        end
//    end
//end

endmodule

 