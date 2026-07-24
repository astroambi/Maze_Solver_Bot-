 

module mpi_mapper (
    input        clk,
    input        reset,      // active LOW
    input  [2:0] move,
    output reg [3:0] mpi_id,
    output reg mode
);

    reg [3:0]  deadend_count;
    reg [19:0] filter_cnt;    
    reg        move_sync_0, move_sync_1; 
    reg        move_is_high;  
    reg        move_reg_d;    

    // 10ms at 50MHz. Adjust this if your U-turns are extremely fast.
    localparam STABLE_TIME = 20'd500_000; 

    // ---------- 1. Synchronize & Hysteresis Debounce ----------
    always @(posedge clk) begin
        if (!reset) begin
            move_sync_0  <= 1'b0;
            move_sync_1  <= 1'b0;
            filter_cnt   <= 20'd0;
            move_is_high <= 1'b0;
        end else begin
            // 2-stage synchronizer
            move_sync_0 <= (move == 3'b100);
            move_sync_1 <= move_sync_0;

            if (move_sync_1 == move_is_high) begin
                // Input matches current state, keep counter at 0
                filter_cnt <= 20'd0;
            end else begin
                // Input is different from current stable state, start counting
                if (filter_cnt < STABLE_TIME) begin
                    filter_cnt <= filter_cnt + 1'b1;
                end else begin
                    // Signal has been different and stable long enough: Flip the state
                    move_is_high <= move_sync_1;
                    filter_cnt   <= 20'd0;
                end
            end
        end
    end

    // ---------- 2. Edge Detection + Counter ----------
    always @(posedge clk) begin
        if (!reset) begin
            deadend_count <= 4'd0;
            move_reg_d    <= 1'b0;
            mode          <= 1'b0;
        end else begin
            move_reg_d <= move_is_high;

            // Trigger ONLY on the rising edge of the stable signal
            if (move_is_high && !move_reg_d) begin
                if (deadend_count >= 4'd9)
                    deadend_count <= 4'd1;
                else
                    deadend_count <= deadend_count + 1'b1;
            end

            // Permanent mode latch
            if (deadend_count == 4'd9)
                mode <= 1'b1;
        end
    end

    // ---------- 3. Mapper LUT ----------
    always @(*) begin
        case (deadend_count)
            4'd1: mpi_id = 4'd1;
            4'd2: mpi_id = 4'd2;
            4'd3: mpi_id = 4'd3;
            4'd4: mpi_id = 4'd4;
            4'd5: mpi_id = 4'd6;
            4'd6: mpi_id = 4'd8;
            4'd7: mpi_id = 4'd9;
            4'd8: mpi_id = 4'd7;
            4'd9: mpi_id = 4'd5;
            default: mpi_id = 4'd1;
        endcase
    end

endmodule