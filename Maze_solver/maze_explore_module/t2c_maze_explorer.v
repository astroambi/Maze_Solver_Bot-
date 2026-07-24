 module t2c_maze_explorer (
    input clk,
    input rst_n,
	 input start_signal,

    // Wall sensors (1 = wall, 0 = free)
    input left, mid, right,

	 input end_cmd,   // Exploration completion signal

    // Ultrasonic distances
	 input [15:0] dist_F,dist_L,dist_R,

	 input ir2,       // Front IR safety validation

    // Movement output to motor driver
    output reg [2:0] move,

	 output reg [3:0] deadend_count,  // Count number of detected dead-ends

	 input wire [2:0] move_latched,   // Feedback from motor driver

	 input wire mode,                 // 0 = left-hand rule, 1 = right-hand rule
	 output reg one_right_turn,       // Flag to detect first RHS turn in return phase

	 input wire post_fwd_active,      // Indicates post-turn forward correction
	 input wire[3:0] end_cmd_count
);

    /*
    | cmd | move  | meaning   |
    |-----|-------|-----------|
    | 000 | STOP              |
    | 001 | FORWARD           |
    | 010 | LEFT              |
    | 011 | RIGHT             |
    | 100 | U_TURN            |
    */

    // ================= FSM STATES =================
    parameter IDLE        = 3'b000;
    parameter DECIDE_MOVE = 3'b001;
    parameter MOVE_FWD    = 3'b010;
    parameter TURN_LEFT   = 3'b011;
    parameter TURN_RIGHT  = 3'b100;
    parameter TURN_BACK   = 3'b101;

    // ================= DIRECTION ENCODING =================
    parameter N = 2'b00;
    parameter E = 2'b01;
    parameter S = 2'b10;
    parameter W = 2'b11;

    // ================= STATE REGISTERS =================
    reg [2:0] state, next_state;
    reg [2:0] move_next;
    reg [1:0] dir, next_dir;

	 reg[7:0] visited_count;  // Count visited maze cells

    // 9x9 visited grid memory
    reg visited [0:8][0:8];

    integer i, j;

    // Maze position coordinates
    reg [7:0] x, y;
    reg [7:0] next_x, next_y;

	 reg [32:0] counter;

 
// ================= SEQUENTIAL FSM UPDATE =================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset to initial conditions
        state <= IDLE;
        move  <= 3'b000;
        dir   <= N;
        x     <= 4;   // Starting X position
        y     <= 8;   // Starting Y position
        one_right_turn <= 1'b0;
    end

    // If start signal removed, force idle
    else if (!start_signal) begin
        state <= IDLE;
        move  <= 3'b000;
        one_right_turn <= 1'b0;   // Clear return-phase detection
    end

    // Final stop condition: both end_cmd and one_right_turn active
    else if (end_cmd && one_right_turn) begin
        state <= IDLE;
        move  <= 3'b000;
    end

    else begin
        // Normal FSM progression
        state <= next_state;
        move  <= move_next;
        dir   <= next_dir;
        x     <= next_x;
        y     <= next_y;

        // Detect first RHS turn during return  to LHStraversal
        if (mode == 1'b1 &&
            post_fwd_active &&
            move_latched == 3'b011 &&
            !one_right_turn)
        begin
            one_right_turn <= 1'b1;
        end
    end
end


// ================= COMBINATIONAL NEXT-STATE LOGIC =================
always @(*) begin
    next_state = state;
    move_next = move;
    next_dir = dir;
    next_x = x;
    next_y = y;

    case (state)

        // Idle immediately transitions to decision state
        IDLE: begin
            next_state = DECIDE_MOVE;
        end


        // ================= DECISION STATE =================
        DECIDE_MOVE: begin

            // Safety check for unknown or floating inputs
            if ((left === 1'bz) || (left === 1'bx) ||
                (mid  === 1'bz) || (mid  === 1'bx) ||
                (right=== 1'bz) || (right=== 1'bx)) begin
                    move_next = 3'b000;
                    next_state = DECIDE_MOVE;
            end 
            else begin

                // ===== LEFT-HAND RULE (Exploration Phase) =====
                if (mode == 0 || one_right_turn) begin

                    if (!left) begin
                        move_next = 3'b010; // LEFT
                        next_state = TURN_LEFT;
                        next_dir = (dir - 1) & 2'b11;
                    end 

                    else if (!mid && ir2 ) begin
                        move_next = 3'b001; // FORWARD
                        next_state = MOVE_FWD;
                        next_dir = dir;
                    end 

                    else if (!right) begin
                        move_next = 3'b011; // RIGHT
                        next_state = TURN_RIGHT;
                        next_dir = (dir + 1) & 2'b11;
                    end 

                    // Dead-end detection
                    else if ((left)&&(right)&& (dist_F<120)) begin
                        move_next = 3'b100; // U-turn
                        next_state = TURN_BACK;
                        next_dir = (dir + 2) & 2'b11;
                        deadend_count = deadend_count +1;
                    end
                end 

                // ===== RIGHT-HAND RULE (Return Phase) =====
                else begin

                    if (!right) begin
                        move_next = 3'b011; // RIGHT
                        next_state = TURN_RIGHT;
                        next_dir = (dir + 1) & 2'b11;
                    end 

                    else if (!mid && ir2) begin
                        move_next = 3'b001; // FORWARD
                        next_state = MOVE_FWD;
                        next_dir = dir;
                    end 

                    else if (!left) begin
                        move_next = 3'b010; // LEFT
                        next_state = TURN_LEFT;
                        next_dir = (dir - 1) & 2'b11;
                    end 

                    // Dead-end detection
                    else if ((left)&&(right)&& (dist_F<120)) begin
                        move_next = 3'b100; // U-turn
                        next_state = TURN_BACK;
                        next_dir = (dir + 2) & 2'b11;
                        deadend_count = deadend_count +1;
                    end
                end
            end
        end


        // ================= MOVEMENT STATES =================
        MOVE_FWD, TURN_LEFT, TURN_RIGHT, TURN_BACK: begin

            // After issuing movement command, return to decision state
            next_state = DECIDE_MOVE;

            // Update position based on direction
            case (next_dir)
                N: next_y = y - 1;
                E: next_x = x + 1;
                S: next_y = y + 1;
                W: next_x = x - 1;
            endcase

            // Mark cell as visited
            if (visited[next_x][next_y] == 1'b0) begin
                visited[next_x][next_y] = 1'b1;
                visited_count = visited_count + 1;
            end 

            // Optional: trigger mode change after exploring enough cells
            if (visited_count > 77) begin
                // mode switch logic could be placed here
            end
        end


        // Default fallback
        default: begin
            move_next = 3'b000;
            next_state = IDLE;
        end
    endcase
end

endmodule