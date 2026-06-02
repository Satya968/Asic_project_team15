// decoder_controller.v — Frame-Interleaved Pipeline Controller
//
// Single counter T_cnt drives everything combinationally:
//   frame_id     = T_cnt[3:0]   frame at pipeline stage-0 this cycle
//   phase_select = T_cnt[4]     0=CN pass, 1=VN pass
//   first_iter   = T_cnt < 32   true for the first 32-cycle period
//
// T_DONE: last processing cycle = frame_last + (iter_last)*32 + 16 + stage_last
//       = 15 + 9*32 + 16 + 15 = 334
// 'done' is a LEVEL signal (stays high until rst or re-start).

module decoder_controller #(
    parameter NUM_COLS   = 16,
    parameter NUM_FRAMES = 16,
    parameter MAX_ITER   = 10
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,

    output wire [3:0]  frame_id,
    output wire        phase_select,
    output wire        first_iter,
    output reg         decoding,
    output reg         done
);

    // 15 + 9*32 + 16 + 15 = 334
    localparam [9:0] T_DONE = (NUM_FRAMES-1)
                            + (MAX_ITER-1)*32
                            + NUM_COLS          // VN offset (16)
                            + (NUM_COLS-1);     // stage-15 latency

    reg [9:0] T_cnt;

    assign frame_id     = T_cnt[3:0];
    assign phase_select = T_cnt[4];
    assign first_iter   = (T_cnt < 10'd32);

    localparam S_IDLE=2'd0, S_RUN=2'd1, S_DONE=2'd2;
    reg [1:0] state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE; T_cnt <= 10'd0;
            decoding <= 1'b0; done <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    decoding <= 1'b0; done <= 1'b0; T_cnt <= 10'd0;
                    if (start) begin
                        state <= S_RUN; decoding <= 1'b1; T_cnt <= 10'd0;
                    end
                end
                S_RUN: begin
                    if (T_cnt == T_DONE) begin
                        state <= S_DONE; done <= 1'b1; decoding <= 1'b0;
                    end else begin
                        T_cnt <= T_cnt + 10'd1;
                    end
                end
                S_DONE: begin  // done stays high (level) until restart
                    if (start) begin
                        state <= S_RUN; decoding <= 1'b1;
                        done <= 1'b0; T_cnt <= 10'd0;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
