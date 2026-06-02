// column_slice.v — Frame-Interleaved Version
//
// qv_mem expanded to NUM_FRAMES*Q entries (flat 1D).
// frame_id   : selects which frame's Qv to feed into nodes this cycle.
// rd_frame_id: passed through to each processing_node for C_hat read.
// qv_wframe  : selects which frame slot to write during LLR loading.

module column_slice #(
    parameter Q           = 42,
    parameter NUM_LAYERS  = 3,
    parameter NUM_FRAMES  = 16,
    parameter LV_WIDTH    = 7,
    parameter MSG_W       = 10,
    parameter [NUM_LAYERS-1:0] LAYER_ACTIVE = {NUM_LAYERS{1'b1}}
)(
    input  wire                                clk,
    input  wire                                rst,
    input  wire                                node_en,
    input  wire                                phase_select,
    input  wire                                first_iter,
    input  wire [3:0]                          frame_id,
    input  wire [3:0]                          rd_frame_id,

    // LLR write port
    input  wire                                qv_we,
    input  wire [3:0]                          qv_wframe,  // frame 0..15
    input  wire [5:0]                          qv_waddr,   // node  0..41
    input  wire signed [4:0]                   qv_wdata,

    input  wire [(Q*NUM_LAYERS*MSG_W)-1:0]     layer_msg_in,
    output wire [(Q*NUM_LAYERS*MSG_W)-1:0]     layer_msg_out,
    output wire [Q-1:0]                        C_hat
);

    // =========================================================================
    // Per-frame Qv memory  — flat: index = frame * Q + node
    //   Q=42, NUM_FRAMES=16  →  672 entries, 10-bit address
    // =========================================================================
    reg signed [4:0] qv_mem [0:NUM_FRAMES*Q-1];

    // Write address: frame*42 + node
    //   42 = 32+8+2 → (wf<<5)+(wf<<3)+(wf<<1)  — no multiplier
    wire [9:0] wf = {6'd0, qv_wframe};
    wire [9:0] qv_waddr_full = (wf<<5) + (wf<<3) + (wf<<1) + {4'd0, qv_waddr};

    integer qi;
    always @(posedge clk or posedge rst) begin : QV_MEM
        if (rst) begin
            for (qi = 0; qi < NUM_FRAMES*Q; qi = qi+1)
                qv_mem[qi] <= 5'sd0;
        end else if (qv_we) begin
            qv_mem[qv_waddr_full] <= qv_wdata;
        end
    end

    // Read base for current frame_id:  frame_id * 42
    wire [9:0] fid = {6'd0, frame_id};
    wire [9:0] qv_rd_base = (fid<<5) + (fid<<3) + (fid<<1);

    // =========================================================================
    // Processing nodes — each node v reads qv_mem[qv_rd_base + v]
    // =========================================================================
    genvar v;
    generate
        for (v = 0; v < Q; v = v+1) begin : GEN_NODES
            processing_node #(
                .NUM_LAYERS  (NUM_LAYERS),
                .NUM_FRAMES  (NUM_FRAMES),
                .LV_WIDTH    (LV_WIDTH),
                .LAYER_ACTIVE(LAYER_ACTIVE)
            ) PN (
                .clk         (clk),
                .rst         (rst),
                .node_en     (node_en),
                .phase_select(phase_select),
                .first_iter  (first_iter),
                .frame_id    (frame_id),
                .rd_frame_id (rd_frame_id),
                .Qv          (qv_mem[qv_rd_base + v[5:0]]),
                .layer_msg_in (layer_msg_in [(v*NUM_LAYERS*MSG_W) +: (NUM_LAYERS*MSG_W)]),
                .layer_msg_out(layer_msg_out[(v*NUM_LAYERS*MSG_W) +: (NUM_LAYERS*MSG_W)]),
                .C_hat        (C_hat[v])
            );
        end
    endgenerate

endmodule
