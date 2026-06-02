// processing_node.v — Frame-Interleaved Version
//
// Adds per-frame Lvc memory and C_hat registers.
// frame_id    : which frame is processed THIS cycle (from pipeline shift reg)
// rd_frame_id : which frame's C_hat to expose on the output port
//
// cn_update_block and vn_update_block are UNCHANGED.

module processing_node #(
    parameter NUM_LAYERS  = 3,
    parameter NUM_FRAMES  = 16,
    parameter LV_WIDTH    = 7,
    parameter [NUM_LAYERS-1:0] LAYER_ACTIVE = {NUM_LAYERS{1'b1}}
)(
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       node_en,
    input  wire                       phase_select,
    input  wire                       first_iter,
    input  wire [3:0]                 frame_id,
    input  wire [3:0]                 rd_frame_id,

    input  wire signed [4:0]          Qv,   // channel LLR for current frame

    input  wire [(NUM_LAYERS*10)-1:0] layer_msg_in,
    output reg  [(NUM_LAYERS*10)-1:0] layer_msg_out,
    output wire                       C_hat
);

    // =========================================================================
    // Unpack layer messages
    // =========================================================================
    wire        pc_in   [0:NUM_LAYERS-1];
    wire        sc_in   [0:NUM_LAYERS-1];
    wire [3:0]  min1_in [0:NUM_LAYERS-1];
    wire [3:0]  min2_in [0:NUM_LAYERS-1];

    genvar g;
    generate
        for (g = 0; g < NUM_LAYERS; g = g+1) begin : UNPACK
            assign pc_in  [g] = layer_msg_in[g*10 + 9];
            assign sc_in  [g] = layer_msg_in[g*10 + 8];
            assign min1_in[g] = layer_msg_in[g*10 + 7 -: 4];
            assign min2_in[g] = layer_msg_in[g*10 + 3 -: 4];
        end
    endgenerate

    // =========================================================================
    // Per-frame Lvc flat memory  (index = frame_id * NUM_LAYERS + layer)
    //   NUM_LAYERS=3: frame_id*3 = (frame_id<<1) + frame_id  (no multiplier)
    //   Max index = 15*3+2 = 47 → 6-bit address
    // =========================================================================
    reg        lvc_sign_mem [0:NUM_FRAMES*NUM_LAYERS-1];
    reg [3:0]  lvc_mag_mem  [0:NUM_FRAMES*NUM_LAYERS-1];

    // Per-frame hard decision
    reg C_hat_reg [0:NUM_FRAMES-1];
    assign C_hat = C_hat_reg[rd_frame_id];

    // lvc base address for current frame (frame_id * 3)
    wire [5:0] lvc_base = ({2'b0, frame_id} << 1) + {2'b0, frame_id};

    // =========================================================================
    // Qv sign/magnitude
    // =========================================================================
    wire        qv_sign = Qv[4];
    wire [4:0]  qv_abs  = Qv[4] ? (-Qv) : Qv;
    wire [3:0]  qv_mag  = qv_abs[4] ? 4'hF : qv_abs[3:0];

    // Lvc source: Qv on first_iter, stored value otherwise
    wire        lvc_sign_use [0:NUM_LAYERS-1];
    wire [3:0]  lvc_mag_use  [0:NUM_LAYERS-1];

    generate
        for (g = 0; g < NUM_LAYERS; g = g+1) begin : LVC_MUX
            assign lvc_sign_use[g] = first_iter ? qv_sign
                                                : lvc_sign_mem[lvc_base + g[1:0]];
            assign lvc_mag_use [g] = first_iter ? qv_mag
                                                : lvc_mag_mem [lvc_base + g[1:0]];
        end
    endgenerate

    // =========================================================================
    // CN update blocks
    // =========================================================================
    wire        sc_cn   [0:NUM_LAYERS-1];
    wire [3:0]  min1_cn [0:NUM_LAYERS-1];
    wire [3:0]  min2_cn [0:NUM_LAYERS-1];

    generate
        for (g = 0; g < NUM_LAYERS; g = g+1) begin : CN_BLOCKS
            wire phase_for_cn = phase_select | ~LAYER_ACTIVE[g];
            cn_update_block CN_U (
                .phase_select(phase_for_cn),
                .sign_in     (sc_in       [g]),
                .min1_in     (min1_in     [g]),
                .min2_in     (min2_in     [g]),
                .lvc_sign    (lvc_sign_use[g]),
                .lvc_mag     (lvc_mag_use [g]),
                .sign_out    (sc_cn       [g]),
                .min1_out    (min1_cn     [g]),
                .min2_out    (min2_cn     [g])
            );
        end
    endgenerate

    // =========================================================================
    // VN update blocks
    // =========================================================================
    wire signed [4:0] mcv_tc [0:NUM_LAYERS-1];

    generate
        for (g = 0; g < NUM_LAYERS; g = g+1) begin : VN_BLOCKS
            wire phase_for_vn = phase_select & LAYER_ACTIVE[g];
            vn_update_block VN_U (
                .phase_select(phase_for_vn),
                .sign_c      (sc_in      [g]),
                .min1_c      (min1_in    [g]),
                .min2_c      (min2_in    [g]),
                .lvc_sign    (lvc_sign_use[g]),
                .lvc_mag     (lvc_mag_use [g]),
                .mcv_tc      (mcv_tc     [g])
            );
        end
    endgenerate

    // =========================================================================
    // LLR accumulation
    // =========================================================================
    integer k;
    reg signed [LV_WIDTH-1:0] Lv;
    always @(*) begin : LV_ACCUM
        Lv = {{(LV_WIDTH-5){Qv[4]}}, Qv};
        for (k = 0; k < NUM_LAYERS; k = k+1)
            Lv = Lv + {{(LV_WIDTH-5){mcv_tc[k][4]}}, mcv_tc[k]};
    end

    wire C_hat_comb = Lv[LV_WIDTH-1];

    // =========================================================================
    // Sequential update — fires every decoding cycle (node_en=1)
    // Uses frame_id to select which frame's state to read/write
    // =========================================================================
    integer j;
    reg signed [LV_WIDTH-1:0] lvc_val;
    reg        [LV_WIDTH-1:0] abs_lvc;

    always @(posedge clk or posedge rst) begin : STATE_UPDATE
        if (rst) begin
            layer_msg_out <= {(NUM_LAYERS*10){1'b0}};
            for (j = 0; j < NUM_FRAMES; j = j+1)
                C_hat_reg[j] <= 1'b0;
            for (j = 0; j < NUM_FRAMES*NUM_LAYERS; j = j+1) begin
                lvc_sign_mem[j] <= 1'b0;
                lvc_mag_mem [j] <= 4'b0;
            end
        end else if (node_en) begin
            for (j = 0; j < NUM_LAYERS; j = j+1) begin
                if (~phase_select) begin
                    // --- CN phase ---
                    layer_msg_out[j*10 + 9]      <= pc_in[j];
                    layer_msg_out[j*10 + 8]       <= sc_cn[j];
                    layer_msg_out[j*10 + 7 -: 4] <= min1_cn[j];
                    layer_msg_out[j*10 + 3 -: 4] <= min2_cn[j];
                end else begin
                    // --- VN phase ---
                    if (j == 0)
                        C_hat_reg[frame_id] <= C_hat_comb;

                    layer_msg_out[j*10 + 9]      <= pc_in[j] ^ C_hat_comb;
                    layer_msg_out[j*10 + 8]       <= sc_in[j];
                    layer_msg_out[j*10 + 7 -: 4] <= min1_in[j];
                    layer_msg_out[j*10 + 3 -: 4] <= min2_in[j];

                    if (LAYER_ACTIVE[j]) begin
                        lvc_val = Lv - {{(LV_WIDTH-5){mcv_tc[j][4]}}, mcv_tc[j]};
                        abs_lvc = lvc_val[LV_WIDTH-1] ? -lvc_val : lvc_val;
                        lvc_sign_mem[lvc_base + j[1:0]] <= lvc_val[LV_WIDTH-1];
                        lvc_mag_mem [lvc_base + j[1:0]] <=
                            (|abs_lvc[LV_WIDTH-1:4]) ? 4'hF : abs_lvc[3:0];
                    end
                end
            end
        end
    end

endmodule
