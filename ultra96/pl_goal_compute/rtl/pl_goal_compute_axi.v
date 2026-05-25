`timescale 1ns / 1ps

module pl_goal_compute_axi #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6
) (
    input  wire                              s_axi_aclk,
    input  wire                              s_axi_aresetn,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  wire [2:0]                        s_axi_awprot,
    input  wire                              s_axi_awvalid,
    output reg                               s_axi_awready,

    input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                              s_axi_wvalid,
    output reg                               s_axi_wready,

    output reg [1:0]                         s_axi_bresp,
    output reg                               s_axi_bvalid,
    input  wire                              s_axi_bready,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  wire [2:0]                        s_axi_arprot,
    input  wire                              s_axi_arvalid,
    output reg                               s_axi_arready,

    output reg [C_S_AXI_DATA_WIDTH-1:0]      s_axi_rdata,
    output reg [1:0]                         s_axi_rresp,
    output reg                               s_axi_rvalid,
    input  wire                              s_axi_rready
);

    localparam [5:0] ADDR_CTRL        = 6'h00;
    localparam [5:0] ADDR_PAN_GOAL    = 6'h04;
    localparam [5:0] ADDR_TILT_GOAL   = 6'h08;
    localparam [5:0] ADDR_IDS         = 6'h0c;
    localparam [5:0] ADDR_STATUS      = 6'h10;
    localparam [5:0] ADDR_LAST_PAN    = 6'h14;
    localparam [5:0] ADDR_LAST_TILT   = 6'h18;
    localparam [5:0] ADDR_TRACK_BOX   = 6'h30;
    localparam [5:0] ADDR_TRACK_XY    = 6'h34;
    localparam [5:0] ADDR_TRACK_FRAME = 6'h38;
    localparam [5:0] ADDR_TRACK_CMD   = 6'h3c;

    localparam [31:0] GOAL_MIN = 32'd0;
    localparam [31:0] GOAL_MAX = 32'd4095;
    // PS-best tracking profile mirrored in RTL:
    // correction = trunc_toward_zero(pixel_error * NUM / DEN), then clamp.
    // This matches ps_track_step() in pl_udp_usb_dxl_bridge.c so PL-drive can
    // replace PS-direct without changing the external Jetson bbox protocol.
    localparam signed [31:0] TRACK_DEADBAND_X = 32'sd14;
    localparam signed [31:0] TRACK_DEADBAND_Y = 32'sd14;
    localparam signed [31:0] TRACK_TICK_PER_PIXEL_NUM = 32'sd1;
    localparam signed [31:0] TRACK_TICK_PER_PIXEL_DEN = 32'sd8;
    localparam signed [31:0] TRACK_MAX_CORRECTION_X = 32'sd72;
    localparam signed [31:0] TRACK_MAX_CORRECTION_Y = 32'sd72;
    localparam signed [31:0] TRACK_MAX_CORRECTION_CLOSE_X = 32'sd32;
    localparam signed [31:0] TRACK_MAX_CORRECTION_CLOSE_Y = 32'sd32;
    localparam [15:0] TRACK_CLOSE_BOX_W = 16'd220;
    localparam [15:0] TRACK_CLOSE_BOX_H = 16'd160;
    localparam [3:0] CMD_LEGACY  = 4'h0;
    localparam [3:0] CMD_SET_PAN = 4'h1;
    localparam [3:0] CMD_SET_TILT = 4'h2;
    localparam [3:0] CMD_SET_CX = 4'h3;
    localparam [3:0] CMD_SET_CY = 4'h4;
    localparam [3:0] CMD_SET_FW = 4'h5;
    localparam [3:0] CMD_SET_FH = 4'h6;
    localparam [3:0] CMD_TRACK = 4'h7;
    localparam [3:0] CMD_SET_BOX = 4'h8;

    reg [C_S_AXI_ADDR_WIDTH-1:0] write_addr_q;
    reg [C_S_AXI_DATA_WIDTH-1:0] write_data_q;
    reg [(C_S_AXI_DATA_WIDTH/8)-1:0] write_strb_q;
    reg write_addr_valid;
    reg write_data_valid;
    reg write_pending;

    reg [31:0] pan_goal;
    reg [31:0] tilt_goal;
    reg [7:0] pan_id;
    reg [7:0] tilt_id;
    reg done_toggle;
    reg track_toggle;
    reg audio_toggle;
    reg [31:0] compute_count;
    reg [31:0] last_pan;
    reg [31:0] last_tilt;

    reg [15:0] track_cx;
    reg [15:0] track_cy;
    reg [15:0] track_bw;
    reg [15:0] track_bh;
    reg [15:0] track_fw;
    reg [15:0] track_fh;
    reg [15:0] track_conf;
    wire aw_capture = !write_addr_valid && !write_pending && !s_axi_bvalid && s_axi_awvalid;
    wire w_capture = !write_data_valid && !write_pending && !s_axi_bvalid && s_axi_wvalid;
    wire write_complete = !write_pending && !s_axi_bvalid &&
                          (write_addr_valid || aw_capture) &&
                          (write_data_valid || w_capture);

    wire signed [31:0] track_error_x;
    wire signed [31:0] track_error_y;
    wire signed [31:0] track_pan_step;
    wire signed [31:0] track_tilt_step;
    wire signed [31:0] track_max_correction_x;
    wire signed [31:0] track_max_correction_y;

    assign track_error_x = $signed({16'h0, track_cx}) - $signed({16'h0, (track_fw >> 1)});
    assign track_error_y = $signed({16'h0, track_cy}) - $signed({16'h0, (track_fh >> 1)});
    assign track_max_correction_x =
        (track_bw >= TRACK_CLOSE_BOX_W || track_bh >= TRACK_CLOSE_BOX_H) ?
        TRACK_MAX_CORRECTION_CLOSE_X : TRACK_MAX_CORRECTION_X;
    assign track_max_correction_y =
        (track_bw >= TRACK_CLOSE_BOX_W || track_bh >= TRACK_CLOSE_BOX_H) ?
        TRACK_MAX_CORRECTION_CLOSE_Y : TRACK_MAX_CORRECTION_Y;
    assign track_pan_step = ps_direct_step(track_error_x, TRACK_DEADBAND_X, track_max_correction_x);
    assign track_tilt_step = -ps_direct_step(track_error_y, TRACK_DEADBAND_Y, track_max_correction_y);

    function [31:0] apply_wstrb;
        input [31:0] old_data;
        input [31:0] new_data;
        input [3:0] strb;
        integer i;
        begin
            apply_wstrb = old_data;
            for (i = 0; i < 4; i = i + 1) begin
                if (strb[i]) begin
                    apply_wstrb[i*8 +: 8] = new_data[i*8 +: 8];
                end
            end
        end
    endfunction

    function [31:0] clamp_goal;
        input signed [63:0] value;
        begin
            if (value < $signed({32'h0, GOAL_MIN})) begin
                clamp_goal = GOAL_MIN;
            end else if (value > $signed({32'h0, GOAL_MAX})) begin
                clamp_goal = GOAL_MAX;
            end else begin
                clamp_goal = value[31:0];
            end
        end
    endfunction

    function signed [31:0] abs32;
        input signed [31:0] value;
        begin
            abs32 = value < 0 ? -value : value;
        end
    endfunction

    function signed [31:0] clamp_signed_mag;
        input signed [31:0] value;
        input signed [31:0] max_mag;
        begin
            if (value > max_mag) begin
                clamp_signed_mag = max_mag;
            end else if (value < -max_mag) begin
                clamp_signed_mag = -max_mag;
            end else begin
                clamp_signed_mag = value;
            end
        end
    endfunction

    function signed [31:0] div64_trunc_zero;
        input signed [63:0] value;
        input signed [31:0] denom;
        reg signed [63:0] quotient;
        begin
            if (value < 64'sd0) begin
                quotient = -((-value) / denom);
            end else begin
                quotient = value / denom;
            end
            div64_trunc_zero = quotient[31:0];
        end
    endfunction

    function signed [31:0] track_scaled_error;
        input signed [31:0] err;
        reg signed [63:0] product;
        begin
            product = $signed(err) * $signed(TRACK_TICK_PER_PIXEL_NUM);
            track_scaled_error = div64_trunc_zero(product, TRACK_TICK_PER_PIXEL_DEN);
        end
    endfunction

    function signed [31:0] ps_direct_step;
        input signed [31:0] err;
        input signed [31:0] deadband;
        input signed [31:0] max_correction;
        begin
            if (abs32(err) <= deadband) begin
                ps_direct_step = 32'sd0;
            end else begin
                ps_direct_step = clamp_signed_mag(track_scaled_error(err), max_correction);
            end
        end
    endfunction

    function [31:0] read_reg;
        input [C_S_AXI_ADDR_WIDTH-1:0] addr;
        begin
            case (addr[5:0] & 6'h3c)
                ADDR_CTRL:        read_reg = pan_goal;
                ADDR_PAN_GOAL:    read_reg = pan_goal;
                ADDR_TILT_GOAL:   read_reg = tilt_goal;
                ADDR_IDS:         read_reg = {16'h0, tilt_id, pan_id};
                ADDR_STATUS:      read_reg = tilt_goal;
                ADDR_LAST_PAN:    read_reg = compute_count;
                ADDR_LAST_TILT:   read_reg = {16'h0, done_toggle, 6'h0, audio_toggle, track_toggle, 7'h0};
                ADDR_TRACK_BOX:   read_reg = {track_bh, track_bw};
                ADDR_TRACK_XY:    read_reg = {track_cy, track_cx};
                ADDR_TRACK_FRAME: read_reg = {track_fh, track_fw};
                ADDR_TRACK_CMD:   read_reg = {7'h0, track_conf, done_toggle, 6'h0, audio_toggle, track_toggle};
                default:          read_reg = 32'h0;
            endcase
        end
    endfunction

    initial begin
        pan_goal = 32'd2048;
        tilt_goal = 32'd2772;
        pan_id = 8'd1;
        tilt_id = 8'd2;
        done_toggle = 1'b0;
        track_toggle = 1'b0;
        audio_toggle = 1'b0;
        compute_count = 32'h0;
        last_pan = 32'd2048;
        last_tilt = 32'd2772;
        track_cx = 16'd640;
        track_cy = 16'd360;
        track_bw = 16'd80;
        track_bh = 16'd60;
        track_fw = 16'd1280;
        track_fh = 16'd720;
        track_conf = 16'h0;
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bresp <= 2'b00;
            s_axi_bvalid <= 1'b0;
            write_addr_q <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            write_data_q <= {C_S_AXI_DATA_WIDTH{1'b0}};
            write_strb_q <= {(C_S_AXI_DATA_WIDTH/8){1'b0}};
            write_addr_valid <= 1'b0;
            write_data_valid <= 1'b0;
            write_pending <= 1'b0;
        end else begin
            if (aw_capture) begin
                s_axi_awready <= 1'b1;
                write_addr_q <= s_axi_awaddr;
                write_addr_valid <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
            end

            if (w_capture) begin
                s_axi_wready <= 1'b1;
                write_data_q <= s_axi_wdata;
                write_strb_q <= s_axi_wstrb;
                write_data_valid <= 1'b1;
            end else begin
                s_axi_wready <= 1'b0;
            end

            if (write_complete) begin
                write_pending <= 1'b1;
                write_addr_valid <= 1'b0;
                write_data_valid <= 1'b0;
            end else if (write_pending && !s_axi_bvalid) begin
                write_pending <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rdata <= 32'h0;
            s_axi_rresp <= 2'b00;
            s_axi_rvalid <= 1'b0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rdata <= read_reg(s_axi_araddr);
                s_axi_rresp <= 2'b00;
                s_axi_rvalid <= 1'b1;
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            pan_goal <= 32'd2048;
            tilt_goal <= 32'd2772;
            pan_id <= 8'd1;
            tilt_id <= 8'd2;
            done_toggle <= 1'b0;
            track_toggle <= 1'b0;
            audio_toggle <= 1'b0;
            compute_count <= 32'h0;
            last_pan <= 32'd2048;
            last_tilt <= 32'd2772;
            track_cx <= 16'd640;
            track_cy <= 16'd360;
            track_bw <= 16'd80;
            track_bh <= 16'd60;
            track_fw <= 16'd1280;
            track_fh <= 16'd720;
            track_conf <= 16'h0;
        end else if (write_pending) begin
            case (write_addr_q[5:0] & 6'h3c)
                ADDR_CTRL: begin
                    case (write_data_q[31:28])
                        CMD_LEGACY: begin
                            if (write_data_q[0]) begin
                                last_pan <= pan_goal;
                                last_tilt <= tilt_goal;
                                compute_count <= compute_count + 1'b1;
                                done_toggle <= ~done_toggle;
                            end
                        end
                        CMD_SET_PAN: begin
                            pan_goal <= {20'h0, write_data_q[11:0]};
                        end
                        CMD_SET_TILT: begin
                            tilt_goal <= {20'h0, write_data_q[11:0]};
                        end
                        CMD_SET_CX: begin
                            track_cx <= write_data_q[15:0];
                        end
                        CMD_SET_CY: begin
                            track_cy <= write_data_q[15:0];
                        end
                        CMD_SET_FW: begin
                            track_fw <= write_data_q[15:0];
                        end
                        CMD_SET_FH: begin
                            track_fh <= write_data_q[15:0];
                        end
                        CMD_SET_BOX: begin
                            track_bw <= {4'h0, write_data_q[11:0]};
                            track_bh <= {4'h0, write_data_q[27:16]};
                        end
                        CMD_TRACK: begin
                            track_conf <= write_data_q[23:8];
                            if (write_data_q[0] && write_data_q[1] && track_fw != 16'h0 && track_fh != 16'h0) begin
                                pan_goal <= clamp_goal($signed({32'h0, pan_goal}) + $signed(track_pan_step));
                                tilt_goal <= clamp_goal($signed({32'h0, tilt_goal}) + $signed(track_tilt_step));
                                last_pan <= clamp_goal($signed({32'h0, pan_goal}) + $signed(track_pan_step));
                                last_tilt <= clamp_goal($signed({32'h0, tilt_goal}) + $signed(track_tilt_step));
                                compute_count <= compute_count + 1'b1;
                                done_toggle <= ~done_toggle;
                                track_toggle <= ~track_toggle;
                            end
                        end
                        default: begin
                        end
                    endcase
                end
                ADDR_PAN_GOAL: begin
                    pan_goal <= write_data_q;
                end
                ADDR_TILT_GOAL: begin
                    tilt_goal <= write_data_q;
                end
                ADDR_IDS: begin
                    pan_id <= write_data_q[7:0];
                    tilt_id <= write_data_q[15:8];
                end
                ADDR_TRACK_BOX: begin
                    track_bw <= write_data_q[15:0];
                    track_bh <= write_data_q[31:16];
                end
                ADDR_TRACK_XY: begin
                    track_cx <= write_data_q[15:0];
                    track_cy <= write_data_q[31:16];
                end
                ADDR_TRACK_FRAME: begin
                    track_fw <= write_data_q[15:0];
                    track_fh <= write_data_q[31:16];
                end
                ADDR_TRACK_CMD: begin
                    track_conf <= write_data_q[23:8];
                    if (write_data_q[0] && write_data_q[1] && track_fw != 16'h0 && track_fh != 16'h0) begin
                        pan_goal <= clamp_goal($signed({32'h0, pan_goal}) + $signed(track_pan_step));
                        tilt_goal <= clamp_goal($signed({32'h0, tilt_goal}) + $signed(track_tilt_step));
                        last_pan <= clamp_goal($signed({32'h0, pan_goal}) + $signed(track_pan_step));
                        last_tilt <= clamp_goal($signed({32'h0, tilt_goal}) + $signed(track_tilt_step));
                        compute_count <= compute_count + 1'b1;
                        done_toggle <= ~done_toggle;
                        track_toggle <= ~track_toggle;
                    end
                end
                default: begin
                end
            endcase
        end
    end

    wire unused_inputs = &{1'b0, s_axi_awprot, s_axi_arprot};

endmodule
