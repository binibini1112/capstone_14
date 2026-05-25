`timescale 1ns / 1ps

module pl_goal_compute_axi_tb;
    integer pass_fd;

    reg clk = 1'b0;
    reg rstn = 1'b0;

    reg [5:0] awaddr = 6'h0;
    reg [2:0] awprot = 3'h0;
    reg awvalid = 1'b0;
    wire awready;

    reg [31:0] wdata = 32'h0;
    reg [3:0] wstrb = 4'hf;
    reg wvalid = 1'b0;
    wire wready;

    wire [1:0] bresp;
    wire bvalid;
    reg bready = 1'b0;

    reg [5:0] araddr = 6'h0;
    reg [2:0] arprot = 3'h0;
    reg arvalid = 1'b0;
    wire arready;

    wire [31:0] rdata;
    wire [1:0] rresp;
    wire rvalid;
    reg rready = 1'b0;

    localparam [5:0] REG_CTRL        = 6'h00;
    localparam [5:0] REG_PAN_GOAL    = 6'h04;
    localparam [5:0] REG_TILT_GOAL   = 6'h08;
    localparam [5:0] REG_IDS         = 6'h0c;
    localparam [5:0] REG_STATUS      = 6'h10;
    localparam [5:0] REG_LAST_PAN    = 6'h14;
    localparam [5:0] REG_LAST_TILT   = 6'h18;
    localparam [5:0] REG_TRACK_BOX   = 6'h30;
    localparam [5:0] REG_TRACK_XY    = 6'h34;
    localparam [5:0] REG_TRACK_FRAME = 6'h38;
    localparam [5:0] REG_TRACK_CMD   = 6'h3c;

    pl_goal_compute_axi dut (
        .s_axi_aclk(clk),
        .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr),
        .s_axi_awprot(awprot),
        .s_axi_awvalid(awvalid),
        .s_axi_awready(awready),
        .s_axi_wdata(wdata),
        .s_axi_wstrb(wstrb),
        .s_axi_wvalid(wvalid),
        .s_axi_wready(wready),
        .s_axi_bresp(bresp),
        .s_axi_bvalid(bvalid),
        .s_axi_bready(bready),
        .s_axi_araddr(araddr),
        .s_axi_arprot(arprot),
        .s_axi_arvalid(arvalid),
        .s_axi_arready(arready),
        .s_axi_rdata(rdata),
        .s_axi_rresp(rresp),
        .s_axi_rvalid(rvalid),
        .s_axi_rready(rready)
    );

    always #5 clk = ~clk;

    task fail;
        input [1023:0] msg;
        begin
            $display("TB_FAIL: %0s", msg);
            $fatal(1);
        end
    endtask

    task axi_write;
        input [5:0] addr;
        input [31:0] data;
        integer guard;
        begin
            @(posedge clk);
            awaddr <= addr;
            awvalid <= 1'b1;
            wvalid <= 1'b0;
            bready <= 1'b1;

            guard = 0;
            while (!awready) begin
                @(posedge clk);
                guard = guard + 1;
                if (guard > 20) fail("AXI write address handshake timeout");
            end

            @(posedge clk);
            awvalid <= 1'b0;

            repeat (2) @(posedge clk);
            wdata <= data;
            wstrb <= 4'hf;
            wvalid <= 1'b1;

            guard = 0;
            while (!wready) begin
                @(posedge clk);
                guard = guard + 1;
                if (guard > 20) fail("AXI write data handshake timeout");
            end

            @(posedge clk);
            wvalid <= 1'b0;

            guard = 0;
            while (!bvalid) begin
                @(posedge clk);
                guard = guard + 1;
                if (guard > 20) fail("AXI write response timeout");
            end

            if (bresp !== 2'b00) fail("AXI write response not OKAY");
            @(posedge clk);
            bready <= 1'b0;
        end
    endtask

    task axi_read;
        input [5:0] addr;
        output [31:0] data;
        integer guard;
        begin
            @(posedge clk);
            araddr <= addr;
            arvalid <= 1'b1;
            rready <= 1'b1;

            guard = 0;
            while (!arready) begin
                @(posedge clk);
                guard = guard + 1;
                if (guard > 20) fail("AXI read address handshake timeout");
            end

            @(posedge clk);
            arvalid <= 1'b0;

            guard = 0;
            while (!rvalid) begin
                @(posedge clk);
                guard = guard + 1;
                if (guard > 20) fail("AXI read data timeout");
            end

            if (rresp !== 2'b00) fail("AXI read response not OKAY");
            data = rdata;
            @(posedge clk);
            rready <= 1'b0;
        end
    endtask

    task expect_read;
        input [5:0] addr;
        input [31:0] expected;
        reg [31:0] got;
        begin
            axi_read(addr, got);
            if (got !== expected) begin
                $display("TB_FAIL: addr=0x%02x expected=0x%08x got=0x%08x", addr, expected, got);
                $fatal(1);
            end
        end
    endtask

    initial begin
        repeat (6) @(posedge clk);
        rstn <= 1'b1;
        repeat (3) @(posedge clk);

        expect_read(REG_PAN_GOAL, 32'd2048);
        expect_read(REG_TILT_GOAL, 32'd2772);
        expect_read(REG_STATUS, 32'd2772);
        expect_read(REG_LAST_PAN, 32'd0);
        expect_read(REG_TRACK_BOX, {16'd60, 16'd80});

        axi_write(REG_PAN_GOAL, 32'd2100);
        axi_write(REG_TILT_GOAL, 32'd2800);
        expect_read(REG_PAN_GOAL, 32'd2100);
        expect_read(REG_TILT_GOAL, 32'd2800);

        axi_write(REG_CTRL, 32'd1);
        expect_read(REG_LAST_PAN, 32'd1);
        expect_read(REG_LAST_TILT, 32'h00008000);

        axi_write(REG_PAN_GOAL, 32'd2048);
        axi_write(REG_TILT_GOAL, 32'd2772);
        axi_write(REG_TRACK_XY, {16'd360, 16'd704});
        axi_write(REG_TRACK_FRAME, {16'd720, 16'd1280});
        axi_write(REG_TRACK_CMD, (32'd900 << 8) | 32'h3);
        expect_read(REG_LAST_PAN, 32'd2);
        expect_read(REG_PAN_GOAL, 32'd2056);
        expect_read(REG_TILT_GOAL, 32'd2772);
        expect_read(REG_CTRL, 32'd2056);
        expect_read(REG_STATUS, 32'd2772);

        axi_write(REG_PAN_GOAL, 32'd2048);
        axi_write(REG_TILT_GOAL, 32'd2772);
        axi_write(REG_TRACK_XY, {16'd400, 16'd0});
        axi_write(REG_TRACK_CMD, (32'd900 << 8) | 32'h3);
        expect_read(REG_LAST_PAN, 32'd3);
        expect_read(REG_PAN_GOAL, 32'd1976);
        expect_read(REG_TILT_GOAL, 32'd2767);
        expect_read(REG_CTRL, 32'd1976);
        expect_read(REG_STATUS, 32'd2767);

        axi_write(REG_TRACK_CMD, (32'd900 << 8) | 32'h3);
        expect_read(REG_LAST_PAN, 32'd4);
        expect_read(REG_PAN_GOAL, 32'd1904);
        expect_read(REG_TILT_GOAL, 32'd2762);
        expect_read(REG_CTRL, 32'd1904);
        expect_read(REG_STATUS, 32'd2762);

        axi_write(REG_TRACK_CMD, (32'd900 << 8) | 32'h3);
        expect_read(REG_LAST_PAN, 32'd5);
        expect_read(REG_PAN_GOAL, 32'd1832);
        expect_read(REG_TILT_GOAL, 32'd2757);
        expect_read(REG_CTRL, 32'd1832);
        expect_read(REG_STATUS, 32'd2757);

        axi_write(REG_CTRL, 32'h10000000 | 32'd2048);
        axi_write(REG_CTRL, 32'h20000000 | 32'd2772);
        axi_write(REG_CTRL, 32'h80000000 | (32'd60 << 16) | 32'd80);
        axi_write(REG_CTRL, 32'h30000000 | 32'd704);
        axi_write(REG_CTRL, 32'h40000000 | 32'd360);
        axi_write(REG_CTRL, 32'h50000000 | 32'd1280);
        axi_write(REG_CTRL, 32'h60000000 | 32'd720);
        axi_write(REG_CTRL, 32'h70000000 | (32'd900 << 8) | 32'h3);
        expect_read(REG_LAST_PAN, 32'd6);
        expect_read(REG_PAN_GOAL, 32'd2056);
        expect_read(REG_TILT_GOAL, 32'd2772);
        expect_read(REG_CTRL, 32'd2056);
        expect_read(REG_STATUS, 32'd2772);

        pass_fd = $fopen("pl_goal_compute_axi_tb.pass", "w");
        if (pass_fd == 0) fail("could not create pass sentinel");
        $fdisplay(pass_fd, "TB_PASS: pl_goal_compute_axi AXI register and track path");
        $fclose(pass_fd);
        $display("TB_PASS: pl_goal_compute_axi AXI register and track path");
        $finish;
    end
endmodule
