#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#define REG_CTRL        0x00u
#define REG_PAN_GOAL    0x04u
#define REG_TILT_GOAL   0x08u
#define REG_IDS         0x0cu
#define REG_STATUS      0x10u
#define REG_LAST_PAN    0x14u
#define REG_LAST_TILT   0x18u
#define REG_TRACK_BOX   0x30u
#define REG_TRACK_XY    0x34u
#define REG_TRACK_FRAME 0x38u
#define REG_TRACK_CMD   0x3cu

#define TRACK_CMD_VALID 0x00000001u
#define TRACK_CMD_TRACK 0x00000002u

#define MAP_SIZE 0x1000u
#define PAN_CENTER 2048u
#define TILT_CENTER 2772u
#define GOAL_MIN 0u
#define GOAL_MAX 4095u
#define PAN_TICKS_PER_REV 4096
#define ADDR_TORQUE_ENABLE 64u
#define ADDR_PROFILE_ACCEL 108u
#define ADDR_PROFILE_VELOCITY 112u
#define ADDR_GOAL_POSITION 116u
#define ADDR_PRESENT_POSITION 132u
#define TRACK_DEADBAND_X 14
#define TRACK_DEADBAND_Y 14
#define TRACK_TICK_PER_PIXEL_NUM 1
#define TRACK_TICK_PER_PIXEL_DEN 8
#define TRACK_MAX_CORRECTION 72
#define TRACK_MAX_CORRECTION_CLOSE 32
#define TRACK_CLOSE_BOX_W 220u
#define TRACK_CLOSE_BOX_H 160u
#define TRACK_PAN_LIMIT_FROM_FRONT 900
#define TRACK_TILT_LIMIT_FROM_FRONT 240
#define LASER_TICKS_PER_RAD 652
#define LASER_TICKS_PER_DEG_NUM 1024
#define LASER_TICKS_PER_DEG_DEN 90
#define LASER_DEFAULT_DISTANCE_MM 1000u
#define LASER_DEFAULT_VERTICAL_FOV_DEG 43u
#define LASER_CAL_TABLE_SIZE 12u
#define PL_CMD_SET_PAN  0x10000000u
#define PL_CMD_SET_TILT 0x20000000u
#define PL_CMD_SET_CX   0x30000000u
#define PL_CMD_SET_CY   0x40000000u
#define PL_CMD_SET_FW   0x50000000u
#define PL_CMD_SET_FH   0x60000000u
#define PL_CMD_TRACK    0x70000000u
#define PL_CMD_SET_BOX  0x80000000u

static volatile sig_atomic_t g_running = 1;
static int64_t g_track_pan_limit = TRACK_PAN_LIMIT_FROM_FRONT;
static int64_t g_track_tilt_limit = TRACK_TILT_LIMIT_FROM_FRONT;

static void on_signal(int signo)
{
    (void)signo;
    g_running = 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s --base 0xA0000000 [--port 5016] [--serial /dev/ttyUSB0] "
        "[--baud 57600] [--pan-id 1] [--tilt-id 2] [--dry-run] [--no-pl] "
        "[--skip-pl-init] [--skip-dxl-init] [--lazy-pl-open] "
        "[--track-direct-ps] [--track-pl-shadow] "
        "[--track-pan-limit 900] [--track-tilt-limit 240] "
        "[--profile-accel 60] [--profile-velocity 180] "
        "[--laser-id 3] [--laser-center 2048] [--laser-offset-mm 38] "
        "[--laser-distance-mm 1000] [--laser-vertical-fov-deg 43] [--laser-sign -1] "
        "[--center-file /home/xilinx/ultra_yubin_v1/front_center.env]\n",
        argv0);
}

static uint32_t parse_u32(const char *s)
{
    return (uint32_t)strtoul(s, NULL, 0);
}

static uint32_t clamp_goal_i64(int64_t value)
{
    if (value < (int64_t)GOAL_MIN) {
        return GOAL_MIN;
    }
    if (value > (int64_t)GOAL_MAX) {
        return GOAL_MAX;
    }
    return (uint32_t)value;
}

static uint32_t wrap_pan_goal_i64(int64_t value)
{
    value %= PAN_TICKS_PER_REV;
    if (value < 0) {
        value += PAN_TICKS_PER_REV;
    }
    return (uint32_t)value;
}

static int64_t pan_delta_ticks_from_deg(int64_t angle_deg)
{
    int64_t numerator = angle_deg * PAN_TICKS_PER_REV;
    if (numerator >= 0) {
        return (numerator + 180) / 360;
    }
    return (numerator - 180) / 360;
}

static uint32_t audio_pan_goal_from_angle(int angle_deg, uint32_t front_pan)
{
    int64_t delta = pan_delta_ticks_from_deg((int64_t)angle_deg);
    return wrap_pan_goal_i64((int64_t)front_pan + delta);
}

static int64_t clamp_correction_i64(int64_t value, int64_t max_correction)
{
    if (value > max_correction) {
        return max_correction;
    }
    if (value < -max_correction) {
        return -max_correction;
    }
    return value;
}

static const uint32_t laser_cal_distance_mm[LASER_CAL_TABLE_SIZE] = {
    250u, 500u, 750u, 1000u, 1250u, 1500u,
    1750u, 2000u, 2250u, 2500u, 2750u, 3000u
};

static const uint32_t laser_cal_tick[LASER_CAL_TABLE_SIZE] = {
    1920u, 1952u, 1978u, 1985u, 1992u, 2000u,
    2000u, 2000u, 2002u, 2002u, 2006u, 2006u
};

static int goal_pair_valid(uint32_t pan_goal, uint32_t tilt_goal)
{
    if (pan_goal > GOAL_MAX || tilt_goal > GOAL_MAX) {
        return 0;
    }
    return !(pan_goal == 0u && tilt_goal == 0u);
}

static int front_center_sane(uint32_t front_pan, uint32_t front_tilt)
{
    if (front_pan < 512u || front_pan > 3583u) {
        return 0;
    }
    if (front_tilt < 1800u || front_tilt > 3400u) {
        return 0;
    }
    return 1;
}

static uint32_t clamp_around_front_i64(int64_t value, uint32_t front, int64_t limit)
{
    int64_t lo = (int64_t)front - limit;
    int64_t hi = (int64_t)front + limit;
    if (lo < (int64_t)GOAL_MIN) {
        lo = GOAL_MIN;
    }
    if (hi > (int64_t)GOAL_MAX) {
        hi = GOAL_MAX;
    }
    if (value < lo) {
        return (uint32_t)lo;
    }
    if (value > hi) {
        return (uint32_t)hi;
    }
    return (uint32_t)value;
}

static int goal_pair_near_front(uint32_t pan_goal, uint32_t tilt_goal,
                                uint32_t front_pan, uint32_t front_tilt)
{
    if (!goal_pair_valid(pan_goal, tilt_goal)) {
        return 0;
    }
    int64_t pan_delta = (int64_t)pan_goal - (int64_t)front_pan;
    int64_t tilt_delta = (int64_t)tilt_goal - (int64_t)front_tilt;
    if (pan_delta < -g_track_pan_limit || pan_delta > g_track_pan_limit) {
        return 0;
    }
    if (tilt_delta < -g_track_tilt_limit || tilt_delta > g_track_tilt_limit) {
        return 0;
    }
    return 1;
}

static void clamp_goal_pair_near_front(uint32_t *pan_goal, uint32_t *tilt_goal,
                                       uint32_t front_pan, uint32_t front_tilt)
{
    *pan_goal = clamp_around_front_i64((int64_t)*pan_goal, front_pan, g_track_pan_limit);
    *tilt_goal = clamp_around_front_i64((int64_t)*tilt_goal, front_tilt, g_track_tilt_limit);
}

static void ps_track_step(uint32_t cx, uint32_t cy, uint32_t bw, uint32_t bh,
                          uint32_t fw, uint32_t fh, uint32_t valid,
                          uint32_t base_pan, uint32_t base_tilt,
                          uint32_t front_pan, uint32_t front_tilt,
                          uint32_t *pan_goal, uint32_t *tilt_goal)
{
    if (!valid || fw == 0u || fh == 0u) {
        return;
    }

    int64_t err_x = (int64_t)cx - (int64_t)(fw / 2u);
    int64_t err_y = (int64_t)cy - (int64_t)(fh / 2u);
    int64_t max_correction = (
        bw >= TRACK_CLOSE_BOX_W || bh >= TRACK_CLOSE_BOX_H
    ) ? TRACK_MAX_CORRECTION_CLOSE : TRACK_MAX_CORRECTION;
    int64_t pan_correction = 0;
    int64_t tilt_correction = 0;
    if (err_x > TRACK_DEADBAND_X || err_x < -TRACK_DEADBAND_X) {
        pan_correction = clamp_correction_i64(
            (err_x * TRACK_TICK_PER_PIXEL_NUM) / TRACK_TICK_PER_PIXEL_DEN,
            max_correction);
    }
    if (err_y > TRACK_DEADBAND_Y || err_y < -TRACK_DEADBAND_Y) {
        tilt_correction = clamp_correction_i64(
            (err_y * TRACK_TICK_PER_PIXEL_NUM) / TRACK_TICK_PER_PIXEL_DEN,
            max_correction);
    }

    *pan_goal = clamp_around_front_i64((int64_t)base_pan + pan_correction,
                                       front_pan, g_track_pan_limit);
    *tilt_goal = clamp_around_front_i64((int64_t)base_tilt - tilt_correction,
                                        front_tilt, g_track_tilt_limit);
}

static void reg_write(volatile uint32_t *regs, uint32_t offset, uint32_t value)
{
    regs[offset / 4u] = value;
}

static uint32_t reg_read(volatile uint32_t *regs, uint32_t offset)
{
    return regs[offset / 4u];
}

static void pl_ctrl_write(volatile uint32_t *regs, uint32_t value)
{
    __sync_synchronize();
    reg_write(regs, REG_CTRL, value);
    __sync_synchronize();
    (void)reg_read(regs, REG_CTRL);
    __sync_synchronize();
}

static uint32_t pl_read_pan_goal(volatile uint32_t *regs)
{
    return reg_read(regs, REG_CTRL);
}

static uint32_t pl_read_tilt_goal(volatile uint32_t *regs)
{
    return reg_read(regs, REG_STATUS);
}

static uint32_t pl_read_count(volatile uint32_t *regs)
{
    return reg_read(regs, REG_LAST_PAN);
}

static uint32_t pl_read_flags(volatile uint32_t *regs)
{
    return reg_read(regs, REG_LAST_TILT);
}

static void pl_cmd_set_goal(volatile uint32_t *regs, uint32_t pan, uint32_t tilt)
{
    pl_ctrl_write(regs, PL_CMD_SET_PAN | (pan & 0xfffu));
    pl_ctrl_write(regs, PL_CMD_SET_TILT | (tilt & 0xfffu));
}

static void pl_cmd_set_track(volatile uint32_t *regs, uint32_t cx, uint32_t cy,
                             uint32_t bw, uint32_t bh,
                             uint32_t fw, uint32_t fh, uint32_t conf, uint32_t valid)
{
    pl_ctrl_write(regs, PL_CMD_SET_CX | (cx & 0xffffu));
    pl_ctrl_write(regs, PL_CMD_SET_CY | (cy & 0xffffu));
    pl_ctrl_write(regs, PL_CMD_SET_BOX | ((bh & 0xfffu) << 16) | (bw & 0xfffu));
    pl_ctrl_write(regs, PL_CMD_SET_FW | (fw & 0xffffu));
    pl_ctrl_write(regs, PL_CMD_SET_FH | (fh & 0xffffu));
    pl_ctrl_write(regs, PL_CMD_TRACK | ((conf & 0xffffu) << 8) |
                         (valid ? TRACK_CMD_VALID : 0u) | TRACK_CMD_TRACK);
}

static void pl_init_defaults(volatile uint32_t *regs, uint32_t pan_id, uint32_t tilt_id,
                             uint32_t front_pan, uint32_t front_tilt)
{
    pl_cmd_set_goal(regs, front_pan, front_tilt);
    pl_ctrl_write(regs, PL_CMD_SET_CX | 640u);
    pl_ctrl_write(regs, PL_CMD_SET_CY | 360u);
    pl_ctrl_write(regs, PL_CMD_SET_BOX | (60u << 16) | 80u);
    pl_ctrl_write(regs, PL_CMD_SET_FW | 1280u);
    pl_ctrl_write(regs, PL_CMD_SET_FH | 720u);
    reg_write(regs, REG_IDS, ((tilt_id & 0xffu) << 8) | (pan_id & 0xffu));
    pl_ctrl_write(regs, 1u);
}

static void load_center_file(const char *path, uint32_t *front_pan, uint32_t *front_tilt)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        return;
    }

    char line[128];
    while (fgets(line, sizeof(line), fp)) {
        unsigned int value = 0;
        if (sscanf(line, "PAN=%u", &value) == 1 && value <= GOAL_MAX) {
            *front_pan = value;
        } else if (sscanf(line, "TILT=%u", &value) == 1 && value <= GOAL_MAX) {
            *front_tilt = value;
        }
    }
    fclose(fp);
}

static int save_center_file(const char *path, uint32_t front_pan, uint32_t front_tilt)
{
    FILE *fp = fopen(path, "w");
    if (!fp) {
        return -1;
    }
    fprintf(fp, "PAN=%u\nTILT=%u\n", front_pan, front_tilt);
    fclose(fp);
    return 0;
}

static int open_pl_regs(uint32_t base, int *mem_fd, void **map, volatile uint32_t **regs)
{
    if (*regs != NULL) {
        return 0;
    }

    *mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (*mem_fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    off_t page_base = (off_t)(base & ~(MAP_SIZE - 1u));
    off_t page_off = (off_t)(base - (uint32_t)page_base);
    *map = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, *mem_fd, page_base);
    if (*map == MAP_FAILED) {
        perror("mmap");
        close(*mem_fd);
        *mem_fd = -1;
        return -1;
    }

    *regs = (volatile uint32_t *)((uint8_t *)*map + page_off);
    return 0;
}

static void close_pl_regs(int *mem_fd, void **map, volatile uint32_t **regs)
{
    *regs = NULL;
    if (*map != MAP_FAILED) {
        munmap(*map, MAP_SIZE);
        *map = MAP_FAILED;
    }
    if (*mem_fd >= 0) {
        close(*mem_fd);
        *mem_fd = -1;
    }
}

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 57600: return B57600;
    case 115200: return B115200;
    case 1000000:
#ifdef B1000000
        return B1000000;
#else
        return B57600;
#endif
    default: return B57600;
    }
}

static int open_serial(const char *path, int baud)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    speed_t speed = baud_to_speed(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static uint16_t dxl_crc_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000u) {
            crc = (uint16_t)((crc << 1) ^ 0x8005u);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static int write_all(int fd, const uint8_t *buf, size_t len)
{
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(fd, buf + off, len - off);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        off += (size_t)n;
    }
    tcdrain(fd);
    return 0;
}

static int dxl_write_data(int fd, uint8_t id, uint16_t addr,
                          const uint8_t *data, uint16_t data_len, int dry_run)
{
    uint8_t pkt[32];
    uint16_t param_len = (uint16_t)(2u + data_len);
    uint16_t pkt_len = (uint16_t)(param_len + 3u);
    size_t total_len = (size_t)(7u + pkt_len);
    if (total_len > sizeof(pkt)) {
        return -1;
    }

    pkt[0] = 0xff;
    pkt[1] = 0xff;
    pkt[2] = 0xfd;
    pkt[3] = 0x00;
    pkt[4] = id;
    pkt[5] = (uint8_t)(pkt_len & 0xffu);
    pkt[6] = (uint8_t)((pkt_len >> 8) & 0xffu);
    pkt[7] = 0x03;
    pkt[8] = (uint8_t)(addr & 0xffu);
    pkt[9] = (uint8_t)((addr >> 8) & 0xffu);
    for (uint16_t i = 0; i < data_len; i++) {
        pkt[10u + i] = data[i];
    }

    uint16_t crc = 0;
    for (size_t i = 0; i < total_len - 2u; i++) {
        crc = dxl_crc_update(crc, pkt[i]);
    }
    pkt[total_len - 2u] = (uint8_t)(crc & 0xffu);
    pkt[total_len - 1u] = (uint8_t)((crc >> 8) & 0xffu);

    if (dry_run) {
        return 0;
    }
    return write_all(fd, pkt, total_len);
}

static int dxl_read_data(int fd, uint8_t id, uint16_t addr,
                         uint8_t *data, uint16_t data_len, int dry_run)
{
    uint8_t params[4];
    params[0] = (uint8_t)(addr & 0xffu);
    params[1] = (uint8_t)((addr >> 8) & 0xffu);
    params[2] = (uint8_t)(data_len & 0xffu);
    params[3] = (uint8_t)((data_len >> 8) & 0xffu);

    uint8_t pkt[16];
    uint16_t pkt_len = 7u;
    pkt[0] = 0xff;
    pkt[1] = 0xff;
    pkt[2] = 0xfd;
    pkt[3] = 0x00;
    pkt[4] = id;
    pkt[5] = (uint8_t)(pkt_len & 0xffu);
    pkt[6] = (uint8_t)((pkt_len >> 8) & 0xffu);
    pkt[7] = 0x02;
    memcpy(&pkt[8], params, sizeof(params));

    uint16_t crc = 0;
    for (size_t i = 0; i < 12u; i++) {
        crc = dxl_crc_update(crc, pkt[i]);
    }
    pkt[12] = (uint8_t)(crc & 0xffu);
    pkt[13] = (uint8_t)((crc >> 8) & 0xffu);

    if (dry_run) {
        memset(data, 0, data_len);
        return 0;
    }
    tcflush(fd, TCIFLUSH);
    if (write_all(fd, pkt, 14u) != 0) {
        return -1;
    }

    uint8_t rx[64];
    size_t rx_len = 0;
    for (int tries = 0; tries < 20; tries++) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        int sel = select(fd + 1, &read_fds, NULL, NULL, &tv);
        if (sel > 0 && FD_ISSET(fd, &read_fds)) {
            ssize_t n = read(fd, rx + rx_len, sizeof(rx) - rx_len);
            if (n > 0) {
                rx_len += (size_t)n;
            }
        }

        for (size_t start = 0; start + 10u <= rx_len; start++) {
            if (rx[start] != 0xff || rx[start + 1u] != 0xff ||
                rx[start + 2u] != 0xfd || rx[start + 3u] != 0x00 ||
                rx[start + 4u] != id) {
                continue;
            }
            uint16_t len = (uint16_t)rx[start + 5u] | ((uint16_t)rx[start + 6u] << 8);
            size_t total = start + 7u + len;
            if (total > rx_len) {
                continue;
            }
            if (rx[start + 7u] != 0x55 || rx[start + 8u] != 0x00 || len < (uint16_t)(data_len + 4u)) {
                return -1;
            }
            uint16_t got_crc = (uint16_t)rx[total - 2u] | ((uint16_t)rx[total - 1u] << 8);
            uint16_t calc_crc = 0;
            for (size_t i = start; i < total - 2u; i++) {
                calc_crc = dxl_crc_update(calc_crc, rx[i]);
            }
            if (got_crc != calc_crc) {
                return -1;
            }
            memcpy(data, &rx[start + 9u], data_len);
            return 0;
        }
    }
    return -1;
}

static int dxl_write1(int fd, uint8_t id, uint16_t addr, uint8_t value, int dry_run)
{
    return dxl_write_data(fd, id, addr, &value, 1u, dry_run);
}

static int dxl_write4(int fd, uint8_t id, uint16_t addr, uint32_t value, int dry_run)
{
    uint8_t data[4];
    data[0] = (uint8_t)(value & 0xffu);
    data[1] = (uint8_t)((value >> 8) & 0xffu);
    data[2] = (uint8_t)((value >> 16) & 0xffu);
    data[3] = (uint8_t)((value >> 24) & 0xffu);
    return dxl_write_data(fd, id, addr, data, 4u, dry_run);
}

static int dxl_sync_write_goal_pair(int fd, uint8_t pan_id, uint8_t tilt_id,
                                    uint32_t pan_goal, uint32_t tilt_goal,
                                    int dry_run)
{
    uint8_t pkt[32];
    const uint16_t data_len = 4u;
    const uint16_t param_len = 4u + (uint16_t)(2u * (1u + data_len));
    const uint16_t pkt_len = param_len + 3u;
    const size_t total_len = (size_t)(7u + pkt_len);
    if (total_len > sizeof(pkt)) {
        return -1;
    }

    pkt[0] = 0xff;
    pkt[1] = 0xff;
    pkt[2] = 0xfd;
    pkt[3] = 0x00;
    pkt[4] = 0xfe;
    pkt[5] = (uint8_t)(pkt_len & 0xffu);
    pkt[6] = (uint8_t)((pkt_len >> 8) & 0xffu);
    pkt[7] = 0x83;
    pkt[8] = (uint8_t)(ADDR_GOAL_POSITION & 0xffu);
    pkt[9] = (uint8_t)((ADDR_GOAL_POSITION >> 8) & 0xffu);
    pkt[10] = (uint8_t)(data_len & 0xffu);
    pkt[11] = (uint8_t)((data_len >> 8) & 0xffu);

    pkt[12] = pan_id;
    pkt[13] = (uint8_t)(pan_goal & 0xffu);
    pkt[14] = (uint8_t)((pan_goal >> 8) & 0xffu);
    pkt[15] = (uint8_t)((pan_goal >> 16) & 0xffu);
    pkt[16] = (uint8_t)((pan_goal >> 24) & 0xffu);

    pkt[17] = tilt_id;
    pkt[18] = (uint8_t)(tilt_goal & 0xffu);
    pkt[19] = (uint8_t)((tilt_goal >> 8) & 0xffu);
    pkt[20] = (uint8_t)((tilt_goal >> 16) & 0xffu);
    pkt[21] = (uint8_t)((tilt_goal >> 24) & 0xffu);

    uint16_t crc = 0;
    for (size_t i = 0; i < total_len - 2u; i++) {
        crc = dxl_crc_update(crc, pkt[i]);
    }
    pkt[total_len - 2u] = (uint8_t)(crc & 0xffu);
    pkt[total_len - 1u] = (uint8_t)((crc >> 8) & 0xffu);

    if (dry_run) {
        return 0;
    }
    return write_all(fd, pkt, total_len);
}

static int dxl_read4(int fd, uint8_t id, uint16_t addr, uint32_t *value, int dry_run)
{
    uint8_t data[4];
    if (dxl_read_data(fd, id, addr, data, 4u, dry_run) != 0) {
        return -1;
    }
    *value = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
             ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    return 0;
}

static int configure_dxl_axis(int fd, uint8_t id, uint32_t profile_accel,
                              uint32_t profile_velocity, int dry_run)
{
    if (dxl_write4(fd, id, ADDR_PROFILE_ACCEL, profile_accel, dry_run) != 0) {
        return -1;
    }
    usleep(1000);
    if (dxl_write4(fd, id, ADDR_PROFILE_VELOCITY, profile_velocity, dry_run) != 0) {
        return -1;
    }
    usleep(1000);
    if (dxl_write1(fd, id, ADDR_TORQUE_ENABLE, 1u, dry_run) != 0) {
        return -1;
    }
    return 0;
}

static int configure_dxl_pair(int fd, uint32_t pan_id, uint32_t tilt_id,
                              uint32_t profile_accel, uint32_t profile_velocity,
                              int dry_run)
{
    if (configure_dxl_axis(fd, (uint8_t)pan_id, profile_accel, profile_velocity, dry_run) != 0) {
        return -1;
    }
    usleep(1000);
    if (configure_dxl_axis(fd, (uint8_t)tilt_id, profile_accel, profile_velocity, dry_run) != 0) {
        return -1;
    }
    return 0;
}

static uint32_t laser_base_from_distance(uint32_t distance_mm, uint32_t fallback_tick)
{
    uint32_t dist = distance_mm ? distance_mm : 3000u;
    if (dist <= laser_cal_distance_mm[0]) {
        return laser_cal_tick[0];
    }
    if (dist >= laser_cal_distance_mm[LASER_CAL_TABLE_SIZE - 1u]) {
        return laser_cal_tick[LASER_CAL_TABLE_SIZE - 1u];
    }

    for (uint32_t i = 0u; i + 1u < LASER_CAL_TABLE_SIZE; i++) {
        uint32_t d0 = laser_cal_distance_mm[i];
        uint32_t d1 = laser_cal_distance_mm[i + 1u];
        if (dist >= d0 && dist <= d1) {
            int64_t t0 = (int64_t)laser_cal_tick[i];
            int64_t t1 = (int64_t)laser_cal_tick[i + 1u];
            int64_t num = ((int64_t)dist - (int64_t)d0) * (t1 - t0);
            int64_t den = (int64_t)d1 - (int64_t)d0;
            return clamp_goal_i64(t0 + (den ? (num / den) : 0));
        }
    }
    return clamp_goal_i64(fallback_tick);
}

static uint32_t laser_goal_from_tilt(uint32_t tilt_goal, uint32_t front_tilt,
                                     uint32_t laser_center, uint32_t distance_mm,
                                     int32_t offset_mm, int32_t sign,
                                     int64_t image_offset_ticks)
{
    (void)tilt_goal;
    (void)front_tilt;
    (void)offset_mm;
    (void)sign;
    uint32_t base_tick = laser_base_from_distance(distance_mm, laser_center);
    return clamp_goal_i64(
        (int64_t)base_tick + image_offset_ticks);
}

static int64_t laser_image_offset_ticks(uint32_t cy, uint32_t frame_h,
                                        uint32_t vertical_fov_deg)
{
    if (frame_h == 0u || vertical_fov_deg == 0u) {
        return 0;
    }
    int64_t err_y = (int64_t)cy - (int64_t)(frame_h / 2u);
    /*
     * Image y grows downward. Laser ticks grow upward, so negate err_y.
     * This points the laser directly at the bbox center before applying
     * the fixed camera-to-laser height correction.
     */
    int64_t num = -err_y * (int64_t)vertical_fov_deg *
                  (int64_t)LASER_TICKS_PER_DEG_NUM;
    int64_t den = (int64_t)LASER_TICKS_PER_DEG_DEN * (int64_t)frame_h;
    return den ? (num / den) : 0;
}

static int send_goals_usb(int serial_fd, uint32_t pan_id, uint32_t tilt_id,
                          uint32_t pan_goal, uint32_t tilt_goal,
                          int laser_enable, uint32_t laser_id, uint32_t laser_goal,
                          int dry_run)
{
    if (!dry_run && serial_fd < 0) {
        return -1;
    }
    if (dxl_sync_write_goal_pair(serial_fd, (uint8_t)pan_id, (uint8_t)tilt_id,
                                 pan_goal, tilt_goal, dry_run) != 0) {
        return -1;
    }
    if (laser_enable) {
        usleep(1000);
        if (dxl_write4(serial_fd, (uint8_t)laser_id, ADDR_GOAL_POSITION,
                       laser_goal, dry_run) != 0) {
            return -1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    uint32_t base = 0;
    int udp_port = 5016;
    uint32_t pan_id = 1;
    uint32_t tilt_id = 2;
    const char *serial_path = "/dev/ttyUSB0";
    int baud = 57600;
    int dry_run = 0;
    int no_pl = 0;
    int skip_pl_init = 0;
    int skip_dxl_init = 0;
    int lazy_pl_open = 0;
    int track_direct_ps = 0;
    int track_pl_shadow = 0;
    uint32_t profile_accel = 60u;
    uint32_t profile_velocity = 180u;
    int laser_enable = 1;
    uint32_t laser_id = 3u;
    uint32_t laser_center = 2048u;
    int32_t laser_offset_mm = 38;
    uint32_t laser_default_distance_mm = LASER_DEFAULT_DISTANCE_MM;
    uint32_t laser_vertical_fov_deg = LASER_DEFAULT_VERTICAL_FOV_DEG;
    int32_t laser_sign = -1;
    const char *center_file = "/home/xilinx/ultra_yubin_v1/front_center.env";
    uint32_t front_pan = PAN_CENTER;
    uint32_t front_tilt = TILT_CENTER;
    uint32_t sw_ctrl = 1u;
    uint32_t sw_count = 0u;
    uint32_t sw_pan = PAN_CENTER;
    uint32_t sw_tilt = TILT_CENTER;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--base") && i + 1 < argc) {
            base = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            udp_port = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--serial") && i + 1 < argc) {
            serial_path = argv[++i];
        } else if (!strcmp(argv[i], "--baud") && i + 1 < argc) {
            baud = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--pan-id") && i + 1 < argc) {
            pan_id = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--tilt-id") && i + 1 < argc) {
            tilt_id = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--dry-run")) {
            dry_run = 1;
        } else if (!strcmp(argv[i], "--no-pl")) {
            no_pl = 1;
        } else if (!strcmp(argv[i], "--skip-pl-init")) {
            skip_pl_init = 1;
        } else if (!strcmp(argv[i], "--skip-dxl-init")) {
            skip_dxl_init = 1;
        } else if (!strcmp(argv[i], "--lazy-pl-open")) {
            lazy_pl_open = 1;
        } else if (!strcmp(argv[i], "--track-direct-ps")) {
            track_direct_ps = 1;
        } else if (!strcmp(argv[i], "--track-pl")) {
            track_direct_ps = 0;
        } else if (!strcmp(argv[i], "--track-pl-shadow")) {
            track_pl_shadow = 1;
        } else if (!strcmp(argv[i], "--track-pan-limit") && i + 1 < argc) {
            g_track_pan_limit = (int64_t)parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--track-tilt-limit") && i + 1 < argc) {
            g_track_tilt_limit = (int64_t)parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--profile-accel") && i + 1 < argc) {
            profile_accel = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--profile-velocity") && i + 1 < argc) {
            profile_velocity = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--laser-disable")) {
            laser_enable = 0;
        } else if (!strcmp(argv[i], "--laser-id") && i + 1 < argc) {
            laser_id = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--laser-center") && i + 1 < argc) {
            laser_center = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--laser-offset-mm") && i + 1 < argc) {
            laser_offset_mm = (int32_t)strtol(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--laser-distance-mm") && i + 1 < argc) {
            laser_default_distance_mm = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--laser-vertical-fov-deg") && i + 1 < argc) {
            laser_vertical_fov_deg = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--laser-sign") && i + 1 < argc) {
            laser_sign = (int32_t)strtol(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--center-file") && i + 1 < argc) {
            center_file = argv[++i];
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (base == 0 && !no_pl) {
        usage(argv[0]);
        return 2;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    load_center_file(center_file, &front_pan, &front_tilt);
    if (!front_center_sane(front_pan, front_tilt)) {
        fprintf(stderr,
                "[ultra_yubin_v1] ignoring invalid center file values pan=%u tilt=%u; using defaults %u,%u\n",
                front_pan, front_tilt, PAN_CENTER, TILT_CENTER);
        front_pan = PAN_CENTER;
        front_tilt = TILT_CENTER;
        (void)save_center_file(center_file, front_pan, front_tilt);
    }
    sw_pan = front_pan;
    sw_tilt = front_tilt;

    int mem_fd = -1;
    void *map = MAP_FAILED;
    volatile uint32_t *regs = NULL;
    if (!no_pl && !lazy_pl_open) {
        if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
            return 1;
        }
    }

    int serial_fd = -1;
    if (!dry_run) {
        serial_fd = open_serial(serial_path, baud);
        if (serial_fd < 0) {
            perror("open serial");
            close_pl_regs(&mem_fd, &map, &regs);
            return 1;
        }
        if (!skip_dxl_init && configure_dxl_pair(serial_fd, pan_id, tilt_id,
                                                 profile_accel, profile_velocity, dry_run) != 0) {
            perror("configure dynamixel");
            close(serial_fd);
            close_pl_regs(&mem_fd, &map, &regs);
            return 1;
        }
        if (!skip_dxl_init && laser_enable &&
            configure_dxl_axis(serial_fd, (uint8_t)laser_id,
                               profile_accel, profile_velocity, dry_run) != 0) {
            perror("configure laser dynamixel");
            close(serial_fd);
            close_pl_regs(&mem_fd, &map, &regs);
            return 1;
        }
    } else {
        (void)configure_dxl_pair(serial_fd, pan_id, tilt_id,
                                 profile_accel, profile_velocity, dry_run);
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        if (serial_fd >= 0) close(serial_fd);
        close_pl_regs(&mem_fd, &map, &regs);
        return 1;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons((uint16_t)udp_port);

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        perror("bind");
        close(sock);
        if (serial_fd >= 0) close(serial_fd);
        close_pl_regs(&mem_fd, &map, &regs);
        return 1;
    }

    if (!no_pl && !skip_pl_init) {
        if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
            close(sock);
            if (serial_fd >= 0) close(serial_fd);
            close_pl_regs(&mem_fd, &map, &regs);
            return 1;
        }
        pl_init_defaults(regs, pan_id, tilt_id, front_pan, front_tilt);
    }

    printf("[ultra_yubin_v1] UDP listen 0.0.0.0:%d, AXI base 0x%08x\n", udp_port, base);
    printf("[ultra_yubin_v1] serial=%s baud=%d dry_run=%d no_pl=%d skip_pl_init=%d skip_dxl_init=%d lazy_pl_open=%d track_direct_ps=%d track_pl_shadow=%d track_pan_limit=%lld track_tilt_limit=%lld pan_id=%u tilt_id=%u laser_id=%u laser_center=%u laser_offset_mm=%d laser_distance_mm=%u laser_vertical_fov_deg=%u laser_sign=%d profile_accel=%u profile_velocity=%u center=(%u,%u) center_file=%s\n",
           serial_path, baud, dry_run, no_pl, skip_pl_init, skip_dxl_init, lazy_pl_open, track_direct_ps, track_pl_shadow,
           (long long)g_track_pan_limit, (long long)g_track_tilt_limit,
           pan_id, tilt_id, laser_id, laser_center, laser_offset_mm,
           laser_default_distance_mm, laser_vertical_fov_deg, laser_sign,
           profile_accel, profile_velocity, front_pan, front_tilt, center_file);
    fflush(stdout);

    while (g_running) {
        char buf[256];
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                             (struct sockaddr *)&peer, &peer_len);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("recvfrom");
            break;
        }

        for (;;) {
            char newer[256];
            struct sockaddr_in newer_peer;
            socklen_t newer_peer_len = sizeof(newer_peer);
            ssize_t newer_n = recvfrom(sock, newer, sizeof(newer) - 1,
                                       MSG_DONTWAIT,
                                       (struct sockaddr *)&newer_peer,
                                       &newer_peer_len);
            if (newer_n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    break;
                }
                perror("recvfrom drain");
                break;
            }
            memcpy(buf, newer, (size_t)newer_n);
            n = newer_n;
            peer = newer_peer;
            peer_len = newer_peer_len;
        }

        buf[n] = '\0';
        while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r' || buf[n - 1] == ' ')) {
            buf[--n] = '\0';
        }

        if (!strcmp(buf, "PING")) {
            char reply[160];
            snprintf(reply, sizeof(reply), "PONG,UDP,ULTRA_YUBIN_V1,base=0x%08x,port=%d,dry=%d,no_pl=%d,lazy=%d,pl_open=%d\n",
                     base, udp_port, dry_run, no_pl, lazy_pl_open, regs != NULL);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        if (!strcmp(buf, "PLPING")) {
            if (!no_pl && open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                const char *reply = "ERR,pl-open-failed\n";
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                continue;
            }
            uint32_t ctrl = no_pl ? sw_ctrl : pl_read_flags(regs);
            uint32_t count = no_pl ? sw_count : pl_read_count(regs);
            uint32_t raw_pan = no_pl ? sw_pan : pl_read_pan_goal(regs);
            uint32_t raw_tilt = no_pl ? sw_tilt : pl_read_tilt_goal(regs);
            uint32_t pan = raw_pan;
            uint32_t tilt = raw_tilt;
            const char *goal_src = no_pl ? "ps" : "pl";
            if (!goal_pair_near_front(raw_pan, raw_tilt, front_pan, front_tilt)) {
                pan = sw_pan;
                tilt = sw_tilt;
                goal_src = "ps_shadow";
                if (!goal_pair_near_front(pan, tilt, front_pan, front_tilt)) {
                    pan = front_pan;
                    tilt = front_tilt;
                    sw_pan = front_pan;
                    sw_tilt = front_tilt;
                    goal_src = "front_fallback";
                }
            } else {
                sw_pan = raw_pan;
                sw_tilt = raw_tilt;
            }
            char reply[224];
            snprintf(reply, sizeof(reply),
                     "PONG,PL,ULTRA_YUBIN_V1,ctrl=0x%08x,count=%u,pan=%u,tilt=%u,raw_pan=%u,raw_tilt=%u,dry=%d,no_pl=%d,src=%s\n",
                     ctrl, count, pan, tilt, raw_pan, raw_tilt, dry_run, no_pl, goal_src);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int torque_enable = 0;
        if (sscanf(buf, "TORQUE %u", &torque_enable) == 1) {
            int usb_ok = 0;
            if (torque_enable) {
                usb_ok = configure_dxl_pair(serial_fd, pan_id, tilt_id,
                                            profile_accel, profile_velocity, dry_run) == 0;
            } else {
                usb_ok = dxl_write1(serial_fd, (uint8_t)pan_id, ADDR_TORQUE_ENABLE, 0u, dry_run) == 0;
                usleep(200);
                usb_ok = usb_ok && dxl_write1(serial_fd, (uint8_t)tilt_id, ADDR_TORQUE_ENABLE, 0u, dry_run) == 0;
            }
            char reply[96];
            snprintf(reply, sizeof(reply), "TORQUE,1,enable=%u,usb=%d,dry=%d,no_pl=%d\n",
                     torque_enable ? 1u : 0u, usb_ok, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        if (!strcmp(buf, "READPOS")) {
            uint32_t pan_pos = sw_pan;
            uint32_t tilt_pos = sw_tilt;
            int usb_ok = 1;
            if (!dry_run) {
                usb_ok = dxl_read4(serial_fd, (uint8_t)pan_id, ADDR_PRESENT_POSITION, &pan_pos, dry_run) == 0;
                usleep(1000);
                usb_ok = usb_ok && dxl_read4(serial_fd, (uint8_t)tilt_id, ADDR_PRESENT_POSITION, &tilt_pos, dry_run) == 0;
            }
            char reply[128];
            snprintf(reply, sizeof(reply), "POS,1,pan=%u,tilt=%u,usb=%d,dry=%d,no_pl=%d\n",
                     pan_pos, tilt_pos, usb_ok, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int set_pan = 0;
        unsigned int set_tilt = 0;
        if (!strcmp(buf, "SETCENTER") || sscanf(buf, "SETCENTER %u %u", &set_pan, &set_tilt) == 2) {
            uint32_t new_pan = set_pan;
            uint32_t new_tilt = set_tilt;
            int usb_ok = 1;
            if (!strcmp(buf, "SETCENTER")) {
                new_pan = sw_pan;
                new_tilt = sw_tilt;
                if (!dry_run) {
                    usb_ok = dxl_read4(serial_fd, (uint8_t)pan_id, ADDR_PRESENT_POSITION, &new_pan, dry_run) == 0;
                    usleep(1000);
                    usb_ok = usb_ok && dxl_read4(serial_fd, (uint8_t)tilt_id, ADDR_PRESENT_POSITION, &new_tilt, dry_run) == 0;
                }
            }
            if (new_pan <= GOAL_MAX && new_tilt <= GOAL_MAX) {
                front_pan = new_pan;
                front_tilt = new_tilt;
                sw_pan = front_pan;
                sw_tilt = front_tilt;
                if (!no_pl && open_pl_regs(base, &mem_fd, &map, &regs) == 0) {
                    pl_cmd_set_goal(regs, front_pan, front_tilt);
                }
                int saved = save_center_file(center_file, front_pan, front_tilt) == 0;
                char reply[160];
                snprintf(reply, sizeof(reply), "CENTER,1,pan=%u,tilt=%u,usb=%d,saved=%d,file=%s\n",
                         front_pan, front_tilt, usb_ok, saved, center_file);
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            } else {
                const char *reply = "ERR,bad-center\n";
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            }
            continue;
        }

        if (!strcmp(buf, "CENTER")) {
            sw_pan = front_pan;
            sw_tilt = front_tilt;
            if (!no_pl && open_pl_regs(base, &mem_fd, &map, &regs) == 0) {
                pl_cmd_set_goal(regs, front_pan, front_tilt);
                pl_ctrl_write(regs, 1u);
            }
            uint32_t laser_goal = laser_goal_from_tilt(front_tilt, front_tilt,
                                                       laser_center, laser_default_distance_mm,
                                                       laser_offset_mm, laser_sign, 0);
            sw_pan = front_pan;
            sw_tilt = front_tilt;
            sw_count++;
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, front_pan, front_tilt,
                                        laser_enable, laser_id, laser_goal, dry_run) == 0;
            char reply[160];
            snprintf(reply, sizeof(reply), "CENTER,1,pan=%u,tilt=%u,laser=%u,laser_id=%u,usb=%d,dry=%d,no_pl=%d\n",
                     front_pan, front_tilt, laser_goal, laser_id, usb_ok, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int pan = 0;
        unsigned int tilt = 0;
        if (sscanf(buf, "G %u %u", &pan, &tilt) == 2) {
            pan = clamp_goal_i64(pan);
            tilt = clamp_goal_i64(tilt);
            if (no_pl) {
            } else {
                if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                    const char *reply = "ERR,pl-open-failed\n";
                    sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                    continue;
                }
                pl_cmd_set_goal(regs, pan, tilt);
                pl_ctrl_write(regs, 1u);
            }
            sw_pan = pan;
            sw_tilt = tilt;
            sw_count++;
            uint32_t laser_goal = laser_goal_from_tilt(tilt, front_tilt,
                                                       laser_center, laser_default_distance_mm,
                                                       laser_offset_mm, laser_sign, 0);
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, pan, tilt,
                                        laser_enable, laser_id, laser_goal, dry_run) == 0;
            uint32_t count = sw_count;
            char reply[192];
            snprintf(reply, sizeof(reply), "S,1,pan=%u,tilt=%u,laser=%u,laser_id=%u,usb=%d,count=%u,dry=%d,no_pl=%d\n",
                     pan, tilt, laser_goal, laser_id, usb_ok, count, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int dxl_id = 0;
        unsigned int dxl_goal = 0;
        if (sscanf(buf, "D %u %u", &dxl_id, &dxl_goal) == 2) {
            dxl_id &= 0xffu;
            dxl_goal = clamp_goal_i64(dxl_goal);
            int config_ok = configure_dxl_axis(serial_fd, (uint8_t)dxl_id,
                                               profile_accel, profile_velocity,
                                               dry_run) == 0;
            int usb_ok = config_ok && dxl_write4(serial_fd, (uint8_t)dxl_id,
                                                 ADDR_GOAL_POSITION, dxl_goal,
                                                 dry_run) == 0;
            char reply[160];
            snprintf(reply, sizeof(reply),
                     "D,1,id=%u,goal=%u,usb=%d,config=%d,dry=%d,no_pl=%d\n",
                     dxl_id, dxl_goal, usb_ok, config_ok, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        int dxl_delta = 0;
        if (sscanf(buf, "DREL %u %d", &dxl_id, &dxl_delta) == 2) {
            dxl_id &= 0xffu;
            uint32_t present = 0u;
            int read_ok = dxl_read4(serial_fd, (uint8_t)dxl_id,
                                    ADDR_PRESENT_POSITION, &present,
                                    dry_run) == 0;
            uint32_t dxl_target = clamp_goal_i64((int64_t)present + (int64_t)dxl_delta);
            int config_ok = read_ok && configure_dxl_axis(serial_fd, (uint8_t)dxl_id,
                                                          profile_accel, profile_velocity,
                                                          dry_run) == 0;
            int usb_ok = config_ok && dxl_write4(serial_fd, (uint8_t)dxl_id,
                                                 ADDR_GOAL_POSITION, dxl_target,
                                                 dry_run) == 0;
            char reply[192];
            snprintf(reply, sizeof(reply),
                     "DREL,1,id=%u,present=%u,delta=%d,goal=%u,usb=%d,read=%d,config=%d,dry=%d,no_pl=%d\n",
                     dxl_id, present, dxl_delta, dxl_target, usb_ok, read_ok,
                     config_ok, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        if (!strcmp(buf, "PLTEST")) {
            if (no_pl || open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                const char *reply = "PLTEST,0,reason=pl-unavailable\n";
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                continue;
            }
            uint32_t before_pan = pl_read_pan_goal(regs);
            uint32_t before_tilt = pl_read_tilt_goal(regs);
            uint32_t before_count = pl_read_count(regs);
            pl_cmd_set_goal(regs, front_pan, front_tilt);
            pl_cmd_set_track(regs, 704u, 360u, 80u, 60u, 1280u, 720u, 900u, 1u);
            usleep(1000);
            uint32_t after_pan = pl_read_pan_goal(regs);
            uint32_t after_tilt = pl_read_tilt_goal(regs);
            uint32_t after_count = pl_read_count(regs);
            uint32_t after_box = reg_read(regs, REG_TRACK_BOX);
            uint32_t after_xy = reg_read(regs, REG_TRACK_XY);
            uint32_t after_frame = reg_read(regs, REG_TRACK_FRAME);
            uint32_t after_cmd = reg_read(regs, REG_TRACK_CMD);
            char reply[320];
            snprintf(reply, sizeof(reply),
                     "PLTEST,1,before_pan=%u,before_tilt=%u,before_count=%u,after_pan=%u,after_tilt=%u,after_count=%u,box=0x%08x,xy=0x%08x,frame=0x%08x,cmd=0x%08x,front_pan=%u,front_tilt=%u\n",
                     before_pan, before_tilt, before_count, after_pan, after_tilt, after_count,
                     after_box, after_xy, after_frame, after_cmd, front_pan, front_tilt);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int cx = 0, cy = 0, bw = 0, bh = 0, fw = 0, fh = 0, conf = 0, valid = 0;
        unsigned int distance_mm = laser_default_distance_mm;
        unsigned int laser_base_override = 0;
        int track_args = sscanf(buf, "T %u %u %u %u %u %u %u %u %u %u",
                                &cx, &cy, &bw, &bh, &fw, &fh, &conf, &valid,
                                &distance_mm, &laser_base_override);
        if (track_args >= 8) {
            uint32_t pan_now = sw_pan;
            uint32_t tilt_now = sw_tilt;
            uint32_t count = sw_count;
            const char *goal_src = "pl";
            char shadow_extra[192] = "";
            if (no_pl || track_direct_ps) {
                uint32_t ps_base_pan = sw_pan;
                uint32_t ps_base_tilt = sw_tilt;
                ps_track_step(cx, cy, bw, bh, fw, fh, valid,
                              sw_pan, sw_tilt, front_pan, front_tilt,
                              &sw_pan, &sw_tilt);
                pan_now = sw_pan;
                tilt_now = sw_tilt;
                sw_count++;
                count = sw_count;
                goal_src = no_pl ? "ps" : "ps_direct";
                if (track_pl_shadow && !no_pl) {
                    if (open_pl_regs(base, &mem_fd, &map, &regs) == 0) {
                        uint32_t shadow_base_pan = ps_base_pan;
                        uint32_t shadow_base_tilt = ps_base_tilt;
                        uint32_t shadow_pan = 0u;
                        uint32_t shadow_tilt = 0u;
                        uint32_t shadow_count = 0u;
                        int64_t shadow_diff_pan = 0;
                        int64_t shadow_diff_tilt = 0;

                        if (!goal_pair_near_front(shadow_base_pan, shadow_base_tilt,
                                                  front_pan, front_tilt)) {
                            shadow_base_pan = front_pan;
                            shadow_base_tilt = front_tilt;
                        }
                        clamp_goal_pair_near_front(&shadow_base_pan, &shadow_base_tilt,
                                                   front_pan, front_tilt);
                        pl_cmd_set_goal(regs, shadow_base_pan, shadow_base_tilt);
                        pl_cmd_set_track(regs, cx, cy, bw, bh, fw, fh, conf, valid);
                        usleep(1000);
                        shadow_pan = pl_read_pan_goal(regs);
                        shadow_tilt = pl_read_tilt_goal(regs);
                        shadow_count = pl_read_count(regs);
                        if (goal_pair_valid(shadow_pan, shadow_tilt)) {
                            clamp_goal_pair_near_front(&shadow_pan, &shadow_tilt,
                                                       front_pan, front_tilt);
                            shadow_diff_pan = (int64_t)shadow_pan - (int64_t)pan_now;
                            shadow_diff_tilt = (int64_t)shadow_tilt - (int64_t)tilt_now;
                            snprintf(shadow_extra, sizeof(shadow_extra),
                                     ",shadow=ok,pl_pan=%u,pl_tilt=%u,pl_count=%u,pl_diff_pan=%lld,pl_diff_tilt=%lld",
                                     shadow_pan, shadow_tilt, shadow_count,
                                     (long long)shadow_diff_pan,
                                     (long long)shadow_diff_tilt);
                        } else {
                            snprintf(shadow_extra, sizeof(shadow_extra),
                                     ",shadow=bad_pl,pl_pan=%u,pl_tilt=%u",
                                     shadow_pan, shadow_tilt);
                        }
                    } else {
                        snprintf(shadow_extra, sizeof(shadow_extra),
                                 ",shadow=pl_open_failed");
                    }
                }
            } else {
                if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                    const char *reply = "ERR,pl-open-failed\n";
                    sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                    continue;
                }
                uint32_t base_pan = pl_read_pan_goal(regs);
                uint32_t base_tilt = pl_read_tilt_goal(regs);
                if (!goal_pair_near_front(base_pan, base_tilt, front_pan, front_tilt)) {
                    base_pan = sw_pan;
                    base_tilt = sw_tilt;
                }
                if (!goal_pair_near_front(base_pan, base_tilt, front_pan, front_tilt)) {
                    base_pan = front_pan;
                    base_tilt = front_tilt;
                }
                clamp_goal_pair_near_front(&base_pan, &base_tilt, front_pan, front_tilt);

                pl_cmd_set_goal(regs, base_pan, base_tilt);
                pl_cmd_set_track(regs, cx, cy, bw, bh, fw, fh, conf, valid);
                usleep(1000);
                pan_now = pl_read_pan_goal(regs);
                tilt_now = pl_read_tilt_goal(regs);
                count = pl_read_count(regs);
                if (goal_pair_valid(pan_now, tilt_now)) {
                    clamp_goal_pair_near_front(&pan_now, &tilt_now, front_pan, front_tilt);
                    pl_cmd_set_goal(regs, pan_now, tilt_now);
                    sw_pan = pan_now;
                    sw_tilt = tilt_now;
                } else {
                    ps_track_step(cx, cy, bw, bh, fw, fh, valid,
                                  sw_pan, sw_tilt, front_pan, front_tilt,
                                  &sw_pan, &sw_tilt);
                    pan_now = sw_pan;
                    tilt_now = sw_tilt;
                    pl_cmd_set_goal(regs, pan_now, tilt_now);
                    goal_src = "ps_fallback";
                }
            }
            int64_t laser_img_ticks =
                laser_image_offset_ticks(cy, fh, laser_vertical_fov_deg);
            uint32_t laser_base = laser_base_override ?
                clamp_goal_i64(laser_base_override) :
                laser_base_from_distance(distance_mm, laser_center);
            uint32_t laser_goal = clamp_goal_i64((int64_t)laser_base + laser_img_ticks);
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, pan_now, tilt_now,
                                        laser_enable, laser_id, laser_goal, dry_run) == 0;
            char reply[512];
            snprintf(reply, sizeof(reply),
                     "T,1,cx=%u,cy=%u,bw=%u,bh=%u,fw=%u,fh=%u,conf=%u,valid=%u,dist=%u,laser_base=%u,pan=%u,tilt=%u,laser=%u,laser_img=%lld,laser_id=%u,usb=%d,count=%u,dry=%d,no_pl=%d,src=%s%s\n",
                     cx, cy, bw, bh, fw, fh, conf, valid, distance_mm, laser_base,
                     pan_now, tilt_now, laser_goal, (long long)laser_img_ticks, laser_id,
                     usb_ok, count, dry_run, no_pl, goal_src, shadow_extra);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        int angle = 0;
        if (sscanf(buf, "A %d %u %u", &angle, &conf, &valid) == 3) {
            uint32_t pan_now = sw_pan;
            uint32_t tilt_now = front_tilt;
            uint32_t count;
            if (valid) {
                pan_now = audio_pan_goal_from_angle(angle, front_pan);
                sw_pan = pan_now;
                sw_tilt = tilt_now;
                sw_count++;
                if (!no_pl && open_pl_regs(base, &mem_fd, &map, &regs) == 0) {
                    pl_cmd_set_goal(regs, pan_now, tilt_now);
                }
            }
            count = sw_count;
            uint32_t laser_goal = laser_goal_from_tilt(tilt_now, front_tilt,
                                                       laser_center, laser_default_distance_mm,
                                                       laser_offset_mm, laser_sign, 0);
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, pan_now, tilt_now,
                                        laser_enable, laser_id, laser_goal, dry_run) == 0;
            char reply[192];
            snprintf(reply, sizeof(reply),
                     "A,1,angle=%d,conf=%u,valid=%u,pan=%u,tilt=%u,laser=%u,laser_id=%u,usb=%d,count=%u,dry=%d,no_pl=%d,src=audio_direct,ps_audio=1\n",
                     angle, conf, valid, pan_now, tilt_now, laser_goal, laser_id,
                     usb_ok, count, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        const char *reply = "ERR,unknown-command\n";
        sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
    }

    close(sock);
    if (serial_fd >= 0) close(serial_fd);
    close_pl_regs(&mem_fd, &map, &regs);
    return 0;
}
