/*
 * QNX BrakeWire ADAS - Shared IPC, CAN-FD & ISO 26262 ASIL-D Definitions
 * Target OS: QNX Neutrino RTOS 8.0 (x86_64 / aarch64le)
 * License: MIT
 */

#ifndef QNX_BRAKEWIRE_IPC_H
#define QNX_BRAKEWIRE_IPC_H

#include <stdint.h>
#include <stddef.h>
#include <sys/iomsg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* QNX Namespace & PPS Paths */
#define BRAKEWIRE_ATTACH_NAME   "qnx/brakewire"
#define BRAKEWIRE_DEV_PATH      "/dev/name/local/qnx/brakewire"
#define BRAKEWIRE_PPS_PATH      "/pps/adas/aeb"

/* ISO 26262 ASIL-D Hard Real-Time Constants */
#define BRAKEWIRE_LOOP_PERIOD_US      1000U   /* 1 kHz hard-RT control loop */
#define BRAKEWIRE_DEADLINE_US         1000U   /* 1,000 us hard deadline guard */
#define BRAKEWIRE_ACTUATOR_PRIO       63      /* SCHED_FIFO highest safety prio */
#define BRAKEWIRE_SENSOR_RX_PRIO      60      /* SCHED_FIFO CAN-FD ingress prio */
#define BRAKEWIRE_PPS_PUB_PRIO        24      /* Non-critical telemetry publisher */
#define BRAKEWIRE_GRAVITY_MPS2        9.80665

/* CAN-FD Frame Identifiers (Automotive Chassis Bus) */
#define CANFD_ID_RADAR_TARGET         0x120U
#define CANFD_ID_LIDAR_OBJECT         0x124U
#define CANFD_ID_WHEEL_SPEED          0x130U
#define CANFD_ID_BRAKE_CMD            0x140U
#define CANFD_ID_HYDRAULIC_BACKUP     0x144U

/* CAN-FD Validation Flags */
#define CANFD_FLAG_VALID              0x0000U
#define CANFD_FLAG_BAD_CRC16          0x0001U
#define CANFD_FLAG_COUNTER_JUMP       0x0002U
#define CANFD_FLAG_BUS_OFF            0x0004U
#define CANFD_FLAG_SENSOR_DISAGREE    0x0008U
#define CANFD_FLAG_STALE_TIMESTAMP    0x0010U

/* Zero-copy IPC message types (QNX _IO_MAX + custom offsets) */
#define BW_MSG_SENSOR_FUSION          (_IO_MAX + 0x10)
#define BW_MSG_TELEMETRY_QUERY        (_IO_MAX + 0x11)
#define BW_MSG_FAULT_INJECT           (_IO_MAX + 0x12)

/* AEB Actuator Operating Modes */
typedef enum {
    AEB_STATE_STANDBY            = 0,
    AEB_STATE_PREFILL            = 1,
    AEB_STATE_PARTIAL_BRAKE      = 2,
    AEB_STATE_FULL_EMERGENCY     = 3,
    AEB_STATE_HYDRAULIC_FALLBACK = 4
} aeb_state_t;

/* CAN-FD 64-byte payload container with E2E Profile-02 style CRC-16 & rolling counter */
typedef struct __attribute__((packed)) {
    uint32_t can_id;
    uint8_t  dlc;            /* Effective payload length (up to 64 bytes) */
    uint8_t  rolling_cnt;    /* 4-bit monotonic sequence counter [0..15] */
    uint16_t crc16_ccitt;    /* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) */
    uint64_t rx_cycles;      /* ClockCycles() captured at driver ISR */
    uint8_t  payload[64];
} canfd_frame_t;

/* Scatter-gather IOV Part 0: IPC Header */
typedef struct {
    uint16_t type;
    uint16_t subtype;
    uint32_t seq_id;
    uint64_t send_cycles;    /* ClockCycles() before MsgSendv() */
} bw_ipc_hdr_t;

/* Scatter-gather IOV Part 1: Fused Sensor Snapshot (LiDAR + 77 GHz Radar + Wheel IMU) */
typedef struct {
    double   ego_speed_mps;
    double   target_speed_mps;
    double   radar_range_m;
    double   lidar_range_m;
    double   fused_range_m;
    double   road_mu;
    double   slip_ratio;
    uint32_t fault_mask;
    uint8_t  radar_valid;
    uint8_t  lidar_valid;
    uint8_t  plausibility_ok;
    uint8_t  channel_active; /* 0 = Primary Fused, 1 = Redundant LiDAR, 2 = Hydraulic Backup */
} bw_sensor_fusion_t;

/* Scatter-gather Reply: Deterministic Brake-by-Wire Command + RT Timing Proof */
typedef struct {
    uint32_t seq_id;
    uint32_t aeb_state;
    double   ttc_s;
    double   decel_cmd_mps2;
    double   hydraulic_bar;
    double   predicted_clearance_m;
    uint32_t ipc_latency_us;
    uint32_t loop_exec_us;
    uint32_t deadline_misses;
    uint32_t fault_mask;
} bw_brake_reply_t;

/* CRC-16/CCITT-FALSE (ISO 26262 E2E helper) */
static inline uint16_t bw_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif /* QNX_BRAKEWIRE_IPC_H */
