/*
   ExternalAHRS backend for Aleph bridge binary UDP protocol.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ALEPH_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_HAL/utility/Socket.h>

class AP_ExternalAHRS_Aleph : public AP_ExternalAHRS_backend {
public:
    AP_ExternalAHRS_Aleph(AP_ExternalAHRS *frontend_ref, AP_ExternalAHRS::state_t &state_ref);

    int8_t get_port(void) const override { return 3; }
    const char* get_name() const override { return "AlephBridge"; }
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &, float &, float &, Vector3f &, float &) const override { return false; }
    void update() override;

protected:
    uint8_t num_gps_sensors(void) const override { return 1; }

private:
    struct __attribute__((packed)) PacketHeader {
        uint16_t magic;
        uint8_t pkt_type;
    };

    struct __attribute__((packed)) ImuPacket {
        uint16_t magic;
        uint8_t pkt_type;
        float gyro[3];
        float accel[3];
    };

    struct __attribute__((packed)) GpsPacket {
        uint16_t magic;
        uint8_t pkt_type;
        int32_t lat;
        int32_t lon;
        int32_t alt_msl;
        int32_t vel_ned[3];
        uint32_t h_acc;
        uint32_t v_acc;
        uint32_t s_acc;
        uint32_t ground_speed;
        uint8_t fix_type;
        uint8_t satellites;
        uint32_t itow;
        int64_t unix_epoch_ms;
    };

    struct __attribute__((packed)) MagPacket {
        uint16_t magic;
        uint8_t pkt_type;
        float mag[3];
    };

    struct __attribute__((packed)) BaroPacket {
        uint16_t magic;
        uint8_t pkt_type;
        float pressure_pa;
        float temperature_c;
    };

    struct __attribute__((packed)) ServoPacket {
        uint16_t magic;
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[16];
    };

    static constexpr uint16_t ALEPH_MAGIC = 0xAE01;
    static constexpr uint8_t PKT_IMU = 0x01;
    static constexpr uint8_t PKT_GPS = 0x02;
    static constexpr uint8_t PKT_MAG = 0x03;
    static constexpr uint8_t PKT_BARO = 0x04;

    static constexpr uint16_t SENSOR_PORT = 9003;
    static constexpr uint16_t SERVO_PORT = 9002;
    static constexpr uint16_t SERVO_FRAME_RATE_HZ = 400;

    void update_thread();
    bool check_udp();
    bool setup_sockets();
    void process_imu(const ImuPacket &pkt);
    void process_gps(const GpsPacket &pkt);
    void process_mag(const MagPacket &pkt);
    void process_baro(const BaroPacket &pkt);
    void send_servo_frame();

    SocketAPM sensor_sock{true};
    SocketAPM servo_sock{true};
    HAL_Semaphore sem;

    bool setup_complete = false;
    bool sockets_ready = false;
    bool have_peer = false;

    uint32_t peer_addr = 0;
    uint32_t frame_count = 0;

    uint32_t last_imu_pkt_ms = 0;
    uint32_t last_gps_pkt_ms = 0;
    uint32_t last_mag_pkt_ms = 0;
    uint32_t last_baro_pkt_ms = 0;
};

#endif // AP_EXTERNAL_AHRS_ALEPH_ENABLED
