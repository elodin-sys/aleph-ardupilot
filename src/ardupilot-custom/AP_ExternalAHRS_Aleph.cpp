/*
   ExternalAHRS backend for Aleph bridge binary UDP protocol.
 */

#include "AP_ExternalAHRS_Aleph.h"

#if AP_EXTERNAL_AHRS_ALEPH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_Aleph::AP_ExternalAHRS_Aleph(AP_ExternalAHRS *frontend_ref, AP_ExternalAHRS::state_t &state_ref) :
    AP_ExternalAHRS_backend(frontend_ref, state_ref)
{
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_Aleph::update_thread, void),
            "AHRS_Aleph", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AlephAHRS: failed thread create");
    }
}

bool AP_ExternalAHRS_Aleph::setup_sockets()
{
    WITH_SEMAPHORE(sem);
    if (sockets_ready) {
        return true;
    }

    sensor_sock.reuseaddress();
    sensor_sock.set_blocking(false);
    if (!sensor_sock.bind("0.0.0.0", SENSOR_PORT)) {
        return false;
    }

    servo_sock.set_blocking(false);
    sockets_ready = true;
    setup_complete = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AlephAHRS: UDP ready :%u", unsigned(SENSOR_PORT));
    return true;
}

void AP_ExternalAHRS_Aleph::update_thread()
{
    // Match other ExternalAHRS backends: don't process external sensor data
    // until scheduler/system init has completed.
    hal.scheduler->delay(1000);
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(1000);

    while (true) {
        if (!setup_sockets()) {
            hal.scheduler->delay(1000);
            continue;
        }

        if (!check_udp()) {
            hal.scheduler->delay_microseconds(1000);
        }
    }
}

bool AP_ExternalAHRS_Aleph::check_udp()
{
    bool got_packet = false;
    uint8_t buf[128];

    while (true) {
        const ssize_t n = sensor_sock.recv(buf, sizeof(buf), 0);
        if (n <= 0) {
            break;
        }
        got_packet = true;

        uint32_t src_addr = 0;
        uint16_t src_port = 0;
        if (sensor_sock.last_recv_address(src_addr, src_port)) {
            WITH_SEMAPHORE(sem);
            peer_addr = src_addr;
            have_peer = true;
        }

        if (n < (ssize_t)sizeof(PacketHeader)) {
            continue;
        }

        PacketHeader hdr {};
        memcpy(&hdr, buf, sizeof(hdr));
        if (hdr.magic != ALEPH_MAGIC) {
            continue;
        }

        switch (hdr.pkt_type) {
        case PKT_IMU: {
            if (n == (ssize_t)sizeof(ImuPacket)) {
                ImuPacket pkt {};
                memcpy(&pkt, buf, sizeof(pkt));
                process_imu(pkt);
            }
            break;
        }
        case PKT_GPS: {
            if (n == (ssize_t)sizeof(GpsPacket)) {
                GpsPacket pkt {};
                memcpy(&pkt, buf, sizeof(pkt));
                process_gps(pkt);
            }
            break;
        }
        case PKT_MAG: {
            if (n == (ssize_t)sizeof(MagPacket)) {
                MagPacket pkt {};
                memcpy(&pkt, buf, sizeof(pkt));
                process_mag(pkt);
            }
            break;
        }
        case PKT_BARO: {
            if (n == (ssize_t)sizeof(BaroPacket)) {
                BaroPacket pkt {};
                memcpy(&pkt, buf, sizeof(pkt));
                process_baro(pkt);
            }
            break;
        }
        default:
            break;
        }

        // Push motor outputs immediately after handling each sensor packet so
        // the bridge always sees actuator feedback, even before scheduler update().
        send_servo_frame();
    }

    return got_packet;
}

void AP_ExternalAHRS_Aleph::process_imu(const ImuPacket &pkt)
{
    last_imu_pkt_ms = AP_HAL::millis();

    AP_ExternalAHRS::ins_data_message_t ins {};
    ins.accel = Vector3f{pkt.accel[0], pkt.accel[1], pkt.accel[2]};
    ins.gyro = Vector3f{pkt.gyro[0], pkt.gyro[1], pkt.gyro[2]};
    ins.temperature = 0.0f;

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = ins.accel;
        state.gyro = ins.gyro;
    }

    AP::ins().handle_external(ins);
}

void AP_ExternalAHRS_Aleph::process_gps(const GpsPacket &pkt)
{
    last_gps_pkt_ms = AP_HAL::millis();

    AP_ExternalAHRS::gps_data_message_t gps {};

    // GPS epoch: Jan 6, 1980 00:00:00 UTC in milliseconds
    static constexpr int64_t GPS_EPOCH_MS = 315964800000LL;
    static constexpr int64_t GPS_LEAP_SECONDS_MS = 18000LL;
    static constexpr int64_t WEEK_MS = 604800000LL;

    if (pkt.unix_epoch_ms > 0) {
        int64_t gps_ms = pkt.unix_epoch_ms - GPS_EPOCH_MS + GPS_LEAP_SECONDS_MS;
        gps.gps_week = uint16_t(gps_ms / WEEK_MS);
        gps.ms_tow = pkt.itow;
    } else {
        gps.gps_week = 0;
        gps.ms_tow = AP_HAL::millis();
    }
    gps.fix_type = AP_GPS_FixType(pkt.fix_type);
    gps.satellites_in_view = pkt.satellites;
    gps.horizontal_pos_accuracy = pkt.h_acc * 1.0e-3f;
    gps.vertical_pos_accuracy = pkt.v_acc * 1.0e-3f;
    gps.horizontal_vel_accuracy = pkt.s_acc * 1.0e-3f;
    gps.hdop = 1.0f;
    gps.vdop = 1.0f;
    gps.latitude = pkt.lat;
    gps.longitude = pkt.lon;
    gps.msl_altitude = pkt.alt_msl / 10;
    gps.ned_vel_north = pkt.vel_ned[0] * 1.0e-3f;
    gps.ned_vel_east = pkt.vel_ned[1] * 1.0e-3f;
    gps.ned_vel_down = pkt.vel_ned[2] * 1.0e-3f;

    {
        WITH_SEMAPHORE(state.sem);
        state.location = Location{pkt.lat, pkt.lon, pkt.alt_msl / 10, Location::AltFrame::ABSOLUTE};
        state.velocity = Vector3f{gps.ned_vel_north, gps.ned_vel_east, gps.ned_vel_down};
        state.have_location = true;
        state.have_velocity = true;
        state.last_location_update_us = AP_HAL::micros();
        if (!state.have_origin && gps.fix_type >= AP_GPS_FixType::FIX_3D) {
            state.origin = state.location;
            state.have_origin = true;
        }
    }

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    }
}

void AP_ExternalAHRS_Aleph::process_mag(const MagPacket &pkt)
{
    last_mag_pkt_ms = AP_HAL::millis();
    AP_ExternalAHRS::mag_data_message_t mag {};
    mag.field = Vector3f{pkt.mag[0], pkt.mag[1], pkt.mag[2]};
    AP::compass().handle_external(mag);
}

void AP_ExternalAHRS_Aleph::process_baro(const BaroPacket &pkt)
{
    last_baro_pkt_ms = AP_HAL::millis();
    AP_ExternalAHRS::baro_data_message_t baro {};
    baro.instance = 0;
    baro.pressure_pa = pkt.pressure_pa;
    baro.temperature = pkt.temperature_c;
    AP::baro().handle_external(baro);
}

void AP_ExternalAHRS_Aleph::send_servo_frame()
{
    uint32_t dst_addr = 0;
    {
        WITH_SEMAPHORE(sem);
        if (!have_peer) {
            return;
        }
        dst_addr = peer_addr;
    }

    ServoPacket pkt {};
    pkt.magic = 18458;
    pkt.frame_rate = SERVO_FRAME_RATE_HZ;
    pkt.frame_count = frame_count++;
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t pwm = 1000;
        IGNORE_RETURN(SRV_Channels::get_output_pwm_chan(i, pwm));
        pkt.pwm[i] = pwm;
    }
    IGNORE_RETURN(servo_sock.sendto(&pkt, sizeof(pkt), dst_addr, SERVO_PORT));
}

void AP_ExternalAHRS_Aleph::update()
{
    if (!setup_complete) {
        return;
    }
    IGNORE_RETURN(check_udp());
    send_servo_frame();
}

bool AP_ExternalAHRS_Aleph::healthy() const
{
    const uint32_t now = AP_HAL::millis();
    return last_imu_pkt_ms != 0 && (now - last_imu_pkt_ms) < 500;
}

bool AP_ExternalAHRS_Aleph::initialised() const
{
    return setup_complete && last_imu_pkt_ms != 0;
}

bool AP_ExternalAHRS_Aleph::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AlephAHRS setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AlephAHRS unhealthy");
        return false;
    }
    if ((AP_HAL::millis() - last_gps_pkt_ms) > 2000) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AlephAHRS GPS timeout");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_Aleph::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
    if (healthy()) {
        status.flags.attitude = true;
        status.flags.horiz_vel = true;
        status.flags.vert_vel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.vert_pos = true;
        status.flags.using_gps = true;
    }
}

#endif // AP_EXTERNAL_AHRS_ALEPH_ENABLED
