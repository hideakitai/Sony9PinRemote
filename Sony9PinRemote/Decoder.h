#pragma once
#ifndef SONY9PINREMOTE_DECODER_H
#define SONY9PINREMOTE_DECODER_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <stdint.h>

#include "Types.h"

#include "util/ArxTypeTraits/ArxTypeTraits.h"
#include "util/ArxContainer/ArxContainer.h"
#include "util/DebugLog/DebugLog.h"

#ifdef SONY9PINREMOTE_DEBUGLOG_ENABLE
#include "util/DebugLog/DebugLogEnable.h"
#else
#include "util/DebugLog/DebugLogDisable.h"
#endif

namespace sony9pin {

#define SONY9PIN_RESPONSE_CHECK(c1, c2, sz, ret)        \
    {                                                   \
        bool is_success = true;                         \
        if (!available()) {                             \
            LOG_ERROR("No response available");         \
            is_success = false;                         \
        } else {                                        \
            LOG_INFO(                                   \
                DebugLogBase::HEX,                      \
                "Response cmd1:", (uint8_t)cmd1(),      \
                "cmd2:", cmd2(),                        \
                "size:", size());                       \
            if (!(cmd1() == c1) || !(cmd2() == c2)) {   \
                LOG_ERROR(                              \
                    DebugLogBase::HEX,                  \
                    "Packet type mismatch:",            \
                    (uint8_t)cmd1(), "!=", (uint8_t)c1, \
                    "or",                               \
                    cmd2(), "!=", c2);                  \
                is_success = false;                     \
            }                                           \
            if (size() != sz) {                         \
                LOG_ERROR(                              \
                    DebugLogBase::DEC,                  \
                    "Packet size not correct:",         \
                    size(), "should be", sz);           \
                is_success = false;                     \
            }                                           \
        }                                               \
        if (!is_success) return ret;                    \
    }

class Decoder {
    uint8_t buffer[MAX_PACKET_SIZE];
    uint8_t next_size {0};
    uint8_t curr_size {0};

public:
    bool available() const {
        return !empty() && (curr_size == next_size);
    }

    bool busy() const {
        return !empty() && (curr_size < next_size);
    }

    Cmd1 cmd1() const {
        return available() ? (Cmd1)(buffer[0] & (uint8_t)HeaderMask::CMD1) : Cmd1::NA;
    }

    uint8_t cmd2() const {
        return available() ? buffer[1] : 0xFF;
    }

    uint8_t data(const uint8_t i) {
        return available() ? buffer[i + 2] : 0x00;
    }
    const uint8_t* data() const {
        return available() ? buffer + 2 : nullptr;
    }

    uint8_t size() const {
        return available() ? next_size - 3 : 0;
    }

    void clear() {
        memset(buffer, 0, MAX_PACKET_SIZE);
        next_size = 0;
        curr_size = 0;
    }

    bool feed(const uint8_t d) {
        if (curr_size >= next_size) {  // next or unexpected response
            clear();
        }

        if (next_size == 0) {  // header byte
            uint8_t type = d & (uint8_t)HeaderMask::CMD1;
            uint8_t size = d & (uint8_t)HeaderMask::SIZE;

            if ((type == (uint8_t)Cmd1::SYSTEM_CONTROL_RETURN) ||
                (type == (uint8_t)Cmd1::SENSE_RETURN)) {
                next_size = size + 3;  // header + cmd2 + size + checksum
                buffer[curr_size++] = d;
            } else {  // this is not response headr
                LOG_ERROR(DebugLogBase::HEX, "Packet is not response:", type);
                clear();
            }
        } else if (curr_size < next_size) {
            buffer[curr_size++] = d;

            if (curr_size == next_size) {
                uint8_t checksum = 0;
                for (uint8_t i = 0; i < curr_size - 1; ++i)
                    checksum += buffer[i];

                if (d == checksum) {
                    return true;
                } else {
                    LOG_ERROR(DebugLogBase::HEX, "Checksum not matched:", checksum, "should be", d);
                    clear();
                }
            }
        } else {
            LOG_ERROR("Unexpected: won't come here");
            clear();
        }

        return false;
    }

    // =============== 1 - System Control Return ===============

    // 10.01 ACK
    // Returned by the device when a command has been successfully received.
    // Although the ACK indicates that the device has begun processing that command,
    // it does not necessarily mean that the command was completed and the device is
    // in the required state.
    bool ack() const {
        SONY9PIN_RESPONSE_CHECK(Cmd1::SYSTEM_CONTROL_RETURN, SystemControlReturn::ACK, 0, false);
        return true;
    }

    // 11.12.XX NAK
    // When an error has be detected, the device will return the NAK (negative acknowledgement)
    // with the following error status in the third byte of the return.
    // If the reason for the device's failure is unknown, then this byte will be zero.
    //
    // NAK Return Byte 3 (XX):
    // BIT DESCRIPTION
    // 0   Unknown command was received. Check the command in this document and check your hardware.
    // 1   Not used.
    // 2   A checksum error occurs when the last byte of the command is not equal to all the
    //     previous bytes added together and logically 'anded' with FF hex.
    // 3   Not used.
    // 4   A parity error occurs when one or more of the bytes in a command do not have
    //     an odd parity bit transmitted with them. This indicates a serial setup problem,
    //     or hardware/cabling problem.
    // 5   Overrun error indicates that the command has overrun the device's internal command
    //     buffer, and that the command cannot be used. This is an error internal to the device
    //     that should not occur unless more than one command per frame is sent.
    // 6   A framing error indicates a serial setup or hardware problem with the controller.
    // 7   A time out occurs when a command takes too long to be sent between bytes,
    //     or a checksum is sent too late.
    Errors nak() const {
        Errors errs;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SYSTEM_CONTROL_RETURN, SystemControlReturn::NAK, 1, errs);
        errs.b_unknown_cmd = buffer[2] & NakMask::UNKNOWN_CMD;
        errs.b_checksum_error = buffer[2] & NakMask::CHECKSUM_ERROR;
        errs.b_parity_error = buffer[2] & NakMask::PARITY_ERROR;
        errs.b_buffer_overrun = buffer[2] & NakMask::BUFFER_OVERRUN;
        errs.b_framing_error = buffer[2] & NakMask::FRAMING_ERROR;
        errs.b_timeout = buffer[2] & NakMask::TIMEOUT;
        return errs;
    }

    // 12.11 DEVICE TYPE
    // Returns the type of device connected and configured.
    // VTR Model          Data-1 Data-2
    // Drastic VVCR       FE     01
    // JVC BR-S822U       F0     1F
    // Panasonic AG-7750  A0     81
    // Panasonic AU-65    A0     15
    // Pioneer VDR-1000A  F0     15
    // Sony BVW-75        20     25
    // Sony PVW-2800      20     41
    uint16_t device_type() const {
        uint16_t dev_no = 0xFFFF;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SYSTEM_CONTROL_RETURN, SystemControlReturn::DEVICE_TYPE, 2, dev_no);
        dev_no = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
        return dev_no;
    }

    // =============== 7 - Sense Return ===============

    // Responses to 61.0A TC Gen Sense

    // 78.08 GENERATOR TC& mp; UB
    // Returned with the TIME and UB data of the TC generated by the device.DATA DATA -
    // 1 through DATA - 4 are time data and DATA - 4 through DATA - 8 are User Bit data.
    // For the data format, refer to the CUE UP WITH DATA command and U - BIT PRESET.
    TimeCodeAndUserBits gen_tc_ub() const {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::GEN_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    // 74.08 GENERATOR TC DATA
    // Returned with the TC TIME data, generated by the device.
    // For the data format, refer to the CUE UP WITH DATA command.
    TimeCode gen_tc() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::GEN_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 74.09 GENERATOR UB DATA
    // Returned with the UB data of the TC generated by the device.
    // For the data format, refer to the U-BIT PRESET.
    UserBits gen_ub() const {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::GEN_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    // ===== Responses to 61.0C Current Time Sense =====
    //
    // TIME DATA FORMAT
    // Time Data Return Format : https://www.drastic.tv/images/protocol/p_tdata.gif
    //
    // 1. DATA-1/BIT-6:DF FLAG ("1" DF, "0" NDF)
    // When the device receives the CURRENT TIME SENSE 61.0C command,
    // and has been set to the DB mode, the device will be set to 1.
    // When the device receives the TIME CODE PRESET 44.04 command, and the TIME CODE GENERATOR
    // has been set to the DF mode, the device will be set to 1.
    //
    // 2. DATA-1/BIT-7: CF FLAG (1 CF ON, 0 CF OFF)
    // When the device receives the CURRENT TIME SENSE 61.0C command, and has been set to
    // the CG mode, the device will be set to 1.

    /// Generic response for timecode + userbits without packet check
    TimeCodeAndUserBits timecode_userbits() const {
        TimeCodeAndUserBits tcub;
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }
    TimeCode timecode() const {
        TimeCode tc;
        decode_to_timecode(tc);
        return tc;
    }
    UserBits userbits() const {
        UserBits ub;
        decode_to_userbits(ub);
        return ub;
    }

    // 74.00 TIMER-1
    // Returned with the CTL counter data. At this time, the BIT-6 of DATA-1 is DATA set to 1,
    // and 0, when the device CTL counter is set to DF/NDF mode. For the data format,
    // refer to the CUE UP WITH DATA command.
    TimeCodeAndUserBits timer1_tc_ub() const {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::TIMER_1, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }
    TimeCode timer1_tc() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::TIMER_1, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 74.01 TIMER-2
    // Returned with the CTL counter data. At this time, the BIT-6 of DATA-1 is DATA set to 1,
    // and 0, when the device CTL counter is set to DF/NDF mode. For the data format,
    // refer to the CUE UP WITH DATA command.
    TimeCodeAndUserBits timer2_tc_ub() const {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::TIMER_2, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }
    TimeCode timer2_tc() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::TIMER_2, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 78.04 LTC TIME &mp; UB
    // Returned with data that is added to DATA-1 to DATA-4 as LTC TIME DATA UB DATA and DATA-5
    // to DATA-8 as LTC UB DATA. For the data format,
    // refer to the CUE UP WITH DATA and U-BIT PRESET command.
    TimeCodeAndUserBits ltc_tc_ub() const {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    // 74.04 LTC TIME DATA
    // When the LTC TIME DATA device is requested, and the LTC data is read correctly,
    // this command is returned to the controller with four data items.
    // For the data format, refer to the CUE UP WITH DATA command.
    TimeCode ltc_tc() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 74.05 LTC UB DATA
    // Returned with the LTC UB DATA. For the data format, refer to the U-BIT PRESET 44.05 command.
    UserBits ltc_ub() const {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    // 78.06 VITC TIME &mp; UB
    // Returned with data that is added to DATA-1 to DATA-4 as VITC TIME DATA DATA and DATA-5
    // to DATA-8 as VITC UB DATA.
    // For the data format, refer to the CUE UP WITH DATA and U-BIT PRESET command.
    TimeCodeAndUserBits vitc_tc_ub() const {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::VITC_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    // 74.06 VITC TIME DATA
    // Returned with the VITC TIME DATA.
    // For the data format, refer to the CUE UP WITH DATA 24.31 command.
    TimeCode vitc_tc() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::VITC_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 74.07 ITC TIME DATA
    UserBits vitc_ub() const {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::VITC_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    // 78.14 LTC INTERPOLATED TIME + UB
    // When the device's LTC TIME DATA and UB DATA is requested, and the INTERPOLATED LTC data
    // is played back by the device, corrected by the CTL, or read TIME &mp; UB DATA incorrectly,
    // this command will be returned to the controller with data added to DATA-1 to DATA-4
    // as LTC TIME DATA, and DATA-5 to DATA-8 as LTC UB DATA.
    // For the data format, refer to the CUE UP WITH DATA and U-BIT PRESET command.
    TimeCodeAndUserBits ltc_interpolated_tc_ub() const {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_INTERPOLATED_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    // 74.14 LTC INTERPOLATED TIME
    // When the device LTC TIME DATA is requested, if the data of LTC played INTERPOLATED back
    // by the device is corrected by the CTL or it is read incorrectly,
    // this TIME DATA command will be returned to the controller with the LTC TIME data.
    // For the data format, refer to the CUE UP WITH DATA command.
    TimeCode ltc_interpolated_tc() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_INTERPOLATED_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 74.15 LTC TIME UB
    UserBits ltc_interpolated_ub() const {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_INTERPOLATED_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    // 78.16 VITC HOLD TIME
    // When the VITC TIME DATA and VITC UB DATA of the device are &mp; UB DATA requested and
    // read incorrectly, this command will be returned to the controller with data which is
    // added to DATA-1 to DATA-4 as VITC TIME DATA and DATA-5 to DATA-8 as VITC UB DATA.
    // For the data format, refer to the CUE UP WITH DATA and U-BIT PRESET command.
    TimeCodeAndUserBits hold_vitc_tc_ub() const {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::HOLD_VITC_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    // 74.16 VITC HOLD TIME
    // When the device VITC TIME DATA is requested, and is read correctly, this DATA command
    // will be returned to the controller with the VITC TIME DATA.
    // For the data format, refer to the CUE UP WITH DATA command.
    TimeCode hold_vitc_tc() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::HOLD_VITC_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 74.17 VITC HOLD UB
    UserBits hold_vitc_ub() const {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::HOLD_VITC_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    // Responses to other sense requests

    // 74.10 IN DATA
    // Returned with the in point data.
    // For the data format, refer to the CUE UP WITH DATA command.
    TimeCode in_data() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::IN_DATA, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 74.11 OUT DATA
    // Returned with the out point data.
    // For the data format, refer to the CUE UP WITH DATA command.
    TimeCode out_data() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::OUT_DATA, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 61.20 STATUS SENSE
    // Requests the device status. The device will respond with the STATUS DATA
    // 7X .20 command according to the contents of DATA-1 of the controller command.
    //
    // DATA-1
    // MSD (Bit7~4): Indicates the initial DATA No.l of the 7X .20 STATUS DATA to be returned.
    // LSD (Bit3~0): Indicates the number of data bytes in 7X .20 STATUS DATA to be returned.
    // ex. When the DATA-1 is 34.
    // The device will return four bytes starting from the third byte,
    // i.e. DATA No.3 to DATA No.6 of the 74 .20 STATUS DATA
    //
    // STATUS RETURN TABLE
    // figure. Status Return Chart : https://www.drastic.tv/images/protocol/p_stats.gif
    // Note: * indicates a bit not set to 1 on a PLAYER device.
    //
    // DATA-0
    // BIT-0 LOCAL
    // This bit will be set to 1 when the device will only accept commands from the controller, and not the panel.
    // BIT-2 HARDWARE ERROR
    // This bit will be set to 1 when a hardware error occurs in the device.
    // BIT-5 CASSETTE OUT
    // The removable media is not present in the device.
    //
    // DATA-1
    // BIT-0 PLAY
    // This bit will be set to 1 when the device goes into the PLAY, REC or EDIT mode, or the device is in the CAPSTAN OVERRIDE mode.
    // BIT-1 RECORD
    // This bit will be set to 1 when the device goes into the REC mode, or when the DATA-4/BIT-4 : EDIT is set to 1.
    // BIT-2 FAST FORWARD
    // This bit will be set to 1 when the device goes into the FAST FORWARD mode.
    // BIT-3 REWIND
    // This bit will be set to 1 when the device goes into the FAST REVERSE mode.
    // BIT-4 EJECT
    // This bit will be set to 1 when the device ejects its media.
    // BIT-5 STOP
    // This bit will be set to 1 when the device is in stop mode.
    // BIT-6 TENSION RELEASE
    // This bit will be set to 1 when the device is in idle mode.
    // BIT-6 STANDBY ON
    // This bit will be set to 1 when the device is in standby mode.
    //
    // DATA-2
    // BIT-0 CUE UP COMPLETE
    // This bit will be set to 1 when the device completes a CUE UP WITH DATA command and the material is at the requested position.
    // BIT-1 STILL
    // This bit will be set to 1 when the device is stopped and displays the current frame of media.
    // BIT-2 REVERSE/FORWARD
    // This bit will be set to 1 when the device is outputting its material in reverse of the normal order. When moving in the normal direction, it will be 0.
    // BIT-3 VAR MODE
    // This bit will be set to 1 when the device goes into the VAR command mode.
    // BIT-4 JOG MODE
    // This bit will be set to 1 when the device goes into the JOG command mode.
    // BIT-5 SHUTTLE MODE
    // This bit will be set to 1 when the device goes into the SHUTTLE command mode.
    // BIT-7 SERVO LOCK
    // This bit will be set to 1 when the playback or record is servo locked with the input or reference sync.
    //
    // DATA-3
    // BIT-0 IN
    // Set to 1 if an in point has been set.
    // BIT-1 OUT
    // Set to 1 if an out point has been set.
    // BIT-7 AUTO MODE
    // Set to 1 if the device has been placed in AUTO mode.
    //
    // DATA-4
    // BIT-0 PRE-ROLL OR CUE UP COMPLETE
    // This bit will be set to 1 when the device goes into the PRE-ROLL and CUE-UP modes (a PRE-ROLL is also performed in the auto-edit, preview and review modes).
    // BIT-1 REVIEW
    // This bit will be set to 1 when the device is in REVIEW mode.
    // BIT-2 AUTO EDIT
    // This bit will be set to 1 when the device is preform an AUTO EDIT.
    // BIT-3 PREVIEW
    // This bit will be set to 1 when the device is in the PREVIEW mode.
    // BIT-4 EDIT MODE
    // Both the bit and the DATA-1/BIT-1 : REC will be set to 1 when the device is in EDIT mode (between EDIT ON and EDIT OFF or AUTO EDIT between the in and out points).
    // BIT-6 FULL EE ON
    // This bit will be set to 1 when the device is in full edit to edit mode.
    // BIT-7 SELECTED EE
    // This bit will be set to 1 when the device is in 'Selected Edit To Edit' mode.
    //
    // DATA-6 (Not supported on most devices)
    // BIT-4 LAMP REVERSE
    // This bit will be set to 1 when the device is searching backwards.
    // BIT-5 LAMP FORWARD
    // This bit will be set to 1 when the device is searching forwards.
    // BIT-4 LAMP STILL
    // This bit will be set to 1 when the device has finished searching.
    //
    // DATA-7
    // BIT-0 IN-OUT STATUS
    // This bit will be set to 1 in the device PREVIEW or AUTO EDIT mode and the material is running between the in point and out point.
    // BIT-4 SYNC ACTIVE
    // This bit will be set to 1 in the device sensing valid sync on the device's input.
    //
    // DATA-8
    // BIT-0 RECORD INHIBIT
    // If this bit is set to 1, record/edit commands will be ignored.
    // BIT-4 END OF TAPE
    // Set to 1 if the device has reached the end of its media.
    // BIT-5 NEAR END
    // Set to 1 if the device is near the end of its media.
    Status status_sense(const uint8_t start = 0, const uint8_t sz = 10) const {
        Status sts;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::STATUS_DATA, sz, sts);
        for (uint8_t i = start; i < start + sz; ++i) {
            size_t idx = 2 + i - start;
            switch (i) {
                case 0: {  // byte 0
                    sts.b_cassette_out = buffer[idx] & StatusMask::CASSETTE_OUT;
                    sts.b_servo_ref_missing = buffer[idx] & StatusMask::SERVO_REF_MISSING;
                    sts.b_local = buffer[idx] & StatusMask::LOCAL;
                    break;
                }
                case 1: {  // byte 1
                    sts.b_standby = buffer[idx] & StatusMask::STANDBY;
                    sts.b_stop = buffer[idx] & StatusMask::STOP;
                    sts.b_eject = buffer[idx] & StatusMask::EJECT;
                    sts.b_rewind = buffer[idx] & StatusMask::REWIND;
                    sts.b_forward = buffer[idx] & StatusMask::FORWARD;
                    sts.b_record = buffer[idx] & StatusMask::RECORD;
                    sts.b_play = buffer[idx] & StatusMask::PLAY;
                    break;
                }
                case 2: {  // byte 2
                    sts.b_servo_lock = buffer[idx] & StatusMask::SERVO_LOCK;
                    sts.b_tso_mode = buffer[idx] & StatusMask::TSO_MODE;
                    sts.b_shuttle = buffer[idx] & StatusMask::SHUTTLE;
                    sts.b_jog = buffer[idx] & StatusMask::JOG;
                    sts.b_var = buffer[idx] & StatusMask::VAR;
                    sts.b_direction = buffer[idx] & StatusMask::DIRECTION;
                    sts.b_still = buffer[idx] & StatusMask::STILL;
                    sts.b_cue_up = buffer[idx] & StatusMask::CUE_UP;
                    break;
                }
                case 3: {  // byte 3
                    sts.b_auto_mode = buffer[idx] & StatusMask::AUTO_MODE;
                    sts.b_freeze_on = buffer[idx] & StatusMask::FREEZE_ON;
                    sts.b_cf_mode = buffer[idx] & StatusMask::CF_MODE;
                    sts.b_audio_out_set = buffer[idx] & StatusMask::AUDIO_OUT_SET;
                    sts.b_audio_in_set = buffer[idx] & StatusMask::AUDIO_IN_SET;
                    sts.b_out_set = buffer[idx] & StatusMask::OUT_SET;
                    sts.b_in_set = buffer[idx] & StatusMask::IN_SET;
                    break;
                }
                case 4: {  // byte 4
                    sts.b_select_ee = buffer[idx] & StatusMask::SELECT_EE;
                    sts.b_full_ee = buffer[idx] & StatusMask::FULL_EE;
                    sts.b_edit = buffer[idx] & StatusMask::EDIT_SET;
                    sts.b_review = buffer[idx] & StatusMask::REVIEW_SET;
                    sts.b_auto_edit = buffer[idx] & StatusMask::AUTO_EDIT_SET;
                    sts.b_preview = buffer[idx] & StatusMask::PREVIEW_SET;
                    sts.b_preroll = buffer[idx] & StatusMask::PREROLL_SET;
                    break;
                }
                case 5: {  // byte 5
                    sts.b_insert = buffer[idx] & StatusMask::INSERT_SET;
                    sts.b_assemble = buffer[idx] & StatusMask::ASSEMBLE_SET;
                    sts.b_video = buffer[idx] & StatusMask::VIDEO_SET;
                    sts.b_a4 = buffer[idx] & StatusMask::A4_SET;
                    sts.b_a3 = buffer[idx] & StatusMask::A3_SET;
                    sts.b_a2 = buffer[idx] & StatusMask::A2_SET;
                    sts.b_a1 = buffer[idx] & StatusMask::A1_SET;
                    break;
                }
                case 6: {  // byte 6
                    sts.b_lamp_still = buffer[idx] & StatusMask::LAMP_STILL;
                    sts.b_lamp_fwd = buffer[idx] & StatusMask::LAMP_FWD;
                    sts.b_lamp_rev = buffer[idx] & StatusMask::LAMP_REV;
                    sts.b_srch_led_8 = buffer[idx] & StatusMask::SRCH_LED_8;
                    sts.b_srch_led_4 = buffer[idx] & StatusMask::SRCH_LED_4;
                    sts.b_srch_led_2 = buffer[idx] & StatusMask::SRCH_LED_2;
                    sts.b_srch_led_1 = buffer[idx] & StatusMask::SRCH_LED_1;
                    break;
                }
                case 7: {  // byte 8
                    sts.b_aud_split = buffer[idx] & StatusMask::AUD_SPLIT;
                    sts.b_sync_act = buffer[idx] & StatusMask::SYNC_ACT;
                    sts.b_spot_erase = buffer[idx] & StatusMask::SPOT_ERASE;
                    sts.b_in_out = buffer[idx] & StatusMask::IN_OUT;
                    break;
                }
                case 8: {  // byte 8
                    sts.b_buzzer = buffer[idx] & StatusMask::BUZZER;
                    sts.b_lost_lock = buffer[idx] & StatusMask::LOST_LOCK;
                    sts.b_near_eot = buffer[idx] & StatusMask::NEAR_EOT;
                    sts.b_eot = buffer[idx] & StatusMask::EOT;
                    sts.b_cf_lock = buffer[idx] & StatusMask::CF_LOCK;
                    sts.b_svo_alarm = buffer[idx] & StatusMask::SVO_ALARM;
                    sts.b_sys_alarm = buffer[idx] & StatusMask::SYS_ALARM;
                    sts.b_rec_inhib = buffer[idx] & StatusMask::REC_INHIB;
                    break;
                }
                case 9: {  // byte 9
                    sts.b_fnc_abort = buffer[idx] & StatusMask::FNC_ABORT;
                    break;
                }
                default: {
                    break;
                }
            }
        }

        return sts;
    }

    // 60.30 PRE-ROLL TIME
    // This command is used for requesting the current pre-roll duration.
    // For the SENSE return data format, see the CUE UP WITH DATA command.
    // Send: 60 30 90
    // Returns: 74 30 00 05 00 00 A9 (Pre-roll is five seconds)
    TimeCode preroll_time() const {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::PREROLL_TIME, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    // 71.36 TIMER MODE
    // Refer to the TIMER MODE SENSE command.
    TimerMode timer_mode() const {
        TimerMode tm = TimerMode::NA;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::TIMER_MODE_STATUS, 1, tm);
        switch (buffer[2]) {
            case 0x00: tm = TimerMode::TIME_CODE; break;
            case 0x01: tm = TimerMode::CTL_COUNTER; break;
            default: tm = TimerMode::NA; break;
        }
        return tm;
    }

private:
    template <typename T>
    inline auto from_bcd_to_dec(const T& n) const
        -> typename std::enable_if<std::is_integral<T>::value, size_t>::type {
        return n - 6 * (n >> 4);
    }

    bool empty() const {
        return curr_size == 0;
    }

    void decode_to_timecode(TimeCode& tc) const {
        tc.is_cf = buffer[2] & 0b10000000;
        tc.is_df = buffer[2] & 0b01000000;
        tc.frame = from_bcd_to_dec(buffer[2] & 0x3F);
        tc.second = from_bcd_to_dec(buffer[3] & 0x7F);
        tc.minute = from_bcd_to_dec(buffer[4] & 0x7F);
        tc.hour = from_bcd_to_dec(buffer[5] & 0x3F);
    }

    void decode_to_userbits(UserBits& ub, const size_t offset = 0) const {
        for (size_t i = 0; i < 4; ++i)
            ub.bytes[i] = buffer[2 + offset + i];
    }
};

}  // namespace sony9pin

#include "util/DebugLog/DebugLogRestoreState.h"

#endif  // SONY9PINREMOTE_DECODER_H
