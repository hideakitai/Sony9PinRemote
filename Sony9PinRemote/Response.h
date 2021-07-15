#ifndef HT_RS422_SONY9PINREMOTE_RESPONSE_H
#define HT_RS422_SONY9PINREMOTE_RESPONSE_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <stdint.h>

#include "Types.h"

namespace sony9pin {

class Response {
    // TODO: Do not have buffer inside of decoder
    static constexpr uint8_t MAX_RESPONSE_SIZE {15 + 3};
    uint8_t buffer[MAX_RESPONSE_SIZE];

    uint8_t next_size {0};
    uint8_t curr_size {0};
    uint8_t b_parsing {false};  // TODO: maybe we don't need this

    uint16_t device_no {0};
    size_t err_count {0};

public:
    // TODO: make private
    Status sts;
    Errors err;

    Cmd1 cmd1() const {
        if (!available()) return Cmd1::NA;
        return (Cmd1)(buffer[0] & (uint8_t)HeaderMask::CMD1);
    }
    uint8_t cmd2() const {
        if (!available()) return 0xFF;
        return buffer[1];
    }

    bool available() const {
        return !empty() && (curr_size == next_size) && !b_parsing;
    }
    bool busy() const {
        return b_parsing;
    }
    const Status& status() const {
        return sts;
    }
    const Errors& errors() const {
        return err;
    }
    bool error_count() const {
        return err_count;
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
        for (uint8_t i = 0; i < MAX_RESPONSE_SIZE; ++i) buffer[i] = 0;
        next_size = 0;
        curr_size = 0;
        b_parsing = false;
    }

    bool feed(const uint8_t d) {
        if (curr_size >= next_size) {  // next or unexpected response
            clear();
            b_parsing = true;
        }

        if (next_size == 0) {  // header byte
            uint8_t type = d & (uint8_t)HeaderMask::CMD1;
            uint8_t size = d & (uint8_t)HeaderMask::SIZE;

            if ((type == (uint8_t)Cmd1::SYSTEM_CONTROL_RETURN) ||
                (type == (uint8_t)Cmd1::SENSE_RETURN)) {
                next_size = size + 3;  // header + cmd2 + size + checksum
                buffer[curr_size++] = d;
            } else {  // this is not response headr
                Serial.println("response type error !");
                clear();
            }
        } else if (curr_size < next_size) {
            buffer[curr_size++] = d;

            if (curr_size == next_size) {
                uint8_t checksum = 0;
                for (uint8_t i = 0; i < curr_size - 1; ++i)
                    checksum += buffer[i];

                if (d == checksum) {
                    b_parsing = false;
                    decode_response();
                    return true;
                } else {
                    clear();
                    Serial.println("ERROR: checksum check failed!");
                }
            }
        } else {
            Serial.println("ERROR: won't come here!");
            clear();
        }

        return false;
    }

    void print_nak() {
        Serial.print("NAK received ! err = ");
        Serial.println(buffer[2], BIN);
        if (err.b_unknown_cmd) Serial.println("Unknown Command");
        if (err.b_checksum_error) Serial.println("Checksum Error");
        if (err.b_parity_error) Serial.println("Parity Error");
        if (err.b_buffer_overrun) Serial.println("Buffer Overrun");
        if (err.b_framing_error) Serial.println("Framing Error");
        if (err.b_timeout) Serial.println("Timeout");
    }

    void print_status() {
        Serial.println("<Remote Status>");
        Serial.println("=================");
        Serial.print("Cassette Out : ");
        Serial.println(sts.b_cassette_out);
        Serial.print("Local        : ");
        Serial.println(sts.b_local);
        Serial.println("-----------------");
        Serial.print("Standby      : ");
        Serial.println(sts.b_standby);
        Serial.print("Stop         : ");
        Serial.println(sts.b_stop);
        Serial.print("Rewind       : ");
        Serial.println(sts.b_rewind);
        Serial.print("Forward      : ");
        Serial.println(sts.b_forward);
        Serial.print("Record       : ");
        Serial.println(sts.b_record);
        Serial.print("Play         : ");
        Serial.println(sts.b_play);
        Serial.println("-----------------");
        Serial.print("Servo Lock   : ");
        Serial.println(sts.b_servo_lock);
        Serial.print("Shuttle      : ");
        Serial.println(sts.b_shuttle);
        Serial.print("Jog          : ");
        Serial.println(sts.b_jog);
        Serial.print("Var          : ");
        Serial.println(sts.b_var);
        Serial.print("Direction    : ");
        Serial.println(sts.b_direction);
        Serial.print("Still        : ");
        Serial.println(sts.b_still);
        Serial.println("-----------------");
        Serial.print("Auto Mode    : ");
        Serial.println(sts.b_auto_mode);
        Serial.print("Aout Set     : ");
        Serial.println(sts.b_audio_out_set);
        Serial.print("Ain Set      : ");
        Serial.println(sts.b_audio_in_set);
        Serial.print("Out Set      : ");
        Serial.println(sts.b_out_set);
        Serial.print("In Set       : ");
        Serial.println(sts.b_in_set);
        Serial.println("-----------------");
        Serial.print("Select EE    : ");
        Serial.println(sts.b_select_ee);
        Serial.print("Full EE      : ");
        Serial.println(sts.b_full_ee);
        Serial.println("-----------------");
        Serial.print("Lamp Still   : ");
        Serial.println(sts.b_lamp_still);
        Serial.print("Lamp Fwd     : ");
        Serial.println(sts.b_lamp_fwd);
        Serial.print("Lamp Rev     : ");
        Serial.println(sts.b_lamp_rev);
        Serial.println("-----------------");
        Serial.print("Near EOT     : ");
        Serial.println(sts.b_near_eot);
        Serial.print("EOT          : ");
        Serial.println(sts.b_eot);
        Serial.println("=================");
    }

#define SONY9PIN_RESPONSE_CHECK(c1, c2, sz, ret)           \
    if (!available()) {                                    \
        Serial.println("[Error] No response available");   \
        return ret;                                        \
    }                                                      \
    if (!(cmd1() == c1) || !(cmd2() == c2)) {              \
        Serial.println("[Error] Packet type mismatch");    \
        return ret;                                        \
    }                                                      \
    if (size() != sz) {                                    \
        Serial.println("[Error] Packet size not correct"); \
        return ret;                                        \
    }

    bool ack() const {
        SONY9PIN_RESPONSE_CHECK(Cmd1::SYSTEM_CONTROL_RETURN, SystemControlReturn::ACK, 0, false);
        return true;
    }

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

    uint16_t device_type() const {
        uint16_t dev_no = 0xFFFF;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SYSTEM_CONTROL_RETURN, SystemControlReturn::DEVICE_TYPE, 2, dev_no);
        dev_no = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
        return dev_no;
    }

    struct TimeCode {
        uint8_t frame;
        uint8_t second;
        uint8_t minute;
        uint8_t hour;
        bool is_cf;
        bool is_df;
    };
    union UserBits {
        uint8_t bytes[4];
        uint32_t i;
    };
    struct TimeCodeAndUserBits {
        TimeCode tc;
        UserBits ub;
    };

    // Responses to 61.0A TC Gen Sense

    TimeCodeAndUserBits get_gen_tc_ub() {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::GEN_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    TimeCode get_gen_tc() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::GEN_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    UserBits get_gen_ub() {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::GEN_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    // Responses to 61.0C Current Time Sense

    TimeCode get_timer_1() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::TIMER_1, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    TimeCode get_timer_2() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::TIMER_2, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    TimeCodeAndUserBits get_ltc_tc_ub() {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    TimeCode get_ltc_tc() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    UserBits get_ltc_ub() {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    TimeCodeAndUserBits get_vitc_tc_ub() {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::VITC_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    TimeCode get_vitc_tc() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::VITC_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    UserBits get_vitc_ub() {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::VITC_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    TimeCodeAndUserBits get_ltc_interpolated_tc_ub() {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_INTERPOLATED_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    TimeCode get_ltc_interpolated_tc() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_INTERPOLATED_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    UserBits get_ltc_interpolated_ub() {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::LTC_INTERPOLATED_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    TimeCodeAndUserBits get_hold_vitc_tc_ub() {
        TimeCodeAndUserBits tcub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::HOLD_VITC_TC_UB, 8, tcub);
        decode_to_timecode(tcub.tc);
        decode_to_userbits(tcub.ub);
        return tcub;
    }

    TimeCode get_hold_vitc_tc() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::HOLD_VITC_TC, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    UserBits get_hold_vitc_ub() {
        UserBits ub;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::HOLD_VITC_UB, 4, ub);
        decode_to_userbits(ub);
        return ub;
    }

    // Responses to other sense requests

    TimeCode get_in_data() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::IN_DATA, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    TimeCode get_out_data() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::OUT_DATA, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    Status get_status(const uint8_t start = 0, const uint8_t sz = 10) {
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

        print_status();
        return sts;
    }

    TimeCode get_preroll_time() {
        TimeCode tc;
        SONY9PIN_RESPONSE_CHECK(Cmd1::SENSE_RETURN, SenseReturn::PREROLL_TIME, 4, tc);
        decode_to_timecode(tc);
        return tc;
    }

    TimerMode get_timer_mode() {
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
    bool empty() const {
        return curr_size == 0;
    }

    // bool has_system_control_return() const {
    //     if (!available()) return false;
    //     return cmd1() == Cmd1::SYSTEM_CONTROL_RETURN;
    // }

    // bool has_sense_return() const {
    //     if (!available()) return false;
    //     return cmd1() == Cmd1::SENSE_RETURN;
    // }

    // bool has_ack() const {
    //     if (!available()) return false;
    //     return has_system_control_return() && (cmd2() == SystemControlReturn::ACK);
    // }
    // bool has_nak() const {
    //     if (!available()) return false;
    //     return has_system_control_return() && (cmd2() == SystemControlReturn::NAK);
    // }
    // bool has_device_type() const {
    //     if (!available()) return false;
    //     return has_system_control_return() && (cmd2() == SystemControlReturn::DEVICE_TYPE);
    // }

    // bool has_status() const {
    //     if (!available()) return false;
    //     return has_sense_return() && (cmd2() == SenseReturn::STATUS_DATA);
    // }

    void decode_response() {
        // store the data which is useful if it can be referred anytime we want
        // TODO: size check???
        switch (cmd1()) {
            case Cmd1::SYSTEM_CONTROL: {
                switch (cmd2()) {
                    case SystemControlReturn::NAK: {
                        err_count++;
                        err = nak();
                        print_nak();
                        break;
                    }
                    case SystemControlReturn::DEVICE_TYPE: {
                        device_no = device_type();
                        break;
                    }
                    default: {
                        Serial.println("[Error] Invalid System Control Command 2");
                        break;
                    }
                }
            }
            case Cmd1::SENSE_RETURN: {
                switch (cmd2()) {
                    case SenseReturn::STATUS_DATA: {
                        sts = get_status();
                        break;
                    }
                    default: {
                        break;
                    }
                }
            }
            default: {
                break;
            }
        }
    }

    void decode_to_timecode(TimeCode& tc) {
        tc.is_cf = buffer[2] & 0b10000000;
        tc.is_df = buffer[2] & 0b01000000;
        tc.frame = (((buffer[2] & 0x30) >> 4) * 10) | (buffer[2] & 0x0F);
        tc.second = (((buffer[3] & 0x70) >> 4) * 10) | (buffer[3] & 0x0F);
        tc.minute = (((buffer[4] & 0x70) >> 4) * 10) | (buffer[4] & 0x0F);
        tc.hour = (((buffer[5] & 0x30) >> 4) * 10) | (buffer[5] & 0x0F);
    }

    void decode_to_userbits(UserBits& ub, const size_t offset = 0) {
        for (size_t i = 0; i < 4; ++i)
            ub.bytes[i] = buffer[2 + offset + i];
    }

};  // namespace sony9pin

}  // namespace sony9pin
#endif  // HT_RS422_SONY9PINREMOTE_RESPONSE_H
