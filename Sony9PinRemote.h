#ifndef HT_RS422_SONY9PINREMOTE_H
#define HT_RS422_SONY9PINREMOTE_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <stdint.h>

#if defined(ARDUINO) || defined(OF_VERSION_MAJOR)
#define SONY9PINREMOTE_ENABLE_STREAM
#endif

#include "Sony9PinRemote/Types.h"
#include "Sony9PinRemote/Encoder.h"
#include "Sony9PinRemote/Decoder.h"

namespace sony9pin {

#ifdef SONY9PINREMOTE_ENABLE_STREAM

// Arduino
#ifdef ARDUINO
using StreamType = Stream;
#define SONY9PINREMOTE_STREAM_WRITE(data, size) \
    if (size > 0) stream->write(data, size)
#define SONY9PINREMOTE_STREAM_READ() stream->read()
#define SONY9PINREMOTE_ELAPSED_MILLIS() millis()

// openFrameworks
#elif defined(OF_VERSION_MAJOR)
using StreamType = ofSerial;
#define SONY9PINREMOTE_STREAM_WRITE(data, size) \
    if (size > 0) stream->writeBytes(data, size)
#define SONY9PINREMOTE_STREAM_READ() stream->readByte()
#define SONY9PINREMOTE_ELAPSED_MILLIS() ofGetElapsedTimeMillis()

#endif  // ARDUINO / OF_VERSION_MAIJOR

// Not Supported
#else  // SONY9PINREMOTE_ENABLE_STREAM

#error THIS PLATFORM IS NOT SUPPORTED

#endif  // SONY9PINREMOTE_ENABLE_STREAM

namespace serial {
    static constexpr size_t BAUDRATE {38400};
    static constexpr size_t CONFIG {SERIAL_8O1};
}  // namespace serial

class Controller {
    // reference
    // https://en.wikipedia.org/wiki/9-Pin_Protocol
    // https://www.drastic.tv/support-59/legacysoftwarehardware/72-miscellaneous-legacy/158-vvcr-422-serial-protocol

    StreamType* stream;
    Encoder encoder;
    Decoder decoder;

    uint16_t dev_type {0xFFFF};
    Status sts;
    Errors err;
    size_t err_count {0};

    uint8_t status_start {0};
    uint8_t status_size {10};

    bool b_force_send {false};

public:
    void attach(StreamType& s, const bool force_send = false) {
        b_force_send = force_send;
        stream = &s;
        stream->flush();
        while (stream->available())
            stream->read();
    }

    bool parse() {
        while (stream->available()) {
            if (decoder.feed(stream->read())) {
                // store the data which is useful if it can be referred anytime we want
                switch (decoder.cmd1()) {
                    case Cmd1::SYSTEM_CONTROL_RETURN: {
                        switch (decoder.cmd2()) {
                            case SystemControlReturn::NAK: {
                                err_count++;
                                err = decoder.nak();
                                break;
                            }
                            case SystemControlReturn::DEVICE_TYPE: {
                                dev_type = decoder.device_type();
                                break;
                            }
                        }
                    }
                    case Cmd1::SENSE_RETURN: {
                        if (decoder.cmd2() == SenseReturn::STATUS_DATA) {
                            // decode status based on requested range by `status_sense()`
                            sts = decoder.status_sense(status_start, status_size);
                        }
                        break;
                    }
                    default:
                        break;
                }
                return true;
            }
        }
        return false;
    }

    bool parse_until(const uint32_t timeout_ms) {
        const uint32_t begin_ms = SONY9PINREMOTE_ELAPSED_MILLIS();
        while (true) {
            if (parse())
                return true;
            if (SONY9PINREMOTE_ELAPSED_MILLIS() > begin_ms + timeout_ms)
                return false;
        }
    }

    bool ready() const { return b_force_send ? true : !decoder.busy(); }
    bool available() const { return available(); }

    uint16_t device_type() const { return dev_type; }
    const Status& status() const { return sts; }
    const Errors& errors() const { return err; }
    size_t error_count() const { return err_count; }

    // =============== 0 - System Control ===============

    void local_disable() {
        auto packet = encoder.local_disable();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void device_type_request() {
        auto packet = encoder.device_type_request();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void local_enable() {
        auto packet = encoder.local_enable();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== 2 - Transport Control ===============

    void stop() {
        auto packet = encoder.stop();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void play() {
        auto packet = encoder.play();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void record() {
        auto packet = encoder.record();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void standby_off() {
        auto packet = encoder.standby_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void standby_on() {
        auto packet = encoder.standby_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void eject() {
        auto packet = encoder.eject();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void fast_forward() {
        auto packet = encoder.fast_forward();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void jog_forward(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.jog_forward(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void var_forward(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.var_forward(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void shuttle_forward(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.shuttle_forward(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void frame_step_forward() {
        auto packet = encoder.frame_step_forward();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void fast_reverse() {
        auto packet = encoder.fast_reverse();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void rewind() {
        auto packet = encoder.rewind();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void jog_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.jog_reverse(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void var_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.var_reverse(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void shuttle_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.shuttle_reverse(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void frame_step_reverse() {
        auto packet = encoder.frame_step_reverse();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void preroll() {
        auto packet = encoder.preroll();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void cue_up_with_data(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff) {
        auto packet = encoder.cue_up_with_data(hh, mm, ss, ff);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void sync_play() {
        auto packet = encoder.sync_play();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void prog_speed_play_plus(const uint8_t v) {
        auto packet = encoder.prog_speed_play_plus(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void prog_speed_play_minus(const uint8_t v) {
        auto packet = encoder.prog_speed_play_minus(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void preview() {
        auto packet = encoder.preview();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void review() {
        auto packet = encoder.review();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void auto_edit() {
        auto packet = encoder.auto_edit();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void outpoint_preview() {
        auto packet = encoder.outpoint_preview();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void anti_clog_timer_disable() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.anti_clog_timer_disable();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void anti_clog_timer_enable() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.anti_clog_timer_enable();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void dmc_set_fwd(const uint8_t data1, const uint8_t data2) {
        auto packet = encoder.dmc_set_fwd(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void dmc_set_rev(const uint8_t data1, const uint8_t data2) {
        auto packet = encoder.dmc_set_rev(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void full_ee_off() {
        auto packet = encoder.full_ee_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void full_ee_on() {
        auto packet = encoder.full_ee_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void select_ee_on() {
        auto packet = encoder.select_ee_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void edit_off() {
        auto packet = encoder.edit_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void edit_on() {
        auto packet = encoder.edit_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void freeze_off() {
        auto packet = encoder.freeze_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void freeze_on() {
        auto packet = encoder.freeze_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== 4 - Preset/Select Control ===============

    void timer_1_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff, const bool is_df) {
        auto packet = encoder.timer_1_preset(hh, mm, ss, ff, is_df);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void time_code_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff, const bool is_df) {
        auto packet = encoder.time_code_preset(hh, mm, ss, ff, is_df);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void user_bit_preset(const uint8_t data1, const uint8_t data2, const uint8_t data3, const uint8_t data4) {
        auto packet = encoder.user_bit_preset(data1, data2, data3, data4);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void timer_1_reset() {
        auto packet = encoder.timer_1_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void in_entry() {
        auto packet = encoder.in_entry();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void out_entry() {
        auto packet = encoder.out_entry();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_in_entry() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_in_entry();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_out_entry() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_out_entry();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void in_data_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff) {
        auto packet = encoder.in_data_preset(hh, mm, ss, ff);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void out_data_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff) {
        auto packet = encoder.out_data_preset(hh, mm, ss, ff);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_in_data_preset() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_in_data_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_out_data_preset() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_out_data_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void in_shift_plus() {
        auto packet = encoder.in_shift_plus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void in_shift_minus() {
        auto packet = encoder.in_shift_minus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void out_shift_plus() {
        auto packet = encoder.out_shift_plus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void out_shift_minus() {
        auto packet = encoder.out_shift_minus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_in_shift_plus() {
        auto packet = encoder.audio_in_shift_plus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_in_shift_minus() {
        auto packet = encoder.audio_in_shift_minus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_out_shift_plus() {
        auto packet = encoder.audio_out_shift_plus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_out_shift_minus() {
        auto packet = encoder.audio_out_shift_minus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void in_flag_reset() {
        auto packet = encoder.in_flag_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void out_flag_reset() {
        auto packet = encoder.out_flag_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_in_flag_reset() {
        auto packet = encoder.audio_in_flag_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_out_flag_reset() {
        auto packet = encoder.audio_out_flag_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void in_recall() {
        auto packet = encoder.in_recall();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void out_recall() {
        auto packet = encoder.out_recall();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_in_recall() {
        auto packet = encoder.audio_in_recall();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_out_recall() {
        auto packet = encoder.audio_out_recall();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void lost_lock_reset() {
        auto packet = encoder.lost_lock_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void edit_preset(const uint8_t data1, const uint8_t data2) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.edit_preset(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void preroll_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff) {
        auto packet = encoder.preroll_preset(hh, mm, ss, ff);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void tape_audio_select(const uint8_t v) {
        auto packet = encoder.tape_audio_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void servo_ref_select(const uint8_t v) {
        auto packet = encoder.servo_ref_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void head_select(const uint8_t v) {
        auto packet = encoder.head_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void color_frame_select(const uint8_t v) {
        auto packet = encoder.color_frame_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void timer_mode_select(const TimerMode tm) {
        auto packet = encoder.timer_mode_select(tm);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void input_check(const uint8_t v) {
        auto packet = encoder.input_check(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void edit_field_select(const uint8_t v) {
        auto packet = encoder.edit_field_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void freeze_mode_select(const uint8_t v) {
        auto packet = encoder.freeze_mode_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void record_inhibit() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.record_inhibit();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void auto_mode_off() {
        auto packet = encoder.auto_mode_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void auto_mode_on() {
        auto packet = encoder.auto_mode_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void spot_erase_off() {
        auto packet = encoder.spot_erase_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void spot_erase_on() {
        auto packet = encoder.spot_erase_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_split_off() {
        auto packet = encoder.audio_split_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_split_on() {
        auto packet = encoder.audio_split_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void output_h_phase() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.output_h_phase();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void output_video_phase() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.output_video_phase();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_input_level() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_input_level();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_output_level() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_output_level();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_adv_level() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_adv_level();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_output_phase() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_output_phase();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_adv_output_phase() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_adv_output_phase();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void cross_fade_time_preset() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.cross_fade_time_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void local_key_map() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.local_key_map();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void still_off_time(const uint8_t data1, const uint8_t data2) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.still_off_time(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void stby_off_time(const uint8_t data1, const uint8_t data2) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.stby_off_time(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== 6 - Sense Request ===============

    void tc_gen_sense(const uint8_t data1) {
        auto packet = encoder.tc_gen_sense(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }
    void tc_gen_sense_tc() {
        tc_gen_sense(TcGenData::TC);
    }
    void tc_gen_sense_ub() {
        tc_gen_sense(TcGenData::UB);
    }
    void tc_ub_gen_sense_tc_and_ub() {
        tc_gen_sense(TcGenData::TC_UB);
    }

    void current_time_sense(const uint8_t data1) {
        auto packet = encoder.current_time_sense(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void in_data_sense() {
        auto packet = encoder.in_data_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void out_data_sense() {
        auto packet = encoder.out_data_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_in_data_sense() {
        auto packet = encoder.audio_in_data_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void audio_out_data_sense() {
        auto packet = encoder.audio_out_data_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void status_sense(const uint8_t start = 0, const uint8_t size = 10) {
        status_start = start;
        status_size = size;
        auto packet = encoder.status_sense(start, size);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void extended_vtr_status(const uint8_t data1) {
        auto packet = encoder.extended_vtr_status(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void signal_control_sense(const uint8_t data1, const uint8_t data2) {
        auto packet = encoder.signal_control_sense(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void local_keymap_sense() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.local_keymap_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void head_meter_sense(const uint8_t data1) {
        auto packet = encoder.head_meter_sense(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void remaining_time_sense() {
        auto packet = encoder.remaining_time_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void cmd_speed_sense() {
        auto packet = encoder.cmd_speed_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void edit_preset_sense(const uint8_t data1) {
        auto packet = encoder.edit_preset_sense(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void preroll_time_sense() {
        auto packet = encoder.preroll_time_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void timer_mode_sense() {
        auto packet = encoder.timer_mode_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void record_inhibit_sense() {
        auto packet = encoder.record_inhibit_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void da_inp_emph_sense() {
        auto packet = encoder.da_inp_emph_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void da_pb_emph_sense() {
        auto packet = encoder.da_pb_emph_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void da_samp_freq_sense() {
        auto packet = encoder.da_samp_freq_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void cross_fade_time_sense(const uint8_t data1) {
        auto packet = encoder.cross_fade_time_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== A - BlackMagic Advanced Media Protocol ===============

    void bmd_seek_to_timeline_pos(const uint8_t data1, const uint8_t data2) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.bmd_seek_to_timeline_pos(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void clear_playlist() {
        auto packet = encoder.clear_playlist();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void append_preset() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.append_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void set_playback_loop(const bool b_enable, const uint8_t mode = LoopMode::SINGLE_CLIP) {
        auto packet = encoder.set_playback_loop(b_enable, mode);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void set_stop_mode(const uint8_t stop_mode) {
        auto packet = encoder.set_stop_mode(stop_mode);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void bmd_seek_relative_clip(const int8_t index) {
        auto packet = encoder.bmd_seek_relative_clip(index);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void auto_skip(const int8_t n) {
        auto packet = encoder.auto_skip(n);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void list_next_id() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.list_next_id();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== 1 - System Control Return ===============

    bool ack() const { return decoder.ack(); }
    Errors nak() const { return decoder.nak(); }
    uint16_t device_tpe() const { return decoder.device_type(); }

    // =============== 7 - Sense Return ===============

    // Responses to 61.0A Gen Time Sense
    TimeCodeAndUserBits gen_tc_ub() const { return decoder.gen_tc_ub(); }
    TimeCode gen_tc() const { return decoder.gen_tc(); }
    UserBits gen_ub() const { return decoder.gen_ub(); }

    // Responses to 61.0C Current Time Sense
    TimeCode timer_1() const { return decoder.timer_1(); }
    TimeCode timer_2() const { return decoder.timer_2(); }
    TimeCodeAndUserBits ltc_tc_ub() const { return decoder.ltc_tc_ub(); }
    TimeCode ltc_tc() const { return decoder.ltc_tc(); }
    UserBits ltc_ub() const { return decoder.ltc_ub(); }
    TimeCodeAndUserBits vitc_tc_ub() const { return decoder.vitc_tc_ub(); }
    TimeCode vitc_tc() const { return decoder.vitc_tc(); }
    UserBits vitc_ub() const { return decoder.vitc_ub(); }
    TimeCodeAndUserBits ltc_interpolated_tc_ub() const { return decoder.ltc_interpolated_tc_ub(); }
    TimeCode ltc_interpolated_tc() const { return decoder.ltc_interpolated_tc(); }
    UserBits ltc_interpolated_ub() const { return decoder.ltc_interpolated_ub(); }
    TimeCodeAndUserBits hold_vitc_tc_ub() const { return decoder.hold_vitc_tc_ub(); }
    TimeCode hold_vitc_tc() const { return decoder.hold_vitc_tc(); }
    UserBits hold_vitc_ub() const { return decoder.hold_vitc_ub(); }

    // Responses to other sense requests
    TimeCode in_data() const { return decoder.in_data(); }
    TimeCode out_data() const { return decoder.out_data(); }
    Status status_sense() const { return decoder.status_sense(status_start, status_size); }
    TimeCode preroll_time() const { return decoder.preroll_time(); }
    TimerMode timer_mode() const { return decoder.timer_mode(); }

    // =============== Status Checker ===============

    // byte 0
    bool is_media_exist() const { return !sts.b_cassette_out; }           // set if no ssd is present
    bool is_servo_ref_exist() const { return !sts.b_servo_ref_missing; }  // set if servo reference is absent
    bool is_remote_enabled() const { return !sts.b_local; }               // set if remote is disabled (local control)
    // byte 1
    bool is_disk_available() const { return sts.b_standby; }  // set if a disk is available
    bool is_stopping() const { return sts.b_stop; }           // When the machine is in full stop, this is 1. The thread state depends on the tape/ee and standby settings.
    bool is_ejecting() const { return sts.b_eject; }          // When the tape is ejecting this is 1.
    bool is_fast_reverse() const { return sts.b_rewind; }     // When the machine is in fast reverse this is 1.
    bool is_fast_forward() const { return sts.b_forward; }    // When the machine is in fast forward this is 1.
    bool is_recoding() const { return sts.b_record; }         // This bit goes from 0 to 1 some number of frames after the machine starts recording. For the DVR2000 we measured 5 frames. Others have varying delays on the record status.
    bool is_playing() const { return sts.b_play; }            // This bit goes from 0 to 1 some number of frames after the machine starts playing. For the DVR2000 we measured 5 frames. Others have varying delays on the play status.
    // byte 2
    bool is_servo_locked() const { return sts.b_servo_lock; }  // 1 indicates servos are locked. This is a necessary condition for an edit to occur correctly.
    bool is_tso_mode() const { return sts.b_tso_mode; }        // Bit is 1 in tape speed override: in this mode, audio and video are still locked though speed is off play speed by +/- up to 15%.
    bool is_shuttle() const { return sts.b_shuttle; }
    bool is_jog() const { return sts.b_jog; }
    bool is_var() const { return sts.b_var; }
    bool is_reverse() const { return sts.b_direction; }  // clear if playback is forwarding, set if playback is reversing
    bool is_paused() const { return sts.b_still; }       // set if playback is paused, or if in input preview mode
    bool is_cue_up() const { return sts.b_cue_up; }
    // byte 3
    bool is_auto_mode() const { return sts.b_auto_mode; }  // set if in Auto Mode
    bool is_freezing() const { return sts.b_freeze_on; }
    bool is_cf_mode() const { return sts.b_cf_mode; }
    bool is_audio_out_set() const { return sts.b_audio_out_set; }
    bool is_audio_in_set() const { return sts.b_audio_in_set; }
    bool is_out_set() const { return sts.b_out_set; }
    bool is_in_set() const { return sts.b_in_set; }
    // byte 4
    bool is_select_ee() const { return sts.b_select_ee; }  // set if in input preview mode
    bool is_full_ee() const { return sts.b_full_ee; }
    bool is_edit() const { return sts.b_edit; }
    bool is_review() const { return sts.b_review; }
    bool is_auto_edit() const { return sts.b_auto_edit; }
    bool is_preview() const { return sts.b_preview; }
    bool is_preroll() const { return sts.b_preroll; }
    // byte 5
    bool is_insert() const { return sts.b_insert; }
    bool is_assemble() const { return sts.b_assemble; }
    bool is_video() const { return sts.b_video; }
    bool is_a4() const { return sts.b_a4; }
    bool is_a3() const { return sts.b_a3; }
    bool is_a2() const { return sts.b_a2; }
    bool is_a1() const { return sts.b_a1; }
    // byte 6
    bool is_lamp_still() const { return sts.b_lamp_still; }  // set according to playback speed and direction
    bool is_lamp_fwd() const { return sts.b_lamp_fwd; }
    bool is_lamp_rev() const { return sts.b_lamp_rev; }
    bool is_srch_led_8() const { return sts.b_srch_led_8; }
    bool is_srch_led_4() const { return sts.b_srch_led_4; }
    bool is_srch_led_2() const { return sts.b_srch_led_2; }
    bool is_srch_led_1() const { return sts.b_srch_led_1; }
    // byte 7
    bool is_aud_split() const { return sts.b_aud_split; }
    bool is_syn_act() const { return sts.b_sync_act; }
    bool is_spot_erase() const { return sts.b_spot_erase; }
    bool is_in_out() const { return sts.b_in_out; }
    // byte 8
    bool is_buzzer() const { return sts.b_buzzer; }
    bool is_lost_lock() const { return sts.b_lost_lock; }
    bool is_near_eot() const { return sts.b_near_eot; }  // set if total space left on available SSDs is less than 3 minutes
    bool is_eot() const { return sts.b_eot; }            // set if total space left on available SSDs is less than 30 seconds
    bool is_cf_lock() const { return sts.b_cf_lock; }
    bool is_svo_alarm() const { return sts.b_svo_alarm; }
    bool is_sys_alarm() const { return sts.b_sys_alarm; }
    bool is_rec_inhib() const { return sts.b_rec_inhib; }
    // byte 9
    bool is_fnc_abort() const { return sts.b_fnc_abort; }

    // =============== Utilities ===============

    void print_nak() {
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
};

}  // namespace sony9pin

namespace Sony9PinRemote = sony9pin;
namespace Sony9PinDevice = Sony9PinRemote::DeviceType;
namespace Sony9PinSerial = Sony9PinRemote::serial;

#endif  // HT_RS422_SONY9PINREMOTE_H
