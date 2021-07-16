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
#ifdef ARDUINO
using StreamType = Stream;
#define SONY9PINREMOTE_STREAM_WRITE(data, size) \
    if (size > 0) stream->write(data, size)
#define SONY9PINREMOTE_STREAM_READ stream->read
#elif defined(OF_VERSION_MAJOR)
using StreamType = ofSerial;
#define SONY9PINREMOTE_STREAM_WRITE(data, size) \
    if (size > 0) stream->writeBytes(data, size)
#define SONY9PINREMOTE_STREAM_READ stream->readByte
#endif
#else
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
                // TODO: size check???
                switch (decoder.cmd1()) {
                    case Cmd1::SYSTEM_CONTROL: {
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
                            default: {
                                Serial.println("[Error] Invalid System Control Command 2");
                                break;
                            }
                        }
                    }
                    case Cmd1::SENSE_RETURN: {
                        switch (decoder.cmd2()) {
                            case SenseReturn::STATUS_DATA: {
                                sts = decoder.status_sense();
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
                return true;
            }
        }
        return false;
    }

    bool parse_until(const uint32_t timeout_ms) {
        // TODO: make abstractin (millis())
        const uint32_t begin_ms = millis();
        while (true) {
            if (parse())
                return true;
            if (millis() > begin_ms + timeout_ms)
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

    void cue_up_with_data(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        auto packet = encoder.cue_up_with_data(hours, minutes, seconds, frames);
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

    void timer_1_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        // TODO: Drop or Non-Drop
        auto packet = encoder.timer_1_preset(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void time_code_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        // TODO: Drop or Non-Drop
        auto packet = encoder.time_code_preset(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void user_bit_preset(const uint8_t data1, const uint8_t data2, const uint8_t data3, const uint8_t data4) {
        // TODO: more user-friendly arguments?
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

    void in_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        auto packet = encoder.in_data_preset(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    void out_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        auto packet = encoder.out_data_preset(hours, minutes, seconds, frames);
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

    void preroll_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        auto packet = encoder.preroll_preset(hours, minutes, seconds, frames);
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

    void timer_mode_select(const uint8_t v) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.timer_mode_select(v);
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

    // TODO: implement!
    // =============== 1 - System Control Return ===============

    // TODO: implement!
    // =============== 7 - Sense Return ===============

    // TODO: implement!
    // =============== Status Checker ===============

    // TODO: support more status
    bool is_media_exist() const { return !sts.b_cassette_out; }  // set if no ssd is present
    bool is_remote_enabled() const { return !sts.b_local; }      // set if remote is disabled (local control)
    bool is_disk_available() const { return sts.b_standby; }     // set if a disk is available
    bool is_stopping() const { return sts.b_stop; }
    bool is_rewinding() const { return sts.b_rewind; }
    bool is_forwarding() const { return sts.b_forward; }
    bool is_recoding() const { return sts.b_record; }
    bool is_playing() const { return sts.b_play; }
    bool is_servo_lock() const { return sts.b_servo_lock; }
    bool is_shuttle() const { return sts.b_shuttle; }
    bool is_jog() const { return sts.b_jog; }
    bool is_var() const { return sts.b_var; }
    bool is_reverse() const { return sts.b_direction; }    // clear if playback is forwarding, set if playback is reversing
    bool is_paused() const { return sts.b_still; }         // set if playback is paused, or if in input preview mode
    bool is_auto_mode() const { return sts.b_auto_mode; }  // set if in Auto Mode
    bool is_a_out_set() const { return sts.b_audio_out_set; }
    bool is_a_in_set() const { return sts.b_audio_in_set; }
    bool is_out_set() const { return sts.b_out_set; }
    bool is_in_set() const { return sts.b_in_set; }
    bool is_select_ee() const { return sts.b_select_ee; }  // set if in input preview mode
    bool is_full_ee() const { return sts.b_full_ee; }
    bool is_lamp_still() const { return sts.b_lamp_still; }  // set according to playback speed and direction
    bool is_lamp_fwd() const { return sts.b_lamp_fwd; }
    bool is_lamp_rev() const { return sts.b_lamp_rev; }
    bool is_near_eot() const { return sts.b_near_eot; }  // set if total space left on available SSDs is less than 3 minutes
    bool is_eot() const { return sts.b_eot; }            // set if total space left on available SSDs is less than 30 seconds

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
