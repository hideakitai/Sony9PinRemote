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

    // DESCRIPTION:
    // When receiving this command, all local operational functions of the device will be disabled.
    // This includes front panel transport controls, but not front panel setup controls.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void local_disable() {
        auto packet = encoder.local_disable();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // When the device receives the DEVICE TYPE REQUEST command
    // DEVICE TYPE return with 2 bytes data will be returned:
    // REPLY: SystemControlReturn::DEVICE_TYPE
    void device_type_request() {
        auto packet = encoder.device_type_request();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // When receiving this command, the front panel operation of the device will be enabled.
    // When the device is initially powered on, it will be set to the LOCAL ENABLE state.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void local_enable() {
        auto packet = encoder.local_enable();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== 2 - Transport Control ===============

    // DESCRIPTION:
    // Stop the device and pass the device's input to the device's output.
    // Cease all processing of the current material.
    // REPLY: ACK
    void stop() {
        auto packet = encoder.stop();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Plays from the current position at normal play speed for the material.
    // REPLY: ACK
    void play() {
        auto packet = encoder.play();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Records from the current position at normal play speed.
    // REPLY: ACK
    void record() {
        auto packet = encoder.record();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // The STANDBY OFF command places the device in a stop state, passing all material from the current inputs to the outputs.
    // This should be sent after a stop command to place the device in a fully idle state.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void standby_off() {
        auto packet = encoder.standby_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Places the device in ready, pause mode.
    // The current material is ready for use and the current material, if possible, is presented at the output.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void standby_on() {
        auto packet = encoder.standby_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // If the device supports removable media, remove the media from the device.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void eject() {
        auto packet = encoder.eject();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Moves forward through the material at the highest allowable speed
    // (Usually FORWARD 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: x2 faster
    void fast_forward() {
        auto packet = encoder.fast_forward();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // NOTE: For the commands that follow
    //
    // When receiving one of the following commands (JOG, VAR or SHUTTLE),
    // the device will play forward or backward according to the speed data.
    // When the command byte low nibble is 1 and only DATA-1 is used,
    // the material speed is defined as follows:
    // TAPE SPEED=10(N/32-2)
    // N : SPEED DATA OF DATA-1 (DECIMAL)
    //
    // When the command byte low nibble is 2 and both DATA-1 and DATA-2 are used,
    // the material speed is more precise. In this case the tape speed will is defined as follows:
    // TAPE SPEED= 10(N/32-2) + N'/256 {10(N+1/32-2) -10(N+1/32-2) }
    // N : SPEED DATA OF DATA-1
    // N' : SPEED DATA OF DATA-2
    //
    // for example:
    // Send: 21 11 40 48 (Jog @ play speed)
    // Send: 21 11 20 52 (Jog @ half play speed)
    // Send: 21 13 42 76 (Shuttle @ slightly faster than play speed)
    // Send: 21 11 3E 70 (Jog @ slightly slower than play speed)
    // Send: 21 11 4A 7C (Jog @ two times reverse play speed)
    // Send: 21 13 66 9A (Shuttle @ fifteen times play speed)

    // DESCRIPTION:
    // Move forward through the material,
    // usually with varying speeds sent by the FORWARD controller for fine positioning.
    // REPLY: ACK
    void jog_forward(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.jog_forward(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Move forward through the material, while creating the smoothest possible FORWARD output of the material.
    // This 'smoothing' process may slightly vary the requested speed.
    // REPLY: ACK
    void var_forward(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.var_forward(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Move forward through the material, at the exact play speed, regardless of FORWARD results.
    // Usually used for visual searching.
    // REPLY: ACK
    void shuttle_forward(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.shuttle_forward(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Move the device's material one frame (actual or logical depending on the FORWARD media) forward and pause.
    // REPLY: ACK
    void frame_step_forward() {
        auto packet = encoder.frame_step_forward();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Moves backward through the material at the highest allowable speed
    // (Usually REWIND 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: same as rewind()
    void fast_reverse() {
        auto packet = encoder.fast_reverse();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Moves backward through the material at the highest allowable speed
    // (Usually REWIND 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: x2 faster
    void rewind() {
        auto packet = encoder.rewind();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Move backward through the material,
    // usually with varying speeds sent by the REVERSE controller, for fine positioning.
    // REPLY: ACK
    void jog_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.jog_reverse(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Move backward through the material, while to creating the smoothest possible REVERSE output of the material.
    // This 'smoothing' process may vary the speed slightly from the requested speed.
    // REPLY: ACK
    void var_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.var_reverse(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Move backward through the material, at the exact play speed, regardless of REVERSE results.
    // Usually used for visual searching.
    // REPLY: ACK
    void shuttle_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        auto packet = encoder.shuttle_reverse(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Move the device's material one frame (actual or logical depending on the REVERSE media) backward and pause.
    // REPLY: ACK
    void frame_step_reverse() {
        auto packet = encoder.frame_step_reverse();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Positions the device at the current in point (IN ENTRY)
    // minus the length of the current pre-roll (PRE-ROLL TIME PRESET).
    // REPLY: ACK
    void preroll() {
        auto packet = encoder.preroll();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Cues up the device to the position defined by DATA-1 to DATA-4.
    // Once the DATA device begins cueing, the PRE-ROLL/CUE-UP data bit (Byte 4, Bit 0) will be set on in the STATUS return.
    // Upon successful completion, the CUE-UP COMPLETE data bit (Byte 2, Bit 0) will be set ON and the PRE-ROLL/CUE UP data bit will be set OFF.
    // If the device is unable to seek to that point, then the PRE-ROLL/CUE-UP data bit will be set OFF and the CUE-UP COMPLETE will NOT be set ON.
    // for example:
    // Send: 24 31 13 58 16 02 A7 (Cue to 2 hours, 16 minutes, 58 seconds, 13 frames)
    // Send: 24 31 24 36 52 21 F1 (Cue to 21 hours, 52 minutes, 36 seconds, 24 frames)
    // REPLY: ACK
    void cue_up_with_data(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        auto packet = encoder.cue_up_with_data(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void sync_play() {
        auto packet = encoder.sync_play();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void prog_speed_play_plus(const uint8_t v) {
        auto packet = encoder.prog_speed_play_plus(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void prog_speed_play_minus(const uint8_t v) {
        auto packet = encoder.prog_speed_play_minus(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Play the current edit.
    // Cue the device to the pre-roll point (in point minus pre-roll duration),
    // play the device through the in point to the point two seconds
    // (assuming a two second post-roll) after the out point.
    // REPLY: ACK
    void preview() {
        auto packet = encoder.preview();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Play the last edit.
    // Cue the device to the last pre-roll point (last in point minus pre-roll duration),
    // play the device through the last in point to the point two seconds
    // (assuming a two second post-roll) after the last out point.
    // REPLY: ACK
    void review() {
        auto packet = encoder.review();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void auto_edit() {
        auto packet = encoder.auto_edit();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void outpoint_preview() {
        auto packet = encoder.outpoint_preview();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void anti_clog_timer_disable() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.anti_clog_timer_disable();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void anti_clog_timer_enable() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.anti_clog_timer_enable();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void dmc_set_fwd(const uint8_t data1, const uint8_t data2) {
        auto packet = encoder.dmc_set_fwd(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void dmc_set_rev(const uint8_t data1, const uint8_t data2) {
        auto packet = encoder.dmc_set_rev(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Full 'Edit To Edit' mode off attempts to pass all material from the device to the output.
    // This device has no effect on the current EDIT PRESET, but it does set all channels to the device,
    // unless the device is in an idle state.
    // REPLY: ACK
    void full_ee_off() {
        auto packet = encoder.full_ee_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Full 'Edit to Edit' mode on attempts to pass all inputs to the device to the device's output.
    // This device has no effect on the current EDIT PRESET but it does set all channels to the device's inputs.
    // REPLY: ACK
    void full_ee_on() {
        auto packet = encoder.full_ee_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Sets each EDIT PRESET channel assigned by the DATA-1 of the EDIT PRESET command to the edit to edit mode.
    // All selected channels are passed through from the device's inputs to the device's outputs.
    // To clear the SELECTED EE mode, use the EE OFF or the EDIT OFF command.
    // REPLY: ACK
    void select_ee_on() {
        auto packet = encoder.select_ee_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void edit_off() {
        auto packet = encoder.edit_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void edit_on() {
        auto packet = encoder.edit_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void freeze_off() {
        auto packet = encoder.freeze_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void freeze_on() {
        auto packet = encoder.freeze_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== 4 - Preset/Select Control ===============

    // DESCRIPTION:
    // This command presets the device's control (CTL) counter to the value which has been given by
    // the DATA-1 to DATA-4 bytes in the command. For the data format, refer to the CUE UP WITH DATA
    // command. The mode of the Drop Frame (DF) or Non Drop Frame (NDF) is decided according to
    // bit-6 of DATA-1: DATA 1, BIT 6 Drop Frame
    // 0 OFF 1 ON
    // Send: 44 00 00 10 20 01 75 (CTL counter set to 1 hour, 20 minutes, 10 seconds, 0 frames)
    // REPLY: ACK
    void timer_1_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        // TODO: Drop or Non-Drop
        auto packet = encoder.timer_1_preset(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Presets the value, given by DATA-1 to DATA-4, to the time code start of the PRESET time code
    // generator. This command will only effect devices capable of recording time code independent
    // of the inputs. For the data format, refer to the CUE UP WITH DATA command. The mode of the
    // Drop Frame (DF) or Non Drop Frame (NDF) is decided according to bit-6 of DATA-1:
    // DATA 1, BIT 6 Drop Frame
    // 0 OFF 1 ON
    // Send: 44 04 00 15 30 00 75 (Preset TC set to 30 minutes, 15 seconds, 0 frames)
    // REPLY: ACK
    void time_code_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        // TODO: Drop or Non-Drop
        auto packet = encoder.time_code_preset(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Presets the user bit values in the time code recording of the device, if the device supports
    // user bits, to the value given by DATA-1 to DATA-4 as follows:
    // Send: 44 05 60 63 44 45 95 (Set UB to 06364454)
    // REPLY: ACK
    void user_bit_preset(const uint8_t data1, const uint8_t data2, const uint8_t data3, const uint8_t data4) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.user_bit_preset(data1, data2, data3, data4);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Resets the control (CTL) counter to zero.
    // REPLY: ACK
    void timer_1_reset() {
        auto packet = encoder.timer_1_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Store the current position of the device as the in point for the next edit.
    // REPLY: ACK
    void in_entry() {
        auto packet = encoder.in_entry();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Store the current position of the device as the out point for the next edit.
    // REPLY: ACK
    void out_entry() {
        auto packet = encoder.out_entry();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_in_entry() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_in_entry();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_out_entry() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_out_entry();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Set the in point for the next edit to the time specified by DATA-1 through DATA-4.
    // See the CUE UP WITH DATA command for the data format.
    // Send: 44 14 21 16 25 04 68 (Set in point to 4 hours, 25 minutes, 16 seconds, 21 frames)
    // REPLY: ACK
    void in_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        auto packet = encoder.in_data_preset(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Set the out point for the next edit to the time specified by DATA-1 through DATA-4.
    // See the CUE UP WITH DATA command for the data format.
    // Send: 44 15 05 09 27 04 92 (Set out point to 4 hours, 27 minutes, 9 seconds, 5 frames)
    // REPLY: ACK
    void out_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        auto packet = encoder.out_data_preset(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_in_data_preset() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_in_data_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_out_data_preset() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_out_data_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Adds one frame to the current in point time code value.
    // REPLY: ACK
    void in_shift_plus() {
        auto packet = encoder.in_shift_plus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Subtracts one frame from the current in point time code value.
    // REPLY: ACK
    void in_shift_minus() {
        auto packet = encoder.in_shift_minus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Adds one frame to the current out point time code value.
    // REPLY: ACK
    void out_shift_plus() {
        auto packet = encoder.out_shift_plus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Subtracts one frame from the current out point time code value.
    // REPLY: ACK
    void out_shift_minus() {
        auto packet = encoder.out_shift_minus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_in_shift_plus() {
        auto packet = encoder.audio_in_shift_plus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_in_shift_minus() {
        auto packet = encoder.audio_in_shift_minus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_out_shift_plus() {
        auto packet = encoder.audio_out_shift_plus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_out_shift_minus() {
        auto packet = encoder.audio_out_shift_minus();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Reset the value of the in point to zero.
    // REPLY: ACK
    void in_flag_reset() {
        auto packet = encoder.in_flag_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Reset the value of the out point to zero.
    // REPLY: ACK
    void out_flag_reset() {
        auto packet = encoder.out_flag_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_in_flag_reset() {
        auto packet = encoder.audio_in_flag_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_out_flag_reset() {
        auto packet = encoder.audio_out_flag_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Sets the current in point to the last in point that was set. Whenever the in point is changed
    // or used, a backup copy of the time code is saved. This time code can be recovered by the
    // IN RECALL command.
    // REPLY: ACK
    void in_recall() {
        auto packet = encoder.in_recall();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Sets the current out point to the last in point that was set. Whenever the out point is
    // changed or used, a backup copy of the time code is saved. This time code can be recovered
    // by the OUT RECALL command.
    // REPLY: ACK
    void out_recall() {
        auto packet = encoder.out_recall();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_in_recall() {
        auto packet = encoder.audio_in_recall();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_out_recall() {
        auto packet = encoder.audio_out_recall();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void lost_lock_reset() {
        auto packet = encoder.lost_lock_reset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // This command is used for selecting the edit mode and selection of preset audio and video channels.
    // DATA-1:
    //         Bit 7 Bit 6   Bit 5   Bit 4   Bit 3   Bit 2    Bit 1    Bit 0
    //               Insert Assemble Video           TC       A2 (Cue) A1 (Cue)
    // DATA-2:
    //         Bit 7 Bit 6   Bit 5   Bit 4   Bit 3   Bit 2    Bit 1    Bit 0
    //                                       DA4     DA3      DA2      DA1
    // when the 41.30 command is used, the audio channels are set as per the table in the Edit :
    // Setup menu.When the 42.30 command is used and Bit1 or Bit0 of Data - 1 are "1",
    // the Cue channel is selected.
    // REPLY: ACK
    void edit_preset(const uint8_t data1, const uint8_t data2) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.edit_preset(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Presets the duration of the pre-roll to the length given by the DATA-1 to PRESET DATA-4.
    // For the data format, refer to the CUE UP WITH DATA command.
    // for example:
    // Send: 44 31 00 05 00 00 7A (Set the pre-roll duration to 5 seconds)
    // REPLY: ACK
    void preroll_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        auto packet = encoder.preroll_preset(hours, minutes, seconds, frames);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void tape_audio_select(const uint8_t v) {
        auto packet = encoder.tape_audio_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void servo_ref_select(const uint8_t v) {
        auto packet = encoder.servo_ref_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void head_select(const uint8_t v) {
        auto packet = encoder.head_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void color_frame_select(const uint8_t v) {
        auto packet = encoder.color_frame_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Selects the default timer to return, by the DATA-1 value as follows:
    // DATA - 1
    //   00 : Time Code
    //   01 : Control(CTL) Counter
    //   FF : device setting dependent.
    // Send : 41 36 11 88(Set the device to time code head)
    // REPLY: ACK
    void timer_mode_select(const uint8_t v) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.timer_mode_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void input_check(const uint8_t v) {
        auto packet = encoder.input_check(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void edit_field_select(const uint8_t v) {
        auto packet = encoder.edit_field_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void freeze_mode_select(const uint8_t v) {
        auto packet = encoder.freeze_mode_select(v);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void record_inhibit() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.record_inhibit();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // This command switches the device from AUTO mode.
    // REPLY: ACK
    void auto_mode_off() {
        auto packet = encoder.auto_mode_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // This command switches the device to AUTO mode.
    // REPLY: ACK
    void auto_mode_on() {
        auto packet = encoder.auto_mode_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void spot_erase_off() {
        auto packet = encoder.spot_erase_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void spot_erase_on() {
        auto packet = encoder.spot_erase_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_split_off() {
        auto packet = encoder.audio_split_off();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_split_on() {
        auto packet = encoder.audio_split_on();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void output_h_phase() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.output_h_phase();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void output_video_phase() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.output_video_phase();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_input_level() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_input_level();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_output_level() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_output_level();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_adv_level() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_adv_level();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_output_phase() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_output_phase();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void audio_adv_output_phase() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.audio_adv_output_phase();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void cross_fade_time_preset() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.cross_fade_time_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // When the slave receives the 00.1D Local Enable command, the control panel may be used
    // according to the local key map that was set by this command. When the slave receives
    // the 00.0C Local Disable command all the keys, buttons, and adjustment controls
    // on the control panel are disabled. The Eject button can always be used.
    // If the slave receives the 41.B8 command, the local key map is preset by the block level
    // in accordance with DATA-1. IF it receives the 4X.B8 command ( X > 2 ) The local key map is
    // preset by the Switch level.
    //
    // Block Level switches :
    // -------------------------------------------------------------------------------------------
    //     Bit 7 Bit 6 Bit 5   Bit 4   Bit 3   Bit 2   Bit 1    Bit 0
    //                       Tracking Monitor  Audio   Video  Transport
    //                       Control  Control Control Control  Control
    //
    //     "1" : This function will be enabled when in remote
    //     "0" : This function will be disabled in remote.
    // When DATA - 2 or more are added, control data with two bytes per each block assigned
    // by DATA - 1 are added following DATA - 1.
    //
    // At present the transport switches are defined as follows :
    // -------------------------------------------------------------------------------------------
    //
    //           Bit 7   Bit 6   Bit 5 Bit 4 Bit 3 Bit 2 Bit 1  Bit 0
    // 1st Byte Execute Preroll Search        Rec  Play  Stop  Standby
    // 2nd Byte                                    Var   Jog   Shuttle
    //
    // None of the other blocks have any switches assigned, but rather operate as follows :
    //   Video Control : Video phase and Sync phase can be adjusted on the system menu in remote mode.
    //   Audio Control : Audio levels and output phase can be adjusted on the Audio : DA out menu in remote mode.
    //   Monitor Control : the wfm monitor output selection on the system
    //                   : wfm monitor menu and the montior level adjustments and monitor out selection on the system
    //                   : audio monitor menu can be adjusted in remote mode.
    //   Tracking Control : Tracking adjustments in the system
    //                    : tracking menu can be made in remote mode.
    // REPLY: ACK
    void local_key_map() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.local_key_map();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void still_off_time(const uint8_t data1, const uint8_t data2) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.still_off_time(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void stby_off_time(const uint8_t data1, const uint8_t data2) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.stby_off_time(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== 6 - Sense Request ===============

    // DESCRIPTION:
    // Request the type of time code data the device is generating,
    // based on the type SENSE of data required.
    // It will then respond according to the contents of DATA-1.
    // DATA-1 = 01: Request for GEN TC -> GEN TIME DATA 74.08 Respond
    // DATA-1 = 10: Request for GEN UB -> GEN UB DATA 74.09 Respond
    // DATA-1 = 11: Request for GEN TC & UB -> GEN TC & UB DATA 78.08 Respond
    // REPLY: based on DATA-1 as above
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

    // DESCRIPTION:
    // Requests the time data or user bit. The device will respond according to the SENSE DATA-1 contents, indicated by the CURRENT TIME SENSE RETURN chart.
    // Send: 61 0C 04 11 (Request CTL counter position)
    // Returns : 74 00 01 02 03 04 7E(Return 04 : 03 : 02 : 01)
    // Send : 61 0C 03 11(Request LTC or VITC time code position)
    // Returns : 74 14 10 20 30 24 7E(Return 24 : 30 : 20 : 10 LTC Interpolated w / CTL)
    // Send : 61 0C 02 10(Request LTC or VITC time code position)
    // Returns : 78 16 12 25 00 00 10 56 9A C5(Return 24 : 30 : 20 : 10 LTC Interpolated w / CTL)
    //
    // REPLY: based on CURRENT TIME SENSE RETURN chart
    // https://www.drastic.tv/images/protocol/p_tcrtn.gif
    void current_time_sense(const uint8_t data1) {
        auto packet = encoder.current_time_sense(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Requests the current in point.
    // See the CUE UP WITH DATA command for the time code return format.
    // REPLY: IN_DATA
    void in_data_sense() {
        auto packet = encoder.in_data_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Requests the current out point.
    // See the CUE UP WITH DATA command for the time code return format.
    // REPLY: OUT_DATA
    void out_data_sense() {
        auto packet = encoder.out_data_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: A_IN_DATA
    void audio_in_data_sense() {
        auto packet = encoder.audio_in_data_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: A_OUT_DATA
    void audio_out_data_sense() {
        auto packet = encoder.audio_out_data_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Requests the device status. The device will respond with the STATUS DATA
    // 7X.20 command according to the contents of DATA-1 of the controller command.
    // DATA-1
    // MSD (Bit7~4): Indicates the initial DATA No.l of the 7X.20 STATUS DATA to be returned.
    // LSD (Bit3~0): Indicates the number of data bytes in 7X.20 STATUS DATA to be returned.
    // ex. When the DATA-1 is 34.
    // The device will return four bytes starting from the third byte,
    // i.e. DATA No.3 to DATA No.6 of the 74.20 STATUS DATA
    // REPLY: STATUS_DATA as above
    void status_sense(const uint8_t start = 0, const uint8_t size = 10) {
        auto packet = encoder.status_sense(start, size);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void extended_vtr_status(const uint8_t data1) {
        auto packet = encoder.extended_vtr_status(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void signal_control_sense(const uint8_t data1, const uint8_t data2) {
        auto packet = encoder.signal_control_sense(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void local_keymap_sense() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.local_keymap_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void head_meter_sense(const uint8_t data1) {
        auto packet = encoder.head_meter_sense(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void remaining_time_sense() {
        auto packet = encoder.remaining_time_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: SPEED_DATA
    void cmd_speed_sense() {
        auto packet = encoder.cmd_speed_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void edit_preset_sense(const uint8_t data1) {
        auto packet = encoder.edit_preset_sense(data1);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: PREROLL_TIME_DATA
    void preroll_time_sense() {
        auto packet = encoder.preroll_time_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: TIMER_MODE_DATA
    void timer_mode_sense() {
        auto packet = encoder.timer_mode_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    void record_inhibit_sense() {
        auto packet = encoder.record_inhibit_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    void da_inp_emph_sense() {
        auto packet = encoder.da_inp_emph_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    void da_pb_emph_sense() {
        auto packet = encoder.da_pb_emph_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    void da_samp_freq_sense() {
        auto packet = encoder.da_samp_freq_sense();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    void cross_fade_time_sense(const uint8_t data1) {
        auto packet = encoder.cross_fade_time_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // =============== A - BlackMagic Advanced Media Protocol ===============

    // DESCRIPTION:
    // 16-bit little endian fractional position [0..65535]
    // REPLY: ACK
    void bmd_seek_to_timeline_pos(const uint8_t data1, const uint8_t data2) {
        // TODO: more user-friendly arguments?
        auto packet = encoder.bmd_seek_to_timeline_pos(data1, data2);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void clear_playlist() {
        auto packet = encoder.clear_playlist();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // 2 Bytes for the length N of the clip name
    // N Bytes for each character of the clip name
    // 4 Byte in point timecode (format is FFSSMMHH)
    // 4 Byte out point timecode (format is FFSSMMHH)
    // REPLY: ACK
    void append_preset() {
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.append_preset();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // Bit0 loop mode enable, 0 = false, 1 = true
    // Bit1 is single clip/timeline, 0 = single clip, 1 = timeline
    // REPLY: ACK
    void set_playback_loop(const bool b_enable, const uint8_t mode = LoopMode::SINGLE_CLIP) {
        auto packet = encoder.set_playback_loop(b_enable, mode);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // 0 = Off
    // 1 = Freeze on last frame of Timeline (not clip)
    // 2 = Freeze on next clip of Timeline (not clip)
    // 3 = Show black
    // REPLY: ACK
    void set_stop_mode(const uint8_t stop_mode) {
        auto packet = encoder.set_stop_mode(stop_mode);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // One-byte signed integer, which is the number of clips to skip (negative for backwards).
    // REPLY: ACK
    void bmd_seek_relative_clip(const int8_t index) {
        auto packet = encoder.bmd_seek_relative_clip(index);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // 8-bit signed number of clips to skip from current clip
    // REPLY: ACK
    void auto_skip(const int8_t n) {
        auto packet = encoder.auto_skip(n);
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

    // DESCRIPTION:
    // when x = 0, single clip request
    // when x = 1, # clips can be specified in the send data
    // REPLY: IDListing
    void list_next_id() {
        // send(Cmd1::BMD_ADVANCED_MEDIA_PRTCL, BmdAdvancedMediaProtocol::LIST_NEXT_ID);
        // TODO: NOT IMPLEMENTED
        auto packet = encoder.list_next_id();
        SONY9PINREMOTE_STREAM_WRITE(packet.data(), packet.size());
    }

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

private:
    // void send(uint8_t& crc) {
    //     SONY9PINREMOTE_STREAM_WRITE(crc);
    //     Serial.println(crc, HEX);
    // }

    // template <typename... Args>
    // void send(uint8_t& crc, const uint8_t arg, Args&&... args) {
    //     SONY9PINREMOTE_STREAM_WRITE(arg);
    //     crc += arg;
    //     Serial.print(arg, HEX);
    //     Serial.print(" ");
    //     send(crc, util::forward<Args>(args)...);
    // }

    // template <typename Cmd2, typename... Args>
    // void send(const Cmd1 cmd1, const Cmd2 cmd2, Args&&... args) {
    //     if (!ready()) return;
    //     decoder.clear();
    //     uint8_t size = sizeof...(args);
    //     uint8_t header = (uint8_t)cmd1 | (size & 0x0F);
    //     uint8_t crc = header + (uint8_t)cmd2;
    //     SONY9PINREMOTE_STREAM_WRITE(header);
    //     SONY9PINREMOTE_STREAM_WRITE((uint8_t)cmd2);
    //     Serial.print("send data = ");
    //     Serial.print(header, HEX);
    //     Serial.print(" ");
    //     Serial.print((uint8_t)cmd2, HEX);
    //     Serial.print(" ");
    //     send(crc, util::forward<Args>(args)...);
    // }
};

}  // namespace sony9pin

namespace Sony9PinRemote = sony9pin;
namespace Sony9PinDevice = Sony9PinRemote::DeviceType;
namespace Sony9PinSerial = Sony9PinRemote::serial;

#endif  // HT_RS422_SONY9PINREMOTE_H
