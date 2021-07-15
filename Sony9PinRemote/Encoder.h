#pragma once
#ifndef SONY9PINREMOTE_ENCODER_H
#define SONY9PINREMOTE_ENCODER_H

#include "Types.h"
#include <vector>

namespace sony9pin {

namespace util {
    template <class T>
    struct remove_reference { using type = T; };
    template <class T>
    struct remove_reference<T&> { using type = T; };
    template <class T>
    struct remove_reference<T&&> { using type = T; };

    template <class T>
    constexpr T&& forward(typename remove_reference<T>::type& t) noexcept {
        return static_cast<T&&>(t);
    }
    template <class T>
    constexpr T&& forward(typename remove_reference<T>::type&& t) noexcept {
        return static_cast<T&&>(t);
    }
}  // namespace util

class Encoder {
    uint8_t buffer[MAX_PACKET_SIZE];

public:
    using Packet = std::vector<uint8_t>;

    // 0 - System Control

    // DESCRIPTION:
    // When receiving this command, all local operational functions of the device will be disabled.
    // This includes front panel transport controls, but not front panel setup controls.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    Packet local_disable() {
        Serial.print(__func__);
        return encode(Cmd1::SYSTEM_CONTROL, SystemCtrl::LOCAL_DISABLE);
    }

    // DESCRIPTION:
    // When the device receives the DEVICE TYPE REQUEST command
    // DEVICE TYPE return with 2 bytes data will be returned:
    // REPLY: SystemControlReturn::DEVICE_TYPE
    Packet device_type_request() {
        Serial.print(__func__);
        return encode(Cmd1::SYSTEM_CONTROL, SystemCtrl::DEVICE_TYPE);
    }

    // DESCRIPTION:
    // When receiving this command, the front panel operation of the device will be enabled.
    // When the device is initially powered on, it will be set to the LOCAL ENABLE state.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    Packet local_enable() {
        Serial.print(__func__);
        return encode(Cmd1::SYSTEM_CONTROL, SystemCtrl::LOCAL_ENABLE);
    }

    // 2 - Transport Control

    // DESCRIPTION:
    // Stop the device and pass the device's input to the device's output.
    // Cease all processing of the current material.
    // REPLY: ACK
    Packet stop() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STOP);
    }

    // DESCRIPTION:
    // Plays from the current position at normal play speed for the material.
    // REPLY: ACK
    Packet play() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PLAY);
    }

    // DESCRIPTION:
    // Records from the current position at normal play speed.
    // REPLY: ACK
    Packet record() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::RECORD);
    }

    // DESCRIPTION:
    // The STANDBY OFF command places the device in a stop state, passing all material from the current inputs to the outputs.
    // This should be sent after a stop command to place the device in a fully idle state.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    Packet standby_off() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STANDBY_OFF);
    }

    // DESCRIPTION:
    // Places the device in ready, pause mode.
    // The current material is ready for use and the current material, if possible, is presented at the output.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    Packet standby_on() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STANDBY_ON);
    }

    // DESCRIPTION:
    // If the device supports removable media, remove the media from the device.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    Packet eject() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::EJECT);
    }

    // DESCRIPTION:
    // Moves forward through the material at the highest allowable speed
    // (Usually FORWARD 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: x2 faster
    Packet fast_forward() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FAST_FWD);
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
    Packet jog_forward(const uint8_t data1, const uint8_t data2 = 0) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::JOG_FWD, data1, data2);
    }

    // DESCRIPTION:
    // Move forward through the material, while creating the smoothest possible FORWARD output of the material.
    // This 'smoothing' process may slightly vary the requested speed.
    // REPLY: ACK
    Packet var_forward(const uint8_t data1, const uint8_t data2 = 0) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::VAR_FWD, data1, data2);
    }

    // DESCRIPTION:
    // Move forward through the material, at the exact play speed, regardless of FORWARD results.
    // Usually used for visual searching.
    // REPLY: ACK
    Packet shuttle_forward(const uint8_t data1, const uint8_t data2 = 0) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SHUTTLE_FWD, data1, data2);
    }

    // DESCRIPTION:
    // Move the device's material one frame (actual or logical depending on the FORWARD media) forward and pause.
    // REPLY: ACK
    Packet frame_step_forward() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FRAME_STEP_FWD);
    }

    // DESCRIPTION:
    // Moves backward through the material at the highest allowable speed
    // (Usually REWIND 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: same as rewind()
    Packet fast_reverse() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FAST_REVERSE);
    }

    // DESCRIPTION:
    // Moves backward through the material at the highest allowable speed
    // (Usually REWIND 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: x2 faster
    Packet rewind() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::REWIND);
    }

    // DESCRIPTION:
    // Move backward through the material,
    // usually with varying speeds sent by the REVERSE controller, for fine positioning.
    // REPLY: ACK
    Packet jog_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::JOG_REV, data1, data2);
    }

    // DESCRIPTION:
    // Move backward through the material, while to creating the smoothest possible REVERSE output of the material.
    // This 'smoothing' process may vary the speed slightly from the requested speed.
    // REPLY: ACK
    Packet var_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::VAR_REV, data1, data2);
    }

    // DESCRIPTION:
    // Move backward through the material, at the exact play speed, regardless of REVERSE results.
    // Usually used for visual searching.
    // REPLY: ACK
    Packet shuttle_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SHUTTLE_REV, data1, data2);
    }

    // DESCRIPTION:
    // Move the device's material one frame (actual or logical depending on the REVERSE media) backward and pause.
    // REPLY: ACK
    Packet frame_step_reverse() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FRAME_STEP_REV);
    }

    // DESCRIPTION:
    // Positions the device at the current in point (IN ENTRY)
    // minus the length of the current pre-roll (PRE-ROLL TIME PRESET).
    // REPLY: ACK
    Packet preroll() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PREROLL);
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
    Packet cue_up_with_data(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::CUE_UP_WITH_DATA, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet sync_play() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SYNC_PLAY);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet prog_speed_play_plus(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PROG_SPEED_PLAY_PLUS, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet prog_speed_play_minus(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PROG_SPEED_PLAY_MINUS, v);
    }

    // DESCRIPTION:
    // Play the current edit.
    // Cue the device to the pre-roll point (in point minus pre-roll duration),
    // play the device through the in point to the point two seconds
    // (assuming a two second post-roll) after the out point.
    // REPLY: ACK
    Packet preview() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PREVIEW);
    }

    // DESCRIPTION:
    // Play the last edit.
    // Cue the device to the last pre-roll point (last in point minus pre-roll duration),
    // play the device through the last in point to the point two seconds
    // (assuming a two second post-roll) after the last out point.
    // REPLY: ACK
    Packet review() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::REVIEW);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet auto_edit() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::AUTO_EDIT);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet outpoint_preview() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::OUTPOINT_PREVIEW);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet anti_clog_timer_disable() {
        Serial.print(__func__);
        // return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::ANTI_CLOG_TIMER_DISABLE);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet anti_clog_timer_enable() {
        Serial.print(__func__);
        // return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::ANTI_CLOG_TIMER_ENABLE);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet dmc_set_fwd(const uint8_t data1, const uint8_t data2) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::DMC_SET_FWD, data1, data2);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet dmc_set_rev(const uint8_t data1, const uint8_t data2) {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::DMC_SET_REV, data1, data2);
    }

    // DESCRIPTION:
    // Full 'Edit To Edit' mode off attempts to pass all material from the device to the output.
    // This device has no effect on the current EDIT PRESET, but it does set all channels to the device,
    // unless the device is in an idle state.
    // REPLY: ACK
    Packet full_ee_off() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FULL_EE_OFF);
    }

    // DESCRIPTION:
    // Full 'Edit to Edit' mode on attempts to pass all inputs to the device to the device's output.
    // This device has no effect on the current EDIT PRESET but it does set all channels to the device's inputs.
    // REPLY: ACK
    Packet full_ee_on() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FULL_EE_ON);
    }

    // DESCRIPTION:
    // Sets each EDIT PRESET channel assigned by the DATA-1 of the EDIT PRESET command to the edit to edit mode.
    // All selected channels are passed through from the device's inputs to the device's outputs.
    // To clear the SELECTED EE mode, use the EE OFF or the EDIT OFF command.
    // REPLY: ACK
    Packet select_ee_on() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SELECT_EE_ON);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet edit_off() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::EDIT_OFF);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet edit_on() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::EDIT_ON);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet freeze_off() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FREEZE_OFF);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet freeze_on() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FREEZE_ON);
    }

    // 4 - Preset/Select Control

    // DESCRIPTION:
    // This command presets the device's control (CTL) counter to the value which has been given by
    // the DATA-1 to DATA-4 bytes in the command. For the data format, refer to the CUE UP WITH DATA
    // command. The mode of the Drop Frame (DF) or Non Drop Frame (NDF) is decided according to
    // bit-6 of DATA-1: DATA 1, BIT 6 Drop Frame
    // 0 OFF 1 ON
    // Send: 44 00 00 10 20 01 75 (CTL counter set to 1 hour, 20 minutes, 10 seconds, 0 frames)
    // REPLY: ACK
    Packet timer_1_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        Serial.print(__func__);
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        // TODO: Drop or Non-Drop
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TIMER_1_PRESET, frames, seconds, minutes, hours);
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
    Packet time_code_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        Serial.print(__func__);
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        // TODO: Drop or Non-Drop
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TIME_CODE_PRESET, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // Presets the user bit values in the time code recording of the device, if the device supports
    // user bits, to the value given by DATA-1 to DATA-4 as follows:
    // Send: 44 05 60 63 44 45 95 (Set UB to 06364454)
    // REPLY: ACK
    Packet user_bit_preset(const uint8_t data1, const uint8_t data2, const uint8_t data3, const uint8_t data4) {
        Serial.print(__func__);
        // TODO: more user-friendly arguments?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::USER_BIT_PRESET, data1, data2, data3, data4);
    }

    // DESCRIPTION:
    // Resets the control (CTL) counter to zero.
    // REPLY: ACK
    Packet timer_1_reset() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TIMER_1_RESET);
    }

    // DESCRIPTION:
    // Store the current position of the device as the in point for the next edit.
    // REPLY: ACK
    Packet in_entry() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_ENTRY);
    }

    // DESCRIPTION:
    // Store the current position of the device as the out point for the next edit.
    // REPLY: ACK
    Packet out_entry() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_ENTRY);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_entry() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_ENTRY);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_entry() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_ENTRY);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // Set the in point for the next edit to the time specified by DATA-1 through DATA-4.
    // See the CUE UP WITH DATA command for the data format.
    // Send: 44 14 21 16 25 04 68 (Set in point to 4 hours, 25 minutes, 16 seconds, 21 frames)
    // REPLY: ACK
    Packet in_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        Serial.print(__func__);
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_DATA_PRESET, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // Set the out point for the next edit to the time specified by DATA-1 through DATA-4.
    // See the CUE UP WITH DATA command for the data format.
    // Send: 44 15 05 09 27 04 92 (Set out point to 4 hours, 27 minutes, 9 seconds, 5 frames)
    // REPLY: ACK
    Packet out_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        Serial.print(__func__);
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_DATA_PRESET, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_data_preset() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_DATA_PRESET);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_data_preset() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_DATA_PRESET);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // Adds one frame to the current in point time code value.
    // REPLY: ACK
    Packet in_shift_plus() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_SHIFT_PLUS);
    }

    // DESCRIPTION:
    // Subtracts one frame from the current in point time code value.
    // REPLY: ACK
    Packet in_shift_minus() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_SHIFT_MINUS);
    }

    // DESCRIPTION:
    // Adds one frame to the current out point time code value.
    // REPLY: ACK
    Packet out_shift_plus() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_SHIFT_PLUS);
    }

    // DESCRIPTION:
    // Subtracts one frame from the current out point time code value.
    // REPLY: ACK
    Packet out_shift_minus() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_SHIFT_MINUS);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_shift_plus() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_SHIFT_PLUS);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_shift_minus() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_SHIFT_MINUS);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_shift_plus() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_SHIFT_PLUS);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_shift_minus() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_SHIFT_MINUS);
    }

    // DESCRIPTION:
    // Reset the value of the in point to zero.
    // REPLY: ACK
    Packet in_flag_reset() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_FLAG_RESET);
    }

    // DESCRIPTION:
    // Reset the value of the out point to zero.
    // REPLY: ACK
    Packet out_flag_reset() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_FLAG_RESET);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_flag_reset() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_FLAG_RESET);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_flag_reset() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_FLAG_RESET);
    }

    // DESCRIPTION:
    // Sets the current in point to the last in point that was set. Whenever the in point is changed
    // or used, a backup copy of the time code is saved. This time code can be recovered by the
    // IN RECALL command.
    // REPLY: ACK
    Packet in_recall() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_RECALL);
    }

    // DESCRIPTION:
    // Sets the current out point to the last in point that was set. Whenever the out point is
    // changed or used, a backup copy of the time code is saved. This time code can be recovered
    // by the OUT RECALL command.
    // REPLY: ACK
    Packet out_recall() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_RECALL);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_recall() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_RECALL);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_recall() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_RECALL);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet lost_lock_reset() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::LOST_LOCK_RESET);
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
    Packet edit_preset(const uint8_t data1, const uint8_t data2) {
        Serial.print(__func__);
        // TODO: more user-friendly arguments?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::EDIT_PRESET, data1, data2);
    }

    // DESCRIPTION:
    // Presets the duration of the pre-roll to the length given by the DATA-1 to PRESET DATA-4.
    // For the data format, refer to the CUE UP WITH DATA command.
    // for example:
    // Send: 44 31 00 05 00 00 7A (Set the pre-roll duration to 5 seconds)
    // REPLY: ACK
    Packet preroll_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames) {
        Serial.print(__func__);
        // TODO: need to convert to BCD(Binary Coded Decimal)?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::PREROLL_PRESET, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet tape_audio_select(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TAPE_AUDIO_SELECT, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet servo_ref_select(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SERVO_REF_SELECT, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet head_select(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::HEAD_SELECT, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet color_frame_select(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::COLOR_FRAME_SELECT, v);
    }

    // DESCRIPTION:
    // Selects the default timer to return, by the DATA-1 value as follows:
    // DATA - 1
    //   00 : Time Code
    //   01 : Control(CTL) Counter
    //   FF : device setting dependent.
    // Send : 41 36 11 88(Set the device to time code head)
    // REPLY: ACK
    Packet timer_mode_select(const uint8_t v) {
        Serial.print(__func__);
        // TODO: more user-friendly arguments?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TIMER_MODE_SELECT, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet input_check(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::INPUT_CHECK, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet edit_field_select(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::EDIT_FIELD_SELECT, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet freeze_mode_select(const uint8_t v) {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::FREEZE_MODE_SELECT, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet record_inhibit() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::RECORD_INHIBIT);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // This command switches the device from AUTO mode.
    // REPLY: ACK
    Packet auto_mode_off() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUTO_MODE_OFF);
    }

    // DESCRIPTION:
    // This command switches the device to AUTO mode.
    // REPLY: ACK
    Packet auto_mode_on() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUTO_MODE_ON);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet spot_erase_off() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SPOT_ERASE_OFF);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet spot_erase_on() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SPOT_ERASE_ON);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_split_off() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_SPLIT_OFF);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_split_on() {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_SPLIT_OFF);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet output_h_phase() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUTPUT_H_PHASE);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet output_video_phase() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUTPUT_VIDEO_PHASE);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_input_level() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_INPUT_LEVEL);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_output_level() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUTPUT_LEVEL);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_adv_level() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_ADV_LEVEL);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_output_phase() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUTPUT_PHASE);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet audio_adv_output_phase() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_ADV_OUTPUT_PHASE);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet cross_fade_time_preset() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::CROSS_FADE_TIME_PRESET);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
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
    Packet local_key_map() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::LOCAL_KEY_MAP);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet still_off_time(const uint8_t data1, const uint8_t data2) {
        Serial.print(__func__);
        // TODO: more user-friendly arguments?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::STILL_OFF_TIME, data1, data2);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet stby_off_time(const uint8_t data1, const uint8_t data2) {
        Serial.print(__func__);
        // TODO: more user-friendly arguments?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::STBY_OFF_TIME, data1, data2);
    }

    // 6 - Sense Request

    // DESCRIPTION:
    // Request the type of time code data the device is generating,
    // based on the type SENSE of data required.
    // It will then respond according to the contents of DATA-1.
    // DATA-1 = 01: Request for GEN TC -> GEN TIME DATA 74.08 Respond
    // DATA-1 = 10: Request for GEN UB -> GEN UB DATA 74.09 Respond
    // DATA-1 = 11: Request for GEN TC & UB -> GEN TC & UB DATA 78.08 Respond
    // REPLY: based on DATA-1 as above
    Packet tc_gen_sense(const uint8_t data1) {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::TC_GEN_SENSE, data1);
    }
    Packet tc_gen_sense_tc() {
        return tc_gen_sense(TcGenData::TC);
    }
    Packet tc_gen_sense_ub() {
        return tc_gen_sense(TcGenData::UB);
    }
    Packet tc_ub_gen_sense_tc_and_ub() {
        return tc_gen_sense(TcGenData::TC_UB);
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
    Packet current_time_sense(const uint8_t data1) {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::CURRENT_TIME_SENSE, data1);
    }

    // DESCRIPTION:
    // Requests the current in point.
    // See the CUE UP WITH DATA command for the time code return format.
    // REPLY: IN_DATA
    Packet in_data_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::IN_DATA_SENSE);
    }

    // DESCRIPTION:
    // Requests the current out point.
    // See the CUE UP WITH DATA command for the time code return format.
    // REPLY: OUT_DATA
    Packet out_data_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::OUT_DATA_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: A_IN_DATA
    Packet audio_in_data_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::AUDIO_IN_DATA_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: A_OUT_DATA
    Packet audio_out_data_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::AUDIO_OUT_DATA_SENSE);
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
    Packet status_sense(const uint8_t start = 0, const uint8_t size = 10) {
        Serial.print(__func__);
        const uint8_t v = size | (start << 4);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::STATUS_SENSE, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet extended_vtr_status(const uint8_t data1) {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::EXTENDED_VTR_STATUS, data1);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet signal_control_sense(const uint8_t data1, const uint8_t data2) {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::SIGNAL_CONTROL_SENSE, data1, data2);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet local_keymap_sense() {
        Serial.print(__func__);
        // return encode(Cmd1::SENSE_REQUEST, SenseRequest::LOCAL_KEYMAP_SENSE);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet head_meter_sense(const uint8_t data1) {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::HEAD_METER_SENSE, data1);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet remaining_time_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::REMAINING_TIME_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: SPEED_DATA
    Packet cmd_speed_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::CMD_SPEED_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet edit_preset_sense(const uint8_t data1) {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::EDIT_PRESET_SENSE, data1);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: PREROLL_TIME_DATA
    Packet preroll_time_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::PREROLL_TIME_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: TIMER_MODE_DATA
    Packet timer_mode_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::TIMER_MODE_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet record_inhibit_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::RECORD_INHIBIT_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet da_inp_emph_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::DA_INPUT_EMPHASIS_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet da_pb_emph_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::DA_PLAYBACK_EMPHASIS_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet da_samp_freq_sense() {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::DA_SAMPLING_FREQUENCY_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet cross_fade_time_sense(const uint8_t data1) {
        Serial.print(__func__);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::CROSS_FADE_TIME_SENSE, data1);
    }

    // A - BlackMagic Advanced Media Protocol

    // DESCRIPTION:
    // 16-bit little endian fractional position [0..65535]
    // REPLY: ACK
    Packet bmd_seek_to_timeline_pos(const uint8_t data1, const uint8_t data2) {
        Serial.print(__func__);
        // TODO: more user-friendly arguments?
        return encode(Cmd1::SYSTEM_CONTROL, SystemCtrl::BMD_SEEK_TO_TIMELINE_POS, data1, data2);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet clear_playlist() {
        Serial.print(__func__);
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::CLEAR_PLAYLIST);
    }

    // DESCRIPTION:
    // 2 Bytes for the length N of the clip name
    // N Bytes for each character of the clip name
    // 4 Byte in point timecode (format is FFSSMMHH)
    // 4 Byte out point timecode (format is FFSSMMHH)
    // REPLY: ACK
    Packet append_preset() {
        Serial.print(__func__);
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::APPEND_PRESET);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

    // DESCRIPTION:
    // Bit0 loop mode enable, 0 = false, 1 = true
    // Bit1 is single clip/timeline, 0 = single clip, 1 = timeline
    // REPLY: ACK
    Packet set_playback_loop(const bool b_enable, const uint8_t mode = LoopMode::SINGLE_CLIP) {
        Serial.print(__func__);
        const uint8_t v = (uint8_t)b_enable | ((uint8_t)(mode & 0x01) << 1);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SET_PLAYBACK_LOOP, v);
    }

    // DESCRIPTION:
    // 0 = Off
    // 1 = Freeze on last frame of Timeline (not clip)
    // 2 = Freeze on next clip of Timeline (not clip)
    // 3 = Show black
    // REPLY: ACK
    Packet set_stop_mode(const uint8_t stop_mode) {
        Serial.print(__func__);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SET_STOP_MODE, stop_mode);
    }

    // DESCRIPTION:
    // One-byte signed integer, which is the number of clips to skip (negative for backwards).
    // REPLY: ACK
    Packet bmd_seek_relative_clip(const int8_t index) {
        Serial.print(__func__);
        return encode(Cmd1::BMD_EXTENSION, BmdExtensions::SEEK_RELATIVE_CLIP, index);
    }

    // DESCRIPTION:
    // 8-bit signed number of clips to skip from current clip
    // REPLY: ACK
    Packet auto_skip(const int8_t n) {
        Serial.print(__func__);
        return encode(Cmd1::BMD_ADVANCED_MEDIA_PRTCL, BmdAdvancedMediaProtocol::AUTO_SKIP, (uint8_t)n);
    }

    // DESCRIPTION:
    // when x = 0, single clip request
    // when x = 1, # clips can be specified in the encode data
    // REPLY: IDListing
    Packet list_next_id() {
        Serial.print(__func__);
        // return encode(Cmd1::BMD_ADVANCED_MEDIA_PRTCL, BmdAdvancedMediaProtocol::LIST_NEXT_ID);
        // TODO: NOT IMPLEMENTED
        Serial.println("NOT IMPLEMENTED");
        return Packet();
    }

private:
    Packet encode(Packet& packet, uint8_t& crc) {
        packet.emplace_back(crc);
        Serial.println(crc, HEX);
        return packet;
    }

    template <typename... Args>
    Packet encode(Packet& packet, uint8_t& crc, const uint8_t arg, Args&&... args) {
        packet.emplace_back(arg);
        crc += arg;
        Serial.print(arg, HEX);
        Serial.print(" ");
        return encode(packet, crc, util::forward<Args>(args)...);
    }

    template <typename Cmd2, typename... Args>
    Packet encode(const Cmd1 cmd1, const Cmd2 cmd2, Args&&... args) {
        uint8_t size = sizeof...(args);
        uint8_t header = (uint8_t)cmd1 | (size & 0x0F);
        uint8_t crc = header + (uint8_t)cmd2;
        Packet packet;
        packet.emplace_back(header);
        packet.emplace_back((uint8_t)cmd2);
        Serial.print(" : encode data = ");
        Serial.print(header, HEX);
        Serial.print(" ");
        Serial.print((uint8_t)cmd2, HEX);
        Serial.print(" ");
        return encode(packet, crc, util::forward<Args>(args)...);
    }
};

}  // namespace sony9pin

#endif  // SONY9PINREMOTE_ENCODER_H
