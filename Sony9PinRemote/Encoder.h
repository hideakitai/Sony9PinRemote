#pragma once
#ifndef SONY9PINREMOTE_ENCODER_H
#define SONY9PINREMOTE_ENCODER_H

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
public:
#if ARX_HAVE_LIBSTDCPLUSPLUS >= 201103L  // Have libstdc++11
    using Packet = std::vector<uint8_t>;
#else   // Do not have libstdc++11
    using Packet = arx::vector<uint8_t, MAX_PACKET_SIZE>;
#endif  // Do not have libstdc++11

    // =============== 0 - System Control ===============

    // 00.0C LOCAL DISABLE
    // When receiving this command, all local operational functions of the device will be disabled.
    // This includes front panel transport controls, but not front panel setup controls.
    // Send: 00 0C 0C
    // Returns: 10 01 11
    // HyperDeck NOTE: NOT SUPPORTED
    Packet local_disable() {
        LOG_INFO(" ");
        return encode(Cmd1::SYSTEM_CONTROL, SystemCtrl::LOCAL_DISABLE);
    }

    // 00.11 DEVICE TYPE
    // When the device receives the DEVICE TYPE REQUEST command
    // REQUEST the DEVICE TYPE return with 2 bytes data will be returned:
    // Return: 12.11 DEVICE TYPE
    // Send: 00 11 11
    // Returns: 12 11 FE 01 22
    Packet device_type_request() {
        LOG_INFO(" ");
        return encode(Cmd1::SYSTEM_CONTROL, SystemCtrl::DEVICE_TYPE);
    }

    // 00.1D LOCAL ENABLE
    // When receiving this command, the front panel operation of the device will be enabled.
    // When the device is initially powered on, it will be set to the LOCAL ENABLE state.
    // Send: 00 1D 1D
    // Returns: 10 01 11
    // HyperDeck NOTE: NOT SUPPORTED
    Packet local_enable() {
        LOG_INFO(" ");
        return encode(Cmd1::SYSTEM_CONTROL, SystemCtrl::LOCAL_ENABLE);
    }

    // =============== 2 - Transport Control ===============

    // 20.00 STOP
    // Stop the device and pass the device's input to the device's output.
    // Cease all processing of the current material.
    // Send: 20 00 20
    // Returns: 10 01 11
    Packet stop() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STOP);
    }

    // 20.01 PLAY
    // Plays from the current position at normal play speed for the material.
    // Send: 20 01 21
    // Returns: 10 01 11
    Packet play() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PLAY);
    }

    // 20.02 RECORD
    // Records from the current position at normal play speed.
    // Send: 20 02 22
    // Returns: 10 01 11
    Packet record() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::RECORD);
    }

    // 20.04 STANDBY OFF
    // The STANDBY OFF command places the device in a stop state, passing all material from the current inputs to the outputs. This should be sent after a stop command to place the device in a fully idle state.
    // Send: 20 04 24
    // Returns: 10 01 11
    Packet standby_off() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STANDBY_OFF);
    }

    // 20.05 STANDBY ON
    // Places the device in ready, pause mode. The current material is ready for use and the current material, if possible, is presented at the output.
    // Send: 20 05 25
    // Returns: 10 01 11
    Packet standby_on() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STANDBY_ON);
    }

    // 20.0F EJECT
    // If the device supports removable media, remove the media from the device.
    // Send: 20 0F 2F
    // Returns: 10 01 11
    Packet eject() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::EJECT);
    }

    // 20.10 FAST FORWARD
    // Moves forward through the material at the highest allowable speed
    // (Usually FORWARD 32 to 90 times play speed).
    // Send: 20 10 30
    // Returns: 10 01 11
    Packet fast_forward() {
        LOG_INFO(" ");
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

    // 2X.11 JOG FORWARD
    // Move forward through the material,
    // usually with varying speeds sent by the FORWARD controller for fine positioning.
    // Send: 21 11 40 48 (Jog @ play speed)
    Packet jog_forward(const uint8_t data1, const uint8_t data2 = 0) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::JOG_FWD, data1, data2);
    }

    // 2X.12 VAR FORWARD *
    // Move forward through the material, while creating the smoothest possible FORWARD
    // output of the material. This 'smoothing' process may slightly vary the requested speed.
    // Send: 21 11 20 52 (Jog @ half play speed)
    Packet var_forward(const uint8_t data1, const uint8_t data2 = 0) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::VAR_FWD, data1, data2);
    }

    // 2X.13 SHUTTLE FORWARD
    // Move forward through the material, at the exact play speed, regardless of FORWARD results.
    // Usually used for visual searching.
    // Send: 21 13 42 76 (Shuttle @ slightly faster than play speed)
    Packet shuttle_forward(const uint8_t data1, const uint8_t data2 = 0) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SHUTTLE_FWD, data1, data2);
    }

    // 20.14 FRAME STEP *
    // Move the device's material one frame (actual or logical depending on the FORWARD media)
    // forward and pause.
    // Send: 20 14 34
    // Returns: 10 01 11
    Packet frame_step_forward() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FRAME_STEP_FWD);
    }

    // 20.20 FAST REVERSE
    // Moves backward through the material at the highest allowable speed
    // (Usually REWIND 32 to 90 times play speed).
    // Send: 20 20 40
    // Returns: 10 01 11
    Packet fast_reverse() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FAST_REVERSE);
    }

    // 20.20 Rewind
    // same as FAST REVERSE
    Packet rewind() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::REWIND);
    }

    // 2X.11 JOG REVERSE
    // Move backward through the material, usually with varying speeds sent
    // by the REVERSE controller, for fine positioning.
    // Send: 21 11 3E 70 (Jog @ slightly slower than play speed)
    Packet jog_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::JOG_REV, data1, data2);
    }

    // 2X.12 VAR REVERSE *
    // Move backward through the material, while to creating the smoothest possible REVERSE output
    // of the material. This 'smoothing' process may vary the speed slightly from the requested speed.
    // Send: 21 11 4A 7C (Jog @ two times reverse play speed)
    Packet var_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::VAR_REV, data1, data2);
    }

    // 2X.13 SHUTTLE REVERSE
    // Move backward through the material, at the exact play speed, regardless of REVERSE results.
    // Usually used for visual searching.
    // Send: 21 13 66 9A (Shuttle @ fifteen times play speed)20.30 PRE-ROLL *
    // Positions the device at the current in point (IN ENTRY) minus the length of the current
    // pre-roll (PRE-ROLL TIME PRESET).
    // Send: 20 30 50
    // Returns: 10 01 11
    Packet shuttle_reverse(const uint8_t data1, const uint8_t data2 = 0) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SHUTTLE_REV, data1, data2);
    }

    // 20.24 Frame Step Reverse
    // Move the device's material one frame (actual or logical depending on the REVERSE media)
    // backward and pause.
    // REPLY: ACK
    Packet frame_step_reverse() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FRAME_STEP_REV);
    }

    // 20.30 Preroll
    // Positions the device at the current in point (IN ENTRY)
    // minus the length of the current pre-roll (PRE-ROLL TIME PRESET).
    // REPLY: ACK
    Packet preroll() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PREROLL);
    }

    // 24.31 CUE UP WITH DATA *
    // Cues up the device to the position defined by DATA-1 to DATA-4. Once the DATA device
    // begins cueing, the PRE-ROLL/CUE-UP data bit (Byte 4, Bit 0) will be set on in
    // the STATUS return. Upon successful completion, the CUE-UP COMPLETE data bit (Byte 2, Bit 0)
    // will be set ON and the PRE-ROLL/CUE UP data bit will be set OFF. If the device is unable to
    // seek to that point, then the PRE-ROLL/CUE-UP data bit will be set OFF and the CUE-UP
    // COMPLETE will NOT be set ON.
    // Cue up with data format
    // figure. Time Data Format : https://www.drastic.tv/images/protocol/p_cwdata.gif
    // Send: 24 13 58 16 02 A7 (Cue to 2 hours, 16 minutes, 58 seconds, 13 frames)
    // Returns: 10 01 11
    // Send: 24 24 36 52 21 F1 (Cue to 21 hours, 52 minutes, 36 seconds, 24 frames)
    // Returns: 10 01 11
    Packet cue_up_with_data(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::CUE_UP_WITH_DATA, ff, ss, mm, hh);
    }

    // 20.34 Sync Play
    // UNKNOWN
    // REPLY: ACK
    Packet sync_play() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SYNC_PLAY);
    }

    // 21.38 Prog Speed Play +
    // UNKNOWN
    // REPLY: ACK
    Packet prog_speed_play_plus(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PROG_SPEED_PLAY_PLUS, v);
    }

    // 21.39 Prog Speed Play -
    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet prog_speed_play_minus(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PROG_SPEED_PLAY_MINUS, v);
    }

    // 20.40 PREVIEW *
    // Play the current edit. Cue the device to the pre-roll point
    // (in point minus pre-roll duration), play the device through the in point to the point
    // two seconds (assuming a two second post-roll) after the out point.
    // Send: 20 40 60
    // Returns: 10 01 11
    Packet preview() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PREVIEW);
    }

    // 20.41 REVIEW *
    // Play the last edit.
    // Cue the device to the last pre-roll point (last in point minus pre-roll duration),
    // play the device through the last in point to the point two seconds
    // (assuming a two second post-roll) after the last out point.
    // Send: 20 41 61
    // Returns: 10 01 11
    Packet review() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::REVIEW);
    }

    // 20.42 AUTO EDIT *
    // Pre-roll the device to the pre-roll point (in point minus the pre-roll).
    // Play the device from the pre-roll point to the in point. At the in point, begin recording
    // the selected material (as per EDIT PRESET) from the in point to the out point.
    // Upon reaching the out point, play the material after the out point for two seconds.
    // Send: 20 42 62
    // Returns: 10 01 11
    Packet auto_edit() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::AUTO_EDIT);
    }

    // 20.43 Outpoint Preview
    // UNKNOWN
    // REPLY: ACK
    Packet outpoint_preview() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::OUTPOINT_PREVIEW);
    }

    // 2X.54 Anti-Clog Timer Disable
    // UNKNOWN
    // REPLY: ACK
    Packet anti_clog_timer_disable() {
        LOG_INFO(" ");
        // return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::ANTI_CLOG_TIMER_DISABLE);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 2X.55 Anti-Clog Timer Enable
    // UNKNOWN
    // REPLY: ACK
    Packet anti_clog_timer_enable() {
        LOG_INFO(" ");
        // return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::ANTI_CLOG_TIMER_ENABLE);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 2X.5C DMC Set Forward
    // UNKNOWN
    // REPLY: ACK
    Packet dmc_set_fwd(const uint8_t data1, const uint8_t data2) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::DMC_SET_FWD, data1, data2);
    }

    // 2X.5D DMC Set Reverse
    // UNKNOWN
    // REPLY: ACK
    Packet dmc_set_rev(const uint8_t data1, const uint8_t data2) {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::DMC_SET_REV, data1, data2);
    }

    // 20.60 FULL EE OFF
    // Full 'Edit To Edit' mode off attempts to pass all material from the device to the output.
    // This device has no effect on the current EDIT PRESET, but it does set all channels to
    // the device, unless the device is in an idle state.
    // Send: 20 60 80
    // Returns: 10 01 11
    Packet full_ee_off() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FULL_EE_OFF);
    }

    // 20.61 FULL EE ON
    // Full 'Edit to Edit' mode on attempts to pass all inputs to the device to the device's output.
    // This device has no effect on the current EDIT PRESET but it does set all channels to
    // the device's inputs.
    // Send: 20 61 81
    // Returns: 10 01 11
    Packet full_ee_on() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FULL_EE_ON);
    }

    // 20.63 SELECT EE ON
    // Sets each EDIT PRESET channel assigned by the DATA-1 of the EDIT PRESET command to the edit
    // to edit mode. All selected channels are passed through from the device's inputs to
    // the device's outputs. To clear the SELECTED EE mode, use the EE OFF or the EDIT OFF command.
    // Send: 20 63 83
    // Returns: 10 01 11
    Packet select_ee_on() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SELECT_EE_ON);
    }

    // 20.64 EDIT OFF
    // An EDIT OFF completes an edit in progress, or resets the channels on a preview started
    // with a SELECT EE ON. The edit is initiated by an EDIT ON command that begins recording
    // on the device's channels that were selected with the EDIT PRESET command.
    // The EDIT OFF command stops the recording exactly 8 (default setting for EDIT OFF) frames
    // after the command is received.
    // Send: 20 64 84
    // Returns: 10 01 11
    Packet edit_off() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::EDIT_OFF);
    }

    // 20.65 EDIT ON
    // The EDIT ON command initiates the recording of an edit setup by EDIT PRESET,
    // exactly 8 (default setting for EDIT ON) frames after the command is received.
    // Normally, to create an edit, all devices in the edit are pre-rolled, each device is
    // placed in play mode, and the speed of each device is adjusted until all devices are
    // running synchronously. Eight frames before the recording device's in point,
    // and EDIT ON command is sent to begin recording. Eight frames before the device's out point,
    // an EDIT OFF command is sent to end the edit.
    // Send: 20 65 84
    // Returns: 10 01 11
    Packet edit_on() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::EDIT_ON);
    }

    // 20.6A Freeze Off
    // UNKNOWN
    // REPLY: ACK
    Packet freeze_off() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FREEZE_OFF);
    }

    // 20.6B Freeze On
    // UNKNOWN
    // REPLY: ACK
    Packet freeze_on() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FREEZE_ON);
    }

    // =============== 4 - Preset/Select Control ===============

    // 44.00 TIMER-1 PRESET
    // This command presets the device's control (CTL) counter to the value which has been given by
    // the DATA-1 to DATA-4 bytes in the command. For the data format, refer to the CUE UP WITH DATA
    // command. The mode of the Drop Frame (DF) or Non Drop Frame (NDF) is decided according to
    // bit-6 of DATA-1: DATA 1, BIT 6 Drop Frame
    // 0 OFF 1 ON
    // Send: 44 00 00 10 20 01 75 (CTL counter set to 1 hour, 20 minutes, 10 seconds, 0 frames)
    // Returns: 10 01 11
    Packet timer1_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff, const bool is_df) {
        LOG_INFO(" ");
        const uint8_t h = from_dec_to_bcd(hh);
        const uint8_t m = from_dec_to_bcd(mm);
        const uint8_t s = from_dec_to_bcd(ss);
        uint8_t f = from_dec_to_bcd(ff);
        f |= (uint8_t)is_df << 6;  // 0: non-drop, 1: drop
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TIMER_1_PRESET, f, s, m, h);
    }

    // 44.04 TIME CODE PRESET
    // Presets the value, given by DATA-1 to DATA-4, to the time code start of the PRESET time code
    // generator. This command will only effect devices capable of recording time code independent
    // of the inputs. For the data format, refer to the CUE UP WITH DATA command. The mode of the
    // Drop Frame (DF) or Non Drop Frame (NDF) is decided according to bit-6 of DATA-1:
    // DATA 1, BIT 6 Drop Frame
    // 0 OFF 1 ON
    // Send: 44 04 00 15 30 00 75 (Preset TC set to 30 minutes, 15 seconds, 0 frames)
    // Returns: 10 01 11
    Packet time_code_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff, const bool is_df) {
        LOG_INFO(" ");
        const uint8_t h = from_dec_to_bcd(hh);
        const uint8_t m = from_dec_to_bcd(mm);
        const uint8_t s = from_dec_to_bcd(ss);
        uint8_t f = from_dec_to_bcd(ff);
        f |= (uint8_t)is_df << 6;  // 0: non-drop, 1: drop
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TIME_CODE_PRESET, f, s, m, h);
    }

    // 44.05 USER-BIT PRESET
    // Presets the user bit values in the time code recording of the device, if the device supports
    // user bits, to the value given by DATA-1 to DATA-4 as follows:
    // Send: 44 05 60 63 44 45 95 (Set UB to 06364454)
    // Returns: 10 01 11
    Packet user_bit_preset(const uint8_t data1, const uint8_t data2, const uint8_t data3, const uint8_t data4) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::USER_BIT_PRESET, data1, data2, data3, data4);
    }

    // 40.08 TIMER-1 RESET
    // Resets the control (CTL) counter to zero.
    // Send: 40 08 48
    // Returns: 10 01 11
    Packet timer1_reset() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TIMER_1_RESET);
    }

    // 40.10 IN ENTRY
    // Store the current position of the device as the in point for the next edit.
    // Send: 40 10 50
    // Returns: 10 01 11
    Packet in_entry() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_ENTRY);
    }

    // 40.11 OUT ENTRY
    // Store the current position of the device as the out point for the next edit.
    // Send: 40 11 51
    // Returns: 10 01 11
    Packet out_entry() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_ENTRY);
    }

    // 40.12 Audio In Entry
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_entry() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_ENTRY);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 40.13 Audio Out Entry
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_entry() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_ENTRY);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 44.14 IN PRESET
    // Set the in point for the next edit to the time specified by DATA-1 through DATA-4.
    // See the CUE UP WITH DATA command for the data format.
    // Send: 44 14 21 16 25 04 68 (Set in point to 4 hours, 25 minutes, 16 seconds, 21 frames)
    // Returns: 10 01 11
    Packet in_data_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff) {
        LOG_INFO(" ");
        const uint8_t h = from_dec_to_bcd(hh);
        const uint8_t m = from_dec_to_bcd(mm);
        const uint8_t s = from_dec_to_bcd(ss);
        const uint8_t f = from_dec_to_bcd(ff);
        // f |= (uint8_t)is_df << 6;  // 0: non-drop, 1: drop
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_DATA_PRESET, f, s, m, h);
    }

    // 44.15 OUT PRESET
    // Set the out point for the next edit to the time specified by DATA-1 through DATA-4.
    // See the CUE UP WITH DATA command for the data format.
    // Send: 44 15 05 09 27 04 92 (Set out point to 4 hours, 27 minutes, 9 seconds, 5 frames)
    // Returns: 10 01 11
    Packet out_data_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff) {
        LOG_INFO(" ");
        const uint8_t h = from_dec_to_bcd(hh);
        const uint8_t m = from_dec_to_bcd(mm);
        const uint8_t s = from_dec_to_bcd(ss);
        const uint8_t f = from_dec_to_bcd(ff);
        // f |= (uint8_t)is_df << 6;  // 0: non-drop, 1: drop
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_DATA_PRESET, f, s, m, h);
    }

    // 4?.16 Audio In Data Preset
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_data_preset() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_DATA_PRESET);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4?.17 Audio Out Data Preset
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_data_preset() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_DATA_PRESET);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 40.18 IN SHIFT + *
    // Adds one frame to the current in point time code value.
    // Send: 40 18 58
    // Returns: 10 01 11
    Packet in_shift_plus() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_SHIFT_PLUS);
    }

    // 40.19 IN SHIFT - *
    // Subtracts one frame from the current in point time code value.
    // Send: 40 19 59
    // Returns: 10 01 11
    Packet in_shift_minus() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_SHIFT_MINUS);
    }

    // 40.1A OUT SHIFT + *
    // Adds one frame to the current out point time code value.
    // Send: 40 1A 5A
    // Returns: 10 01 11
    Packet out_shift_plus() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_SHIFT_PLUS);
    }

    // 40.1B OUT SHIFT - *
    // Subtracts one frame from the current out point time code value.
    // Send: 40 1B 5B
    // Returns: 10 01 11
    Packet out_shift_minus() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_SHIFT_MINUS);
    }

    // 40.1C Audio In Shift +
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_shift_plus() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_SHIFT_PLUS);
    }

    // 40.1D Audio In Shift -
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_shift_minus() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_SHIFT_MINUS);
    }

    // 40.1E Audio Out Shift +
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_shift_plus() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_SHIFT_PLUS);
    }

    // 40.1F Audio Out Shift -
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_shift_minus() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_SHIFT_MINUS);
    }

    // 40.20 IN RESET *
    // Reset the value of the in point to zero.
    // Send: 40 20 60
    // Returns: 10 01 11
    Packet in_flag_reset() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_FLAG_RESET);
    }

    // 40.21 OUT RESET *
    // Reset the value of the out point to zero.
    // Send: 40 21 61
    // Returns: 10 01 11
    Packet out_flag_reset() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_FLAG_RESET);
    }

    // 40.22 Audio In Flag Reset
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_flag_reset() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_FLAG_RESET);
    }

    // 40.23 Audio In Flag Reset
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_flag_reset() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_FLAG_RESET);
    }

    // 40.24 IN RECALL
    // Sets the current in point to the last in point that was set. Whenever the in point is changed
    // or used, a backup copy of the time code is saved. This time code can be recovered by the
    // IN RECALL command.
    // Send: 40 24 64
    // Returns: 10 01 11
    Packet in_recall() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_RECALL);
    }

    // 40.25 OUT RECALL
    // Sets the current out point to the last in point that was set. Whenever the out point is
    // changed or used, a backup copy of the time code is saved. This time code can be recovered
    // by the OUT RECALL command.
    // Send: 40 25 65
    // Returns: 10 01 11
    Packet out_recall() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_RECALL);
    }

    // 40.26 Audio In Recall
    // UNKNOWN
    // REPLY: ACK
    Packet audio_in_recall() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_IN_RECALL);
    }

    // 40.27 Audio Out Recall
    // UNKNOWN
    // REPLY: ACK
    Packet audio_out_recall() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUT_RECALL);
    }

    // 40.2D Lost Lock Reset
    // UNKNOWN
    // REPLY: ACK
    Packet lost_lock_reset() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::LOST_LOCK_RESET);
    }

    // 41.30 EDIT PRESET
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
    // Send: 41 30 62 D3 (Insert Video and Audio-2 in next edit)
    // Returns: 10 01 11
    Packet edit_preset(const uint8_t data1, const uint8_t data2) {
        LOG_INFO(" ");
        // TODO: more user-friendly arguments?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::EDIT_PRESET, data1, data2);
    }

    // 44.31 PRE-ROLL TIME
    // Presets the duration of the pre-roll to the length given by the DATA-1 to PRESET DATA-4.
    // For the data format, refer to the CUE UP WITH DATA command.
    // Send: 44 31 00 05 00 00 7A (Set the pre-roll duration to 5 seconds)
    // Returns: 10 01 11
    Packet preroll_preset(const uint8_t hh, const uint8_t mm, const uint8_t ss, const uint8_t ff) {
        LOG_INFO(" ");
        const uint8_t h = from_dec_to_bcd(hh);
        const uint8_t m = from_dec_to_bcd(mm);
        const uint8_t s = from_dec_to_bcd(ss);
        const uint8_t f = from_dec_to_bcd(ff);
        // f |= (uint8_t)is_df << 6;  // 0: non-drop, 1: drop
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::PREROLL_PRESET, f, s, m, h);
    }

    // 41.32 Tape/Audio Select
    // UNKNOWN
    // REPLY: ACK
    Packet tape_audio_select(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TAPE_AUDIO_SELECT, v);
    }

    // 41.33 Servo Ref Select
    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    Packet servo_ref_select(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SERVO_REF_SELECT, v);
    }

    // 41.34 Head Select
    // UNKNOWN
    // REPLY: ACK
    Packet head_select(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::HEAD_SELECT, v);
    }

    // 41.35 Color Frame Select
    // UNKNOWN
    // REPLY: ACK
    Packet color_frame_select(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::COLOR_FRAME_SELECT, v);
    }

    // 41.36 TIMER MODE SELECT
    // Selects the default timer to return, by the DATA-1 value as follows:
    // DATA - 1
    //   00 : Time Code
    //   01 : Control(CTL) Counter
    //   FF : device setting dependent.
    // Send: 41 36 11 88 (Set the device to time code head)
    // Returns: 10 01 11
    Packet timer_mode_select(const TimerMode tm) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::TIMER_MODE_SELECT, (uint8_t)tm);
    }

    // 41.37 Input Check
    // UNKNOWN
    // REPLY: ACK
    Packet input_check(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::INPUT_CHECK, v);
    }

    // 41.3A Edit Field Select
    // UNKNOWN
    // REPLY: ACK
    Packet edit_field_select(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::EDIT_FIELD_SELECT, v);
    }

    // 41.3B Freeze Mode Select
    // UNKNOWN
    // REPLY: ACK
    Packet freeze_mode_select(const uint8_t v) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::FREEZE_MODE_SELECT, v);
    }

    // 4X.3E Record Inhibit
    // UNKNOWN
    // REPLY: ACK
    Packet record_inhibit() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::RECORD_INHIBIT);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 40.40 AUTO MODE OFF
    // This command switches the device from AUTO mode.
    // Send: 40 40 80
    // Returns: 10 01 11
    Packet auto_mode_off() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUTO_MODE_OFF);
    }

    // 40.41 AUTO MODE ON
    // This command switches the device to AUTO mode.
    // Send: 40 41 81
    // Returns: 10 01 11
    Packet auto_mode_on() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUTO_MODE_ON);
    }

    // 40.42 Spot Erase Off
    // UNKNOWN
    // REPLY: ACK
    Packet spot_erase_off() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SPOT_ERASE_OFF);
    }

    // 40.43 Spot Erase On
    // UNKNOWN
    // REPLY: ACK
    Packet spot_erase_on() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SPOT_ERASE_ON);
    }

    // 40.44 Audio Split Off
    // UNKNOWN
    // REPLY: ACK
    Packet audio_split_off() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_SPLIT_OFF);
    }

    // 40.45 Audio Split On
    // UNKNOWN
    // REPLY: ACK
    Packet audio_split_on() {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_SPLIT_OFF);
    }

    // 4X.98 Output H Phase
    // UNKNOWN
    // REPLY: ACK
    Packet output_h_phase() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUTPUT_H_PHASE);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4X.9B Output Video Phase
    // UNKNOWN
    // REPLY: ACK
    Packet output_video_phase() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUTPUT_VIDEO_PHASE);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4X.A0 Audio Input Level
    // UNKNOWN
    // REPLY: ACK
    Packet audio_input_level() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_INPUT_LEVEL);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4X.A1 Audio Output Level
    // UNKNOWN
    // REPLY: ACK
    Packet audio_output_level() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUTPUT_LEVEL);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4X.A2 Audio Adv Level
    // UNKNOWN
    // REPLY: ACK
    Packet audio_adv_level() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_ADV_LEVEL);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4X.A8 Audio Output Phase
    // UNKNOWN
    // REPLY: ACK
    Packet audio_output_phase() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_OUTPUT_PHASE);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4X.A9 Audio Adv Output PHase
    // UNKNOWN
    // REPLY: ACK
    Packet audio_adv_output_phase() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUDIO_ADV_OUTPUT_PHASE);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4X.AA Cross Fade Time Preset
    // UNKNOWN
    // REPLY: ACK
    Packet cross_fade_time_preset() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::CROSS_FADE_TIME_PRESET);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 4X.B8 Local Key Map
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
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::LOCAL_KEY_MAP);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 42.F8 Still Off Time
    // UNKNOWN
    // REPLY: ACK
    Packet still_off_time(const uint8_t data1, const uint8_t data2) {
        LOG_INFO(" ");
        // TODO: more user-friendly arguments?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::STILL_OFF_TIME, data1, data2);
    }

    // 42.FA Stby Off Time
    // UNKNOWN
    // REPLY: ACK
    Packet stby_off_time(const uint8_t data1, const uint8_t data2) {
        LOG_INFO(" ");
        // TODO: more user-friendly arguments?
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::STBY_OFF_TIME, data1, data2);
    }

    // =============== 6 - Sense Request ===============

    // 61.0A TC GENERATOR DATA
    // Request the type of time code data the device is generating,
    // based on the type SENSE of data required.
    // It will then respond according to the contents of DATA-1.
    // DATA-1 = 01: Request for GEN TC -> GEN TIME DATA 74.08 Respond
    // DATA-1 = 10: Request for GEN UB -> GEN UB DATA 74.09 Respond
    // DATA-1 = 11: Request for GEN TC & UB -> GEN TC & UB DATA 78.08 Respond
    // Send: 61 0A 11 7C
    // Returns: 10 01 11
    Packet tc_gen_sense(const uint8_t data1) {
        LOG_INFO(" ");
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

    // 61.0C CURRENT TIME
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
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::CURRENT_TIME_SENSE, data1);
    }

    // 60.10 IN DATA SENSE
    // Requests the current in point.
    // See the CUE UP WITH DATA command for the time code return format.
    // Send: 60 10 70
    // Returns: 74 10 10 20 30 24 7E (Return in point of 24:30:20:10)
    Packet in_data_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::IN_DATA_SENSE);
    }

    // 60.11 OUT DATA SENSE
    // Requests the current out point.
    // See the CUE UP WITH DATA command for the time code return format.
    // Send: 60 11 71
    // Returns: 74 11 24 30 20 10 7F (Return in point of 10:20:30:24)
    Packet out_data_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::OUT_DATA_SENSE);
    }

    // 60.12 Audio In Data Sense
    // UNKNOWN
    // REPLY: A_IN_DATA
    Packet audio_in_data_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::AUDIO_IN_DATA_SENSE);
    }

    // 60.13 Audio Out Data Sense
    // UNKNOWN
    // REPLY: A_OUT_DATA
    Packet audio_out_data_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::AUDIO_OUT_DATA_SENSE);
    }

    // 61.20 STATUS SENSE
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
        LOG_INFO(" ");
        const uint8_t v = size | (start << 4);
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::STATUS_SENSE, v);
    }

    // 60.21 Extended VTR Status
    // UNKNOWN
    // REPLY: ACK
    Packet extended_vtr_status(const uint8_t data1) {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::EXTENDED_VTR_STATUS, data1);
    }

    // 62.23 Signal Control Sense
    // UNKNOWN
    // REPLY: ACK
    Packet signal_control_sense(const uint8_t data1, const uint8_t data2) {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::SIGNAL_CONTROL_SENSE, data1, data2);
    }

    // 6X.28 Local Key Map Sense
    // UNKNOWN
    // REPLY: ACK
    Packet local_keymap_sense() {
        LOG_INFO(" ");
        // return encode(Cmd1::SENSE_REQUEST, SenseRequest::LOCAL_KEYMAP_SENSE);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 61.2A Head Meter Sense
    // UNKNOWN
    // REPLY: ACK
    Packet head_meter_sense(const uint8_t data1) {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::HEAD_METER_SENSE, data1);
    }

    // 60.2B Remaining Time Sense
    // UNKNOWN
    // REPLY: ACK
    Packet remaining_time_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::REMAINING_TIME_SENSE);
    }

    // 60.2E Cmd Speed Sense
    // UNKNOWN
    // REPLY: SPEED_DATA
    Packet cmd_speed_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::CMD_SPEED_SENSE);
    }

    // 61.30 Edit Preset Sense
    // UNKNOWN
    // REPLY: ACK
    Packet edit_preset_sense(const uint8_t data1) {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::EDIT_PRESET_SENSE, data1);
    }

    // 60.30 PRE-ROLL TIME
    // This command is used for requesting the current pre-roll duration.
    // For the SENSE return data format, see the CUE UP WITH DATA command.
    // Send: 60 30 90
    // Returns: 74 30 00 05 00 00 A9 (Pre-roll is five seconds)
    Packet preroll_time_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::PREROLL_TIME_SENSE);
    }

    // 60.36 TIMER MODE
    // This command is used for requesting the device's default timer SENSE return type,
    // based on return DATA-1:
    //   DATA-1 : 00: Time Code
    //            01: Control (CTL) Counter
    // Send: 60 36 96
    // Returns: 71 36 00 A6 (Sense mode time code)
    Packet timer_mode_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::TIMER_MODE_SENSE);
    }

    // 60.3E Record Inhibit Sense
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet record_inhibit_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::RECORD_INHIBIT_SENSE);
    }

    // 60.52 DA Inp Emph Sense
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet da_inp_emph_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::DA_INPUT_EMPHASIS_SENSE);
    }

    // 60.53 DA PB Emph Sense
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet da_pb_emph_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::DA_PLAYBACK_EMPHASIS_SENSE);
    }

    // 60.58 DA Samp. Freq. Sense
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet da_samp_freq_sense() {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::DA_SAMPLING_FREQUENCY_SENSE);
    }

    // 61.AA Cross Fade Time Sense
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    Packet cross_fade_time_sense(const uint8_t data1) {
        LOG_INFO(" ");
        return encode(Cmd1::SENSE_REQUEST, SenseRequest::CROSS_FADE_TIME_SENSE, data1);
    }

    // =============== A - BlackMagic Advanced Media Protocol ===============

    // 08.02 BmdSeekToTimelinePosition
    // 16-bit little endian fractional position [0..65535]
    // REPLY: ACK
    Packet bmd_seek_to_timeline_pos(const uint8_t data1, const uint8_t data2) {
        LOG_INFO(" ");
        // TODO: more user-friendly arguments?
        return encode(Cmd1::SYSTEM_CONTROL, SystemCtrl::BMD_SEEK_TO_TIMELINE_POS, data1, data2);
    }

    // 20.29 ClearPlaylist
    // UNKNOWN
    // REPLY: ACK
    Packet clear_playlist() {
        LOG_INFO(" ");
        return encode(Cmd1::TRANSPORT_CONTROL, TransportCtrl::CLEAR_PLAYLIST);
    }

    // 4F.16 AppendPreset
    // 2 Bytes for the length N of the clip name
    // N Bytes for each character of the clip name
    // 4 Byte in point timecode (format is FFSSMMHH)
    // 4 Byte out point timecode (format is FFSSMMHH)
    // REPLY: ACK
    Packet append_preset() {
        LOG_INFO(" ");
        // return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::APPEND_PRESET);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

    // 41.42 SetPlaybackLoop
    // Bit0 loop mode enable, 0 = false, 1 = true
    // Bit1 is single clip/timeline, 0 = single clip, 1 = timeline
    // REPLY: ACK
    Packet set_playback_loop(const bool b_enable, const uint8_t mode = LoopMode::SINGLE_CLIP) {
        LOG_INFO(" ");
        const uint8_t v = (uint8_t)b_enable | ((uint8_t)(mode & 0x01) << 1);
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SET_PLAYBACK_LOOP, v);
    }

    // 41.44 SetStopMode
    // 0 = Off
    // 1 = Freeze on last frame of Timeline (not clip)
    // 2 = Freeze on next clip of Timeline (not clip)
    // 3 = Show black
    // REPLY: ACK
    Packet set_stop_mode(const uint8_t stop_mode) {
        LOG_INFO(" ");
        return encode(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SET_STOP_MODE, stop_mode);
    }

    // 81.03 BmdSeekRelativeClip
    // One-byte signed integer, which is the number of clips to skip (negative for backwards).
    // REPLY: ACK
    Packet bmd_seek_relative_clip(const int8_t index) {
        LOG_INFO(" ");
        return encode(Cmd1::BMD_EXTENSION, BmdExtensions::SEEK_RELATIVE_CLIP, index);
    }

    // A1.01 AutoSkip
    // 8-bit signed number of clips to skip from current clip
    // REPLY: ACK
    Packet auto_skip(const int8_t n) {
        LOG_INFO(" ");
        return encode(Cmd1::BMD_ADVANCED_MEDIA_PRTCL, BmdAdvancedMediaProtocol::AUTO_SKIP, (uint8_t)n);
    }

    // AX.15 ListNextID
    // when x = 0, single clip request
    // when x = 1, # clips can be specified in the encode data
    // REPLY: IDListing
    Packet list_next_id() {
        LOG_INFO(" ");
        // return encode(Cmd1::BMD_ADVANCED_MEDIA_PRTCL, BmdAdvancedMediaProtocol::LIST_NEXT_ID);
        // TODO: NOT IMPLEMENTED
        LOG_WARN("NOT IMPLEMENTED");
        return Packet();
    }

private:
    Packet encode(Packet& packet, uint8_t& crc) {
        packet.emplace_back(crc);
        // Serial.println(crc, HEX);
        return packet;
    }

    template <typename... Args>
    Packet encode(Packet& packet, uint8_t& crc, const uint8_t arg, Args&&... args) {
        packet.emplace_back(arg);
        crc += arg;
        // Serial.print(arg, HEX);
        // Serial.print(" ");
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
        // Serial.print(" : encode data = ");
        // Serial.print(header, HEX);
        // Serial.print(" ");
        // Serial.print((uint8_t)cmd2, HEX);
        // Serial.print(" ");
        return encode(packet, crc, util::forward<Args>(args)...);
    }

    template <typename T>
    inline auto from_dec_to_bcd(const T& n)
        -> typename std::enable_if<std::is_integral<T>::value, size_t>::type {
        return n + 6 * (n / 10);
    }
};

}  // namespace sony9pin

#include "util/DebugLog/DebugLogRestoreState.h"

#endif  // SONY9PINREMOTE_ENCODER_H
