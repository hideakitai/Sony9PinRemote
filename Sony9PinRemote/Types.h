#ifndef HT_RS422_SONY9PINREMOTE_TYPES_H
#define HT_RS422_SONY9PINREMOTE_TYPES_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <stdint.h>

namespace sony9pin {

// =============== Common Constants ===============

static constexpr uint8_t MAX_PACKET_SIZE {15 + 3};

// =============== Cmd1 Lists ===============

enum class Cmd1 : uint8_t {
    SYSTEM_CONTROL = 0x00,            // ---> device
    SYSTEM_CONTROL_RETURN = 0x10,     // <--- device
    TRANSPORT_CONTROL = 0x20,         // ---> device
    PRESET_SELECT_CONTROL = 0x40,     // ---> device
    SENSE_REQUEST = 0x60,             // ---> device
    SENSE_RETURN = 0x70,              // <--- device
    BMD_EXTENSION = 0x80,             // <--> device
    BMD_ADVANCED_MEDIA_PRTCL = 0xA0,  // <--> device
    NA = 0xFF
};

// =============== Cmd2 lists  ===============

// ===== 0 - System Control =====
namespace SystemCtrl {
    enum : uint8_t {
        LOCAL_DISABLE = 0x0C,
        DEVICE_TYPE = 0x11,
        LOCAL_ENABLE = 0x1D,
        // Black Magic Advanced Media Protocol
        BMD_SEEK_TO_TIMELINE_POS = 0x02,
    };
}

// ===== 1 - System Control Return =====
namespace SystemControlReturn {
    enum : uint8_t {
        ACK = 0x01,          // auto parse
        NAK = 0x12,          // auto parse
        DEVICE_TYPE = 0x11,  // auto parse
    };
}

// ===== 2 - Transport Control =====
namespace TransportCtrl {
    enum : uint8_t {
        STOP = 0x00,
        PLAY = 0x01,
        RECORD = 0x02,
        STANDBY_OFF = 0x04,
        STANDBY_ON = 0x05,
        EJECT = 0x0F,
        FAST_FWD = 0x10,
        JOG_FWD = 0x11,
        VAR_FWD = 0x12,
        SHUTTLE_FWD = 0x13,
        FRAME_STEP_FWD = 0x14,
        FAST_REVERSE = 0x20,
        REWIND = 0x20,
        JOG_REV = 0x21,
        VAR_REV = 0x22,
        SHUTTLE_REV = 0x23,
        FRAME_STEP_REV = 0x24,
        PREROLL = 0x30,
        CUE_UP_WITH_DATA = 0x31,
        SYNC_PLAY = 0x34,
        PROG_SPEED_PLAY_PLUS = 0x38,
        PROG_SPEED_PLAY_MINUS = 0x39,
        PREVIEW = 0x40,
        REVIEW = 0x41,
        AUTO_EDIT = 0x42,
        OUTPOINT_PREVIEW = 0x43,
        ANTI_CLOG_TIMER_DISABLE = 0x54,  // TODO: NOT IMPLEMENTED
        ANTI_CLOG_TIMER_ENABLE = 0x55,   // TODO: NOT IMPLEMENTED
        DMC_SET_FWD = 0x5C,
        DMC_SET_REV = 0x5D,
        FULL_EE_OFF = 0x60,
        FULL_EE_ON = 0x61,
        SELECT_EE_ON = 0x63,
        EDIT_OFF = 0x64,
        EDIT_ON = 0x65,
        FREEZE_OFF = 0x6A,
        FREEZE_ON = 0x6B,
        // BlackMagic Advanced Media Protocol
        CLEAR_PLAYLIST = 0x29,
    };
}

// ===== 4 - Preset/Select Control =====
namespace PresetSelectCtrl {
    enum : uint8_t {
        TIMER_1_PRESET = 0x00,
        TIME_CODE_PRESET = 0x04,
        USER_BIT_PRESET = 0x05,
        TIMER_1_RESET = 0x08,
        IN_ENTRY = 0x10,
        OUT_ENTRY = 0x11,
        AUDIO_IN_ENTRY = 0x12,
        AUDIO_OUT_ENTRY = 0x13,
        IN_DATA_PRESET = 0x14,
        OUT_DATA_PRESET = 0x15,
        AUDIO_IN_DATA_PRESET = 0x16,   // TODO: NOT IMPLEMENTED
        AUDIO_OUT_DATA_PRESET = 0x17,  // TODO: NOT IMPLEMENTED
        IN_SHIFT_PLUS = 0x18,
        IN_SHIFT_MINUS = 0x19,
        OUT_SHIFT_PLUS = 0x1A,
        OUT_SHIFT_MINUS = 0x1B,
        AUDIO_IN_SHIFT_PLUS = 0x1C,
        AUDIO_IN_SHIFT_MINUS = 0x1D,
        AUDIO_OUT_SHIFT_PLUS = 0x1E,
        AUDIO_OUT_SHIFT_MINUS = 0x1F,
        IN_FLAG_RESET = 0x20,
        OUT_FLAG_RESET = 0x21,
        AUDIO_IN_FLAG_RESET = 0x22,
        AUDIO_OUT_FLAG_RESET = 0x23,
        IN_RECALL = 0x24,
        OUT_RECALL = 0x25,
        AUDIO_IN_RECALL = 0x26,
        AUDIO_OUT_RECALL = 0x27,
        LOST_LOCK_RESET = 0x2D,
        EDIT_PRESET = 0x30,
        PREROLL_PRESET = 0x31,
        TAPE_AUDIO_SELECT = 0x32,
        SERVO_REF_SELECT = 0x33,
        HEAD_SELECT = 0x34,
        COLOR_FRAME_SELECT = 0x35,
        TIMER_MODE_SELECT = 0x36,
        INPUT_CHECK = 0x37,
        EDIT_FIELD_SELECT = 0x3A,
        FREEZE_MODE_SELECT = 0x3B,
        RECORD_INHIBIT = 0x3E,  // TODO: NOT IMPLEMENTED
        AUTO_MODE_OFF = 0x40,
        AUTO_MODE_ON = 0x41,
        SPOT_ERASE_OFF = 0x42,
        SPOT_ERASE_ON = 0x43,
        AUDIO_SPLIT_OFF = 0x44,
        AUDIO_SPLIT_ON = 0x45,
        OUTPUT_H_PHASE = 0x98,          // TODO: NOT IMPLEMENTED
        OUTPUT_VIDEO_PHASE = 0x9B,      // TODO: NOT IMPLEMENTED
        AUDIO_INPUT_LEVEL = 0xA0,       // TODO: NOT IMPLEMENTED
        AUDIO_OUTPUT_LEVEL = 0xA1,      // TODO: NOT IMPLEMENTED
        AUDIO_ADV_LEVEL = 0xA2,         // TODO: NOT IMPLEMENTED
        AUDIO_OUTPUT_PHASE = 0xA8,      // TODO: NOT IMPLEMENTED
        AUDIO_ADV_OUTPUT_PHASE = 0xA9,  // TODO: NOT IMPLEMENTED
        CROSS_FADE_TIME_PRESET = 0xAA,  // TODO: NOT IMPLEMENTED
        LOCAL_KEY_MAP = 0xB8,           // TODO: NOT IMPLEMENTED
        STILL_OFF_TIME = 0xF8,
        STBY_OFF_TIME = 0xFA,
        // BlackMagic Advanced Media Protocol
        APPEND_PRESET = 0x16,  // TODO: NOT IMPLEMENTED
        SET_PLAYBACK_LOOP = 0x42,
        SET_STOP_MODE = 0x44
    };
}

// ===== 6 - Sense Request =====
namespace SenseRequest {
    enum : uint8_t {
        TC_GEN_SENSE = 0x0A,
        CURRENT_TIME_SENSE = 0x0C,
        IN_DATA_SENSE = 0x10,
        OUT_DATA_SENSE = 0x11,
        AUDIO_IN_DATA_SENSE = 0x12,
        AUDIO_OUT_DATA_SENSE = 0x13,
        STATUS_SENSE = 0x20,
        EXTENDED_VTR_STATUS = 0x21,
        SIGNAL_CONTROL_SENSE = 0x23,
        LOCAL_KEYMAP_SENSE = 0x28,
        HEAD_METER_SENSE = 0x2A,
        REMAINING_TIME_SENSE = 0x2B,
        CMD_SPEED_SENSE = 0x2E,
        EDIT_PRESET_SENSE = 0x30,
        PREROLL_TIME_SENSE = 0x31,
        TIMER_MODE_SENSE = 0x36,
        RECORD_INHIBIT_SENSE = 0x3E,
        DA_INPUT_EMPHASIS_SENSE = 0x52,
        DA_PLAYBACK_EMPHASIS_SENSE = 0x53,
        DA_SAMPLING_FREQUENCY_SENSE = 0x58,
        CROSS_FADE_TIME_SENSE = 0xAA,
    };
}

// ===== 7 - Sense Reply =====
namespace SenseReturn {
    enum : uint8_t {
        TIMER_1 = 0x00,
        TIMER_2 = 0x01,
        LTC_TC_UB = 0x04,  // size == 8
        LTC_TC = 0x04,     // size == 4
        LTC_UB = 0x05,
        VITC_TC_UB = 0x06,  // size == 8
        VITC_TC = 0x06,     // size == 4
        VITC_UB = 0x07,
        GEN_TC_UB = 0x08,  // size == 8
        GEN_TC = 0x08,     // size == 4
        GEN_UB = 0x09,
        IN_DATA = 0x10,
        OUT_DATA = 0x11,
        AUDIO_IN_DATA = 0x12,
        AUDIO_OUT_DATA = 0x13,
        LTC_INTERPOLATED_TC_UB = 0x14,  // size == 8
        LTC_INTERPOLATED_TC = 0x14,     // size == 4
        LTC_INTERPOLATED_UB = 0x15,
        HOLD_VITC_TC_UB = 0x16,  // size == 8
        HOLD_VITC_TC = 0x16,     // size == 4
        HOLD_VITC_UB = 0x17,
        STATUS_DATA = 0x20,
        EXTENDED_STATUS_DATA = 0x21,
        SIGNAL_CONTROL_DATA = 0x23,
        LOCAL_KEYMAP = 0x28,
        HEAD_METER_DATA = 0x2A,
        REMAINING_TIME = 0x2B,
        CMD_SPEED_DATA = 0x2E,
        EDIT_PRESET_STATUS = 0x30,
        PREROLL_TIME = 0x31,
        TIMER_MODE_STATUS = 0x36,
        RECORD_INHIBIT_STATUS = 0x3E,
        DA_INPUT_EMPHASIS_DATA = 0x52,
        DA_PLAYBACK_EMPHASIS_DATA = 0x53,
        DA_SAMPLING_FREQUENCY_DATA = 0x58,
        CROSS_FADE_TIME_DATA = 0xAA,
    };
}

// ===== 8 - BlackMagic Extensions =====
namespace BmdExtensions {
    enum : uint8_t {
        SEEK_RELATIVE_CLIP = 0x03,
    };
}

// ===== A - BlackMagic Advanced Media Protocol =====
namespace BmdAdvancedMediaProtocol {
    enum : uint8_t {
        AUTO_SKIP = 0x01,
        LIST_NEXT_ID = 0x15,  // TODO: NOT IMPLEMENTED
    };
}

// =============== Bit Masks ===============

namespace HeaderMask {
    enum : uint8_t {
        CMD1 = 0b11110000,
        SIZE = 0b00001111,
    };
}

namespace NakMask {
    enum : uint8_t {
        UNKNOWN_CMD = 0b00000001,
        NOT_USED_1 = 0b00000010,
        CHECKSUM_ERROR = 0b00000100,
        NOT_USED_3 = 0b00001000,
        PARITY_ERROR = 0b00010000,
        BUFFER_OVERRUN = 0b00100000,
        FRAMING_ERROR = 0b01000000,
        TIMEOUT = 0b10000000
    };
}

namespace StatusMask {
    enum : uint8_t {
        // byte 0
        CASSETTE_OUT = 0b00100000,
        SERVO_REF_MISSING = 0b00010000,
        LOCAL = 0b00000001,
        // byte 1
        STANDBY = 0b10000000,
        STOP = 0b00100000,
        EJECT = 0b00010000,
        REWIND = 0b00001000,
        FORWARD = 0b00000100,
        RECORD = 0b00000010,
        PLAY = 0b00000001,
        // byte 2
        SERVO_LOCK = 0b10000000,
        TSO_MODE = 0b01000000,
        SHUTTLE = 0b00100000,
        JOG = 0b00010000,
        VAR = 0b00001000,
        DIRECTION = 0b00000100,
        STILL = 0b00000010,
        CUE_UP = 0b00000001,
        // byte 3
        AUTO_MODE = 0b10000000,
        FREEZE_ON = 0b01000000,
        CF_MODE = 0b00010000,
        AUDIO_OUT_SET = 0b00001000,
        AUDIO_IN_SET = 0b00000100,
        OUT_SET = 0b00000010,
        IN_SET = 0b00000001,
        // byte 4
        SELECT_EE = 0b10000000,
        FULL_EE = 0b01000000,
        EDIT_SET = 0b00010000,
        REVIEW_SET = 0b00001000,
        AUTO_EDIT_SET = 0b00000100,
        PREVIEW_SET = 0b00000010,
        PREROLL_SET = 0b00000001,
        // byte 5
        INSERT_SET = 0b01000000,
        ASSEMBLE_SET = 0b00100000,
        VIDEO_SET = 0b00010000,
        A4_SET = 0b00001000,
        A3_SET = 0b00000100,
        A2_SET = 0b00000010,
        A1_SET = 0b00000001,
        // byte 6
        LAMP_STILL = 0b01000000,
        LAMP_FWD = 0b00100000,
        LAMP_REV = 0b00010000,
        SRCH_LED_8 = 0b00001000,
        SRCH_LED_4 = 0b00000100,
        SRCH_LED_2 = 0b00000010,
        SRCH_LED_1 = 0b00000001,
        // byte 7
        AUD_SPLIT = 0b00100000,
        SYNC_ACT = 0b00010000,
        SPOT_ERASE = 0b00000100,
        IN_OUT = 0b00000001,
        // byte 8
        BUZZER = 0b10000000,
        LOST_LOCK = 0b01000000,
        NEAR_EOT = 0b00100000,
        EOT = 0b00010000,
        CF_LOCK = 0b00001000,
        SVO_ALARM = 0b00000100,
        SYS_ALARM = 0b00000010,
        REC_INHIB = 0b00000001,
        // byte 9
        FNC_ABORT = 0b10000000,
    };
}

// =============== Data Structs for Decoder ===============

struct Errors {
    bool b_unknown_cmd {false};
    bool b_checksum_error {false};
    bool b_parity_error {false};
    bool b_buffer_overrun {false};
    bool b_framing_error {false};
    bool b_timeout {false};
};

struct Status {
    // byte 0
    bool b_cassette_out {false};       // set if no ssd is present
    bool b_servo_ref_missing {false};  // set if servo reference is absent
    bool b_local {false};              // set if remote is disabled (local control)
    // byte 1
    bool b_standby {false};  // set if a disk is available
    bool b_stop {false};     // When the machine is in full stop, this is 1. The thread state depends on the tape/ee and standby settings.
    bool b_eject {false};    // When the tape is ejecting this is 1.
    bool b_rewind {false};   // When the machine is in fast reverse this is 1.
    bool b_forward {false};  // When the machine is in fast forward this is 1.
    bool b_record {false};   // This bit goes from 0 to 1 some number of frames after the machine starts recording. For the DVR2000 we measured 5 frames. Others have varying delays on the record status.
    bool b_play {false};     // This bit goes from 0 to 1 some number of frames after the machine starts playing. For the DVR2000 we measured 5 frames. Others have varying delays on the play status.
    // byte 2
    bool b_servo_lock {false};  // 1 indicates servos are locked. This is a necessary condition for an edit to occur correctly.
    bool b_tso_mode {false};    // Bit is 1 in tape speed override: in this mode, audio and video are still locked though speed is off play speed by +/- up to 15%.
    bool b_shuttle {false};
    bool b_jog {false};
    bool b_var {false};
    bool b_direction {false};  // clear if playback is forwarding〝set if playback is reversing
    bool b_still {false};      // set if playback is paused, or if in input preview mode
    bool b_cue_up {false};
    // byte 3
    bool b_auto_mode {false};  // set if in Auto Mode
    bool b_freeze_on {false};
    bool b_cf_mode {false};
    bool b_audio_out_set {false};
    bool b_audio_in_set {false};
    bool b_out_set {false};
    bool b_in_set {false};
    // byte 4
    bool b_select_ee {false};  // set if in input preview mode
    bool b_full_ee {false};
    bool b_edit {false};
    bool b_review {false};
    bool b_auto_edit {false};
    bool b_preview {false};
    bool b_preroll {false};
    // byte 5
    bool b_insert {false};
    bool b_assemble {false};
    bool b_video {false};
    bool b_a4 {false};
    bool b_a3 {false};
    bool b_a2 {false};
    bool b_a1 {false};
    // byte 6
    bool b_lamp_still {false};  // set according to playback speed and direction
    bool b_lamp_fwd {false};
    bool b_lamp_rev {false};
    bool b_srch_led_8 {false};
    bool b_srch_led_4 {false};
    bool b_srch_led_2 {false};
    bool b_srch_led_1 {false};
    // byte 7
    bool b_aud_split {false};
    bool b_sync_act {false};
    bool b_spot_erase {false};
    bool b_in_out {false};
    // byte 8
    bool b_buzzer {false};
    bool b_lost_lock {false};
    bool b_near_eot {false};  // set if total space left on available SSDs is less than 3 minutes
    bool b_eot {false};       // set if total space left on available SSDs is less than 30 seconds
    bool b_cf_lock {false};
    bool b_svo_alarm {false};
    bool b_sys_alarm {false};
    bool b_rec_inhib {false};
    // byte 9
    bool b_fnc_abort {false};
};

struct TimeCode {
    uint8_t frame {0};
    uint8_t second {0};
    uint8_t minute {0};
    uint8_t hour {0};
    bool is_cf {false};
    bool is_df {false};
};

union UserBits {
    uint8_t bytes[4];
    uint32_t i {0};
};

struct TimeCodeAndUserBits {
    TimeCode tc;
    UserBits ub;
};

// =============== Mode / Flag Structs ===============

// 12.11 DEVICE TYPE
namespace DeviceType {
    enum : uint16_t {
        BLACKMAGIC_HYPERDECK_STUDIO_MINI_NTSC = 0xF0E0,
        BLACKMAGIC_HYPERDECK_STUDIO_MINI_PAL = 0xF1E0,
        BLACKMAGIC_HYPERDECK_STUDIO_MINI_24P = 0xF2E0,
    };
}

// 41.36 TIMER MODE SELECT
enum class TimerMode : uint8_t {
    TIME_CODE = 0x00,
    CTL_COUNTER = 0x01,
    NA = 0xFF,
};

// 61.0A TC Generator Data Types
namespace TcGenData {
    enum : uint8_t {
        TC = 0x01,    // 74.08 GEN TIME DATA Respond
        UB = 0x10,    // 74.09 GEN UB DATA Respond
        TC_UB = 0x11  // 78.08 GEN TC & UB DATA Respond
    };
}

// 61.0C CURRENT TIME
namespace CurrentTimeSenseFlag {
    enum : uint8_t {
        LTC_TC = 0x01,
        VITC_TC = 0x02,
        TIMER_1 = 0x04,
        TIMER_2 = 0x08,
        LTC_UB = 0x10,
        VITC_UB = 0x20,
    };
}

// 41.42 SetPlaybackLoop (BlackMagiconly)
namespace LoopMode {
    enum : uint8_t {
        SINGLE_CLIP,
        TIMELINE
    };
}

// 41.44 SetStopMode (BlackMagic only)
namespace StopMode {
    enum : uint8_t {
        OFF,
        FREEZE_ON_LAST_FRAME,
        FREEZE_ON_NEXT_CLIP,
        SHOW_BLACK
    };
}

}  // namespace sony9pin

#endif  // HT_RS422_SONY9PINREMOTE_TYPES_H
