#ifndef HT_RS422_SONY9PINREMOTE_TYPES_H
#define HT_RS422_SONY9PINREMOTE_TYPES_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <stdint.h>

namespace sony9pin {

namespace serial {
    static constexpr size_t BAUDRATE {38400};
    static constexpr size_t CONFIG {SERIAL_8O1};
}  // namespace serial

// Data Structs for Response

struct Errors {
    bool b_unknown_cmd;
    bool b_checksum_error;
    bool b_parity_error;
    bool b_buffer_overrun;
    bool b_framing_error;
    bool b_timeout;
};

struct Status {
    // byte 0
    bool b_cassette_out;  // set if no ssd is present
    bool b_local;         // set if remote is disabled (local control)
    // byte 1
    bool b_standby;  // set if a disk is available
    bool b_stop;
    bool b_rewind;
    bool b_forward;
    bool b_record;
    bool b_play;
    // byte 2
    bool b_servo_lock;
    bool b_shuttle;
    bool b_jog;
    bool b_var;
    bool b_direction;  // clear if playback is forwarding〝set if playback is reversing
    bool b_still;      // set if playback is paused, or if in input preview mode
    // byte 3
    bool b_auto_mode;  // set if in Auto Mode
    bool b_a_out_set;
    bool b_a_in_set;
    bool b_out_set;
    bool b_in_set;
    // byte 4
    bool b_select_ee;  // set if in input preview mode
    bool b_full_ee;
    // byte 6
    bool b_lamp_still;  // set according to playback speed and direction
    bool b_lamp_fwd;
    bool b_lamp_rev;
    // byte 8
    bool b_near_eot;  // set if total space left on available SSDs is less than 3 minutes
    bool b_eot;       // set if total space left on available SSDs is less than 30 seconds
};

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

// Cmd1 Lists

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

// Cmd2 lists based on Cmd1

// 0 - System Control
namespace SystemCtrl {
    enum : uint8_t {
        LOCAL_DISABLE = 0x0C,
        DEVICE_TYPE = 0x11,
        LOCAL_ENABLE = 0x1D,
        // Black Magic Advanced Media Protocol
        BMD_SEEK_TO_TIMELINE_POS = 0x02,
    };
}

// 1 - System Control Return
namespace SystemControlReturn {
    enum : uint8_t {
        ACK = 0x01,          // auto parse
        NAK = 0x12,          // auto parse
        DEVICE_TYPE = 0x11,  // auto parse
    };
}

// 2 - Transport Control
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

// 4 - Preset/Select Control
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

// 6 - Sense Request
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
        DA_INP_EMPH_SENSE = 0x52,
        DA_PB_EMPH_SENSE = 0x53,
        DA_SAMP_FREQ_SENSE = 0x58,
        CROSS_FADE_TIME_SENSE = 0xAA,
    };
}

// 7 - Sense Reply
namespace SenseReturn {
    enum : uint8_t {
        TIMER1_DATA = 0x00,
        LTC_USER_BITS_TIME_DATA = 0x04,
        VITCU_USER_BITS_TIME_DATA = 0x06,
        VITC_TIME_DATA = 0x06,
        USER_BITS_VITC_TIME_DATA = 0x07,
        GEN_TC_DATA = 0x08,
        GEN_TCUB_DATA = 0x08,
        GEN_UB_DATA = 0x09,
        IN_DATA = 0x10,
        OUT_DATA = 0x11,
        A_IN_DATA = 0x12,
        A_OUT_DATA = 0x13,
        CORRECTED_LTC_TIME_DATA = 0x14,
        STATUS_DATA = 0x20,  // auto parse
        SPEED_DATA = 0x2E,
        PREROLL_TIME_DATA = 0x31,
        TIMER_MODE_DATA = 0x36,
        RECORD_INHIBIT_STATUS = 0x3E
    };
}

// 8 - BlackMagic Extensions
namespace BmdExtensions {
    enum : uint8_t {
        SEEK_RELATIVE_CLIP = 0x03,
    };
}

// A - BlackMagic Advanced Media Protocol
namespace BmdAdvancedMediaProtocol {
    enum : uint8_t {
        AUTO_SKIP = 0x01,
        LIST_NEXT_ID = 0x15,  // TODO: NOT IMPLEMENTED
    };
}

// Bit Masks

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
        LOCAL = 0b00000001,
        // byte 1
        STANDBY = 0b10000000,
        STOP = 0b00100000,
        REWIND = 0b00001000,
        FORWARD = 0b00000100,
        RECORD = 0b00000010,
        PLAY = 0b00000001,
        // byte 2
        SERVO_LOCK = 0b10000000,
        SHUTTLE = 0b00100000,
        JOG = 0b00010000,
        VAR = 0b00001000,
        DIRECTION = 0b00000100,
        STILL = 0b00000010,
        // byte 3
        AUTO_MODE = 0b10000000,
        A_OUT_SET = 0b00001000,
        A_IN_SET = 0b00000100,
        OUT_SET = 0b00000010,
        IN_SET = 0b00000001,
        // byte 4
        SELECT_EE = 0b10000000,
        FULL_EE = 0b01000000,
        // byte 6
        LAMP_STILL = 0b01000000,
        LAMP_FWD = 0b00100000,
        LAMP_REV = 0b00010000,
        // byte 8
        NEAR_EOT = 0b00100000,
        EOT = 0b00010000
    };
}

namespace DeviceType {
    enum : uint16_t {
        BLACKMAGIC_HYPERDECK_STUDIO_MINI_NTSC = 0xF0E0,
        BLACKMAGIC_HYPERDECK_STUDIO_MINI_PAL = 0xF1E0,
        BLACKMAGIC_HYPERDECK_STUDIO_MINI_24P = 0xF2E0,
    };
}

namespace LoopMode {
    enum : uint8_t {
        SINGLE_CLIP,
        TIMELINE
    };
}

namespace StopMode {
    enum : uint8_t {
        OFF,
        FREEZE_ON_LAST_FRAME,
        FREEZE_ON_NEXT_CLIP,
        SHOW_BLACK
    };
}

// 61.0A TC Generator Data Types
namespace TcGenData {
    enum : uint8_t {
        TC = 0x01,    // 74.08 GEN TIME DATA Respond
        UB = 0x10,    // 74.09 GEN UB DATA Respond
        TC_UB = 0x11  // 78.08 GEN TC & UB DATA Respond
    };
}

namespace CurrentTimeSenseFlag {
    enum : uint8_t {
        LTC_TIME = 0x01,
        VITC_TIME = 0x02,
        TIMER_1 = 0x04,
        TIMER_2 = 0x08,
        LTC_UB = 0x10,
        VITC_UB = 0x20,
    };
}

}  // namespace sony9pin

#endif  // HT_RS422_SONY9PINREMOTE_TYPES_H
