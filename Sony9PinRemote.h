#ifndef HT_RS422_SONY9PINREMOTE_H
#define HT_RS422_SONY9PINREMOTE_H


#ifdef ARDUINO
    #include <Arduino.h>
#endif
#include <stdint.h>

#if defined(ARDUINO)\
 || defined(OF_VERSION_MAJOR)
    #define SONY9PINREMOTE_ENABLE_STREAM
#endif

#include "Sony9PinRemote/Types.h"
#include "Sony9PinRemote/Response.h"

namespace ht {
namespace serial {
namespace rs422 {
namespace sony9pin {


#ifdef SONY9PINREMOTE_ENABLE_STREAM
#ifdef ARDUINO
    using StreamType = Stream;
    #define SONY9PINREMOTE_STREAM_WRITE(data) stream->write(data)
    #define SONY9PINREMOTE_STREAM_READ stream->read
#elif defined (OF_VERSION_MAJOR)
    using StreamType = ofSerial;
    #define SONY9PINREMOTE_STREAM_WRITE(data) stream->writeByte(data)
    #define SONY9PINREMOTE_STREAM_READ stream->readByte
#endif
#else
    #error THIS PLATFORM IS NOT SUPPORTED
#endif // SONY9PINREMOTE_ENABLE_STREAM


namespace util
{
    template <class T> struct remove_reference      { using type = T; };
    template <class T> struct remove_reference<T&>  { using type = T; };
    template <class T> struct remove_reference<T&&> { using type = T; };

    template <class T>
    constexpr T&& forward(typename remove_reference<T>::type& t) noexcept
    {
        return static_cast<T&&>(t);
    }
    template <class T>
    constexpr T&& forward(typename remove_reference<T>::type&& t) noexcept
    {
        return static_cast<T&&>(t);
    }
}

class Controller
{
    // reference
    // https://en.wikipedia.org/wiki/9-Pin_Protocol
    // https://www.drastic.tv/support-59/legacysoftwarehardware/72-miscellaneous-legacy/158-vvcr-422-serial-protocol

    StreamType* stream;
    Response res;
    bool b_force_send {false};

public:


    void attach(StreamType& s, const bool force_send = false)
    {
        b_force_send = force_send;
        stream = &s;
        stream->flush();
        while (stream->available())
            stream->read();
    }

    void parse()
    {
        while (stream->available())
            res.feed(stream->read());
    }

    bool parse_until(const uint32_t timeout_ms)
    {
        const uint32_t begin_ms = millis();
        while (true)
        {
            if (stream->available())
            {
                if (res.feed(stream->read()))
                    return true;
            }
            if (millis() > begin_ms + timeout_ms)
                return false;
        }
    }


    bool ready() const { return b_force_send ? true : !res.busy(); }
    bool available() const { return available(); }

    uint16_t device() const { return res.device_type(); }
    const Status& status() const { return res.status(); }
    const Errors& errors() const { return res.errors(); }
    size_t error_count() const { return res.error_count(); }


    // 0 - System Control

    // DESCRIPTION:
    // When receiving this command, all local operational functions of the device will be disabled.
    // This includes front panel transport controls, but not front panel setup controls.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void local_disable()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SYSTEM_CONTROL, SystemCtrl::LOCAL_DISABLE);
    }

    // DESCRIPTION:
    // When the device receives the DEVICE TYPE REQUEST command
    // DEVICE TYPE return with 2 bytes data will be returned:
    // REPLY: SystemControlReturn::DEVICE_TYPE
    void device_type()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SYSTEM_CONTROL, SystemCtrl::DEVICE_TYPE);
    }

    // DESCRIPTION:
    // When receiving this command, the front panel operation of the device will be enabled.
    // When the device is initially powered on, it will be set to the LOCAL ENABLE state.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void lock_enable()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SYSTEM_CONTROL, SystemCtrl::LOCK_ENABLE);
    }


    // 2 - Transport Control

    // DESCRIPTION:
    // Stop the device and pass the device's input to the device's output.
    // Cease all processing of the current material.
    // REPLY: ACK
    void stop()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STOP);
    }

    // DESCRIPTION:
    // Plays from the current position at normal play speed for the material.
    // REPLY: ACK
    void play()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PLAY);
    }

    // DESCRIPTION:
    // Records from the current position at normal play speed.
    // REPLY: ACK
    void record()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::RECORD);
    }

    // DESCRIPTION:
    // The STANDBY OFF command places the device in a stop state, passing all material from the current inputs to the outputs.
    // This should be sent after a stop command to place the device in a fully idle state.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void standby_off()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STANDBY_OFF);
    }

    // DESCRIPTION:
    // Places the device in ready, pause mode.
    // The current material is ready for use and the current material, if possible, is presented at the output.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void standby_on()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::STANDBY_ON);
    }

    // DESCRIPTION:
    // If the device supports removable media, remove the media from the device.
    // REPLY: ACK
    // HyperDeck NOTE: NOT SUPPORTED
    void eject()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::EJECT);
    }

    // DESCRIPTION:
    // Moves forward through the material at the highest allowable speed
    // (Usually FORWARD 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: x2 faster
    void fast_forward()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FAST_FWD);
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
    void jog_forward(const uint8_t data1, const uint8_t data2 = 0)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::JOG_FWD, data1, data2);
    }

    // DESCRIPTION:
    // Move forward through the material, while creating the smoothest possible FORWARD output of the material.
    // This 'smoothing' process may slightly vary the requested speed.
    // REPLY: ACK
    void var_forward(const uint8_t data1, const uint8_t data2 = 0)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::VAR_FWD, data1, data2);
    }

    // DESCRIPTION:
    // Move forward through the material, at the exact play speed, regardless of FORWARD results.
    // Usually used for visual searching.
    // REPLY: ACK
    void shuttle_forward(const uint8_t data1, const uint8_t data2 = 0)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SHUTTLE_FWD, data1, data2);
    }

    // DESCRIPTION:
    // Moves backward through the material at the highest allowable speed
    // (Usually REWIND 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: same as rewind()
    void fast_reverse()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FAST_REVERSE);
    }

    // DESCRIPTION:
    // Moves backward through the material at the highest allowable speed
    // (Usually REWIND 32 to 90 times play speed).
    // REPLY: ACK
    // HyperDeck NOTE: x2 faster
    void rewind()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::REWIND);
    }

    // DESCRIPTION:
    // Move backward through the material,
    // usually with varying speeds sent by the REVERSE controller, for fine positioning.
    // REPLY: ACK
    void jog_reverse(const uint8_t data1, const uint8_t data2 = 0)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::JOG_REV, data1, data2);
    }

    // DESCRIPTION:
    // Move backward through the material, while to creating the smoothest possible REVERSE output of the material.
    // This 'smoothing' process may vary the speed slightly from the requested speed.
    // REPLY: ACK
    void var_reverse(const uint8_t data1, const uint8_t data2 = 0)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::VAR_REV, data1, data2);
    }

    // DESCRIPTION:
    // Move backward through the material, at the exact play speed, regardless of REVERSE results.
    // Usually used for visual searching.
    // REPLY: ACK
    void shuttle_reverse(const uint8_t data1, const uint8_t data2 = 0)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SHUTTLE_REV, data1, data2);
    }

    // DESCRIPTION:
    // Positions the device at the current in point (IN ENTRY)
    // minus the length of the current pre-roll (PRE-ROLL TIME PRESET).
    // REPLY: ACK
    void preroll()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PREROLL);
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
    void cue_data(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::CUE_DATA, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void sync_play()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SYNC_PLAY);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void prog_speed_play_plus(const uint8_t v)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PROG_SPEED_PLAY_PLUS, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void prog_speed_play_minus(const uint8_t v)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PROG_SPEED_PLAY_MINUS, v);
    }

    // DESCRIPTION:
    // Play the current edit.
    // Cue the device to the pre-roll point (in point minus pre-roll duration),
    // play the device through the in point to the point two seconds
    // (assuming a two second post-roll) after the out point.
    // REPLY: ACK
    void preview()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::PREVIEW);
    }

    // DESCRIPTION:
    // Play the last edit.
    // Cue the device to the last pre-roll point (last in point minus pre-roll duration),
    // play the device through the last in point to the point two seconds
    // (assuming a two second post-roll) after the last out point.
    // REPLY: ACK
    void review()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::REVIEW);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void outpoint_preview()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::OUTPOINT_PREVIEW);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void dmc_set_fwd(const uint8_t data1, const uint8_t data2)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::DMC_SET_FWD, data1, data2);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void dmc_set_rev(const uint8_t data1, const uint8_t data2)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::DMC_SET_REV, data1, data2);
    }

    // DESCRIPTION:
    // Full 'Edit To Edit' mode off attempts to pass all material from the device to the output.
    // This device has no effect on the current EDIT PRESET, but it does set all channels to the device,
    // unless the device is in an idle state.
    // REPLY: ACK
    void full_ee_off()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FULL_EE_OFF);
    }

    // DESCRIPTION:
    // Full 'Edit to Edit' mode on attempts to pass all inputs to the device to the device's output.
    // This device has no effect on the current EDIT PRESET but it does set all channels to the device's inputs.
    // REPLY: ACK
    void full_ee_on()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::FULL_EE_ON);
    }

    // DESCRIPTION:
    // Sets each EDIT PRESET channel assigned by the DATA-1 of the EDIT PRESET command to the edit to edit mode.
    // All selected channels are passed through from the device's inputs to the device's outputs.
    // To clear the SELECTED EE mode, use the EE OFF or the EDIT OFF command.
    // REPLY: ACK
    void select_ee_on()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::SELECT_EE_ON);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void clearPlaylist()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::TRANSPORT_CONTROL, TransportCtrl::CLEAR_PLAYLIST);
    }


    // 4 - Preset/Select Control

    // DESCRIPTION:
    // Store the current position of the device as the in point for the next edit.
    // REPLY: ACK
    void in_entry()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_ENTRY);
    }

    // DESCRIPTION:
    // Store the current position of the device as the out point for the next edit.
    // REPLY: ACK
    void out_entry()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_ENTRY);
    }

    // DESCRIPTION:
    // Set the in point for the next edit to the time specified by DATA-1 through DATA-4.
    // See the CUE UP WITH DATA command for the data format.
    // Send: 44 14 21 16 25 04 68 (Set in point to 4 hours, 25 minutes, 16 seconds, 21 frames)
    // REPLY: ACK
    void in_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_DATA_PRESET, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // Set the out point for the next edit to the time specified by DATA-1 through DATA-4.
    // See the CUE UP WITH DATA command for the data format.
    // Send: 44 15 05 09 27 04 92 (Set out point to 4 hours, 27 minutes, 9 seconds, 5 frames)
    // REPLY: ACK
    void out_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_DATA_PRESET, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // Adds one frame to the current in point time code value.
    // REPLY: ACK
    void in_shift_plus()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_FWD);
    }

    // DESCRIPTION:
    // Subtracts one frame from the current in point time code value.
    // REPLY: ACK
    void in_shift_minus()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_REV);
    }

    // DESCRIPTION:
    // Adds one frame to the current out point time code value.
    // REPLY: ACK
    void out_shift_plus()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_FWD);
    }

    // DESCRIPTION:
    // Subtracts one frame from the current out point time code value.
    // REPLY: ACK
    void out_shift_minus()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_REV);
    }

    // DESCRIPTION:
    // Reset the value of the in point to zero.
    // REPLY: ACK
    void in_reset()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::IN_RESET);
    }

    // DESCRIPTION:
    // Reset the value of the out point to zero.
    // REPLY: ACK
    void out_reset()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::OUT_RESET);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void a_in_reset()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::A_IN_RESET);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void a_out_reset()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::A_OUT_RESET);
    }

    // DESCRIPTION:
    // Presets the duration of the pre-roll to the length given by the DATA-1 to PRESET DATA-4.
    // For the data format, refer to the CUE UP WITH DATA command.
    // for example:
    // Send: 44 31 00 05 00 00 7A (Set the pre-roll duration to 5 seconds)
    // REPLY: ACK
    void preroll_prset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::PREROLL_PRESET, frames, seconds, minutes, hours);
    }

    // DESCRIPTION:
    // This command switches the device from AUTO mode.
    // REPLY: ACK
    void auto_mode_off()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUTO_MODE_OFF);
    }

    // DESCRIPTION:
    // This command switches the device to AUTO mode.
    // REPLY: ACK
    void auto_mode_on()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::AUTO_MODE_ON);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: ACK
    void input_check(uint8_t v)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::INPUT_CHECK, v);
    }

    // DESCRIPTION:
    // Bit0 loop mode enable, 0 = false, 1 = true
    // Bit1 is single clip/timeline, 0 = single clip, 1 = timeline
    // REPLY: ACK
    void set_playback_loop(const bool b_enable, const uint8_t mode = LoopMode::SINGLE_CLIP)
    {
        Serial.print(__func__); Serial.print(" : ");
        const uint8_t v = (uint8_t)b_enable | ((uint8_t)(mode & 0x01) << 1);
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SET_PLAYBACK_LOOP, v);
    }

    // DESCRIPTION:
    // 0 = Off
    // 1 = Freeze on last frame of Timeline (not clip)
    // 2 = Freeze on next clip of Timeline (not clip)
    // 3 = Show black
    // REPLY: ACK
    void set_stop_mode(const uint8_t stop_mode)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::SET_STOP_MODE, stop_mode);
    }

    // DESCRIPTION:
    // 2 Bytes for the length N of the clip name
    // N Bytes for each character of the clip name
    // 4 Byte in point timecode (format is FFSSMMHH)
    // 4 Byte out point timecode (format is FFSSMMHH)
    // REPLY: ACK
    // void append_preset()
    // {
    //     Serial.print(__func__); Serial.print(" : ");
    //     send(Cmd1::PRESET_SELECT_CONTROL, PresetSelectCtrl::APPEND_PRESET);
    // }


    // 6 - Sense Request

    // DESCRIPTION:
    // Request the type of time code data the device is generating,
    // based on the type SENSE of data required.
    // It will then respond according to the contents of DATA-1.
    // DATA-1 = 01: Request for GEN TC -> GEN TIME DATA 74.08 Respond
    // DATA-1 = 10: Request for GEN UB -> GEN UB DATA 74.09 Respond
    // DATA-1 = 11: Request for GEN TC & UB -> GEN TC & UB DATA 78.08 Respond
    // REPLY: based on DATA-1 as above
    void tc_gen_sense(const uint8_t data1)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::TIMECODE_GEN_SENSE, 0x01);
    }
    void ub_gen_sense(const uint8_t data1)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::TIMECODE_GEN_SENSE, 0x10);
    }
    void tc_ub_gen_sense(const uint8_t data1)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::TIMECODE_GEN_SENSE, 0x11);
    }

    // DESCRIPTION:
    // Requests the current in point.
    // See the CUE UP WITH DATA command for the time code return format.
    // REPLY: IN_DATA
    void in_data_sense()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::IN_DATA_SENSE);
    }

    // DESCRIPTION:
    // Requests the current out point.
    // See the CUE UP WITH DATA command for the time code return format.
    // REPLY: OUT_DATA
    void out_data_sense()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::OUT_DATA_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: A_IN_DATA
    void a_in_data_sense()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::A_IN_DATA_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: A_OUT_DATA
    void a_out_data_sense()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::A_OUT_DATA_SENSE);
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
    void status_sense(const uint8_t start = 0, const uint8_t size = 9)
    {
        Serial.print(__func__); Serial.print(" : ");
        const uint8_t v = size | (start << 4);
        send(Cmd1::SENSE_REQUEST, SenseRequest::STATUS_SENSE, v);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: SPEED_DATA
    void speed_sense()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::SPEED_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: PREROLL_TIME_DATA
    void preroll_time_sense()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::PREROLL_TIME_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: TIMER_MODE_DATA
    void timer_mode_sense()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::TIMER_MODE_SENSE);
    }

    // DESCRIPTION:
    // UNKNOWN
    // REPLY: RECORD_INHIBIT_STATUS
    void record_inhibit_sense()
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::SENSE_REQUEST, SenseRequest::RECORD_INHIBIT_SENSE);
    }


    // A - Advanced Media Protocol

    // DESCRIPTION:
    // 8-bit signed number of clips to skip from current clip
    // REPLY: ACK
    void auto_skip(const int8_t n)
    {
        Serial.print(__func__); Serial.print(" : ");
        send(Cmd1::ADVANCED_MEDIA_PRTCL, AdvancedMediaProtocol::AUTO_SKIP, (uint8_t)n);
    }

    // DESCRIPTION:
    // when x = 0, single clip request
    // when x = 1, # clips can be specified in the send data
    // REPLY: IDListing
    // void list_next_id()
    // {
    //     Serial.print(__func__); Serial.print(" : ");
    //     send(Cmd1::ADVANCED_MEDIA_PRTCL, AdvancedMediaProtocol::LIST_NEXT_ID);
    // }


    // Blackmagic Extensions

    // DESCRIPTION:
    // 16-bit little endian fractional position [0..65535]
    // REPLY: ACK
    // void bmd_seek_to_timeline_position(const uint16_t pos)
    // {
    //     Serial.print(__func__); Serial.print(" : ");
    //     send(Cmd1::SYSTEM_CONTROL, BlackmagicExtensions::SEEK_TO_TIMELINE_POSITION, pos);
    // }

    // DESCRIPTION:
    // One-byte signed integer, which is the number of clips to skip (negative for backwards).
    // REPLY: ACK
    // void bmd_seek_relative_clip(const int8_t index)
    // {
    //     Serial.print(__func__); Serial.print(" : ");
    //     send(Cmd1::SYSTEM_CONTROL, BlackmagicExtensions::SEEK_RELATIVE_CLIP, index);
    // }


    bool is_media_exist() const { return !res.sts.b_cassette_out; } // set if no ssd is present
    bool is_remote_enabled() const { return !res.sts.b_local; }     // set if remote is disabled (local control)
    bool is_disk_available() const { return res.sts.b_standby; }    // set if a disk is available
    bool is_stopping() const { return res.sts.b_stop; }
    bool is_rewinding() const { return res.sts.b_rewind; }
    bool is_forwarding() const { return res.sts.b_forward; }
    bool is_recoding() const { return res.sts.b_record; }
    bool is_playing() const { return res.sts.b_play; }
    bool is_servo_lock() const { return res.sts.b_servo_lock; }
    bool is_shuttle() const { return res.sts.b_shuttle; }
    bool is_jog() const { return res.sts.b_jog; }
    bool is_var() const { return res.sts.b_var; }
    bool is_reverse() const { return res.sts.b_direction; }   // clear if playback is forwarding, set if playback is reversing
    bool is_paused() const { return res.sts.b_still; }        // set if playback is paused, or if in input preview mode
    bool is_auto_mode() const { return res.sts.b_auto_mode; } // set if in Auto Mode
    bool is_a_out_set() const { return res.sts.b_a_out_set; }
    bool is_a_in_set() const { return res.sts.b_a_in_set; }
    bool is_out_set() const { return res.sts.b_out_set; }
    bool is_in_set() const { return res.sts.b_in_set; }
    bool is_select_ee() const { return res.sts.b_select_ee; }   // set if in input preview mode
    bool is_full_ee() const { return res.sts.b_full_ee; }
    bool is_lamp_still() const { return res.sts.b_lamp_still; } // set according to playback speed and direction
    bool is_lamp_fwd() const { return res.sts.b_lamp_fwd; }
    bool is_lamp_rev() const { return res.sts.b_lamp_rev; }
    bool is_near_eot() const { return res.sts.b_near_eot; }     // set if total space left on available SSDs is less than 3 minutes
    bool is_eot() const { return res.sts.b_eot; }               // set if total space left on available SSDs is less than 30 seconds


private:

    void send(uint8_t& crc)
    {
        SONY9PINREMOTE_STREAM_WRITE(crc);
        Serial.println(crc, HEX);
    }

    template <typename... Args>
    void send(uint8_t& crc, const uint8_t arg, Args&&... args)
    {
        SONY9PINREMOTE_STREAM_WRITE(arg);
        crc += arg;
        Serial.print(arg, HEX); Serial.print(" ");
        send(crc, util::forward<Args>(args)...);
    }

    template <typename Cmd2, typename... Args>
    void send(const Cmd1 cmd1, const Cmd2 cmd2, Args&&... args)
    {
        if (!ready()) return;
        res.next();
        uint8_t size = sizeof...(args);
        uint8_t header = (uint8_t)cmd1 | (size & 0x0F);
        uint8_t crc = header + (uint8_t)cmd2;
        SONY9PINREMOTE_STREAM_WRITE(header);
        SONY9PINREMOTE_STREAM_WRITE((uint8_t)cmd2);
        Serial.print("send data = ");
        Serial.print(header, HEX); Serial.print(" ");
        Serial.print((uint8_t)cmd2, HEX); Serial.print(" ");
        send(crc, util::forward<Args>(args)...);
    }

};


} // namespace sony9pin
} // namespace rs422
} // namespace serial
} // namespace ht


namespace Sony9PinRemote = ht::serial::rs422::sony9pin;
namespace Sony9PinDevice = Sony9PinRemote::DeviceType;
namespace Sony9PinSerial = Sony9PinRemote::serial;

#endif // HT_RS422_SONY9PINREMOTE_H
