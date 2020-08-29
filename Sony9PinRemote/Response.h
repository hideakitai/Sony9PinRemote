#ifndef HT_RS422_SONY9PINREMOTE_RESPONSE_H
#define HT_RS422_SONY9PINREMOTE_RESPONSE_H

#ifdef ARDUINO
    #include <Arduino.h>
#endif
#include <stdint.h>

#include "Types.h"

namespace ht {
namespace serial {
namespace rs422 {
namespace sony9pin {


    class Response
    {
        static constexpr uint8_t MAX_RESPONSE_SIZE {15 + 3};

        uint8_t buffer[MAX_RESPONSE_SIZE];
        uint8_t next_size {0};
        uint8_t curr_size {0};
        uint8_t b_parsing {false};

        uint16_t device_no {0};
        size_t err_count {0};

    public:

        Status sts;
        Errors err;

        Cmd1 cmd1() const { return (Cmd1)(buffer[0] & (uint8_t)HeaderMask::CMD1); }
        uint8_t cmd2() const { return buffer[1]; }

        bool available() const { return !empty() && (curr_size == next_size); }
        bool empty() const { return curr_size == 0; }
        bool busy() const { return b_parsing; }

        bool has_nak() const { return (cmd1() == Cmd1::SYSTEM_CONTROL_RETURN) && (cmd2() == SystemControlReturn::NAK); }
        bool has_device_type() const { return (cmd1() == Cmd1::SYSTEM_CONTROL_RETURN) && (cmd2() == SystemControlReturn::DEVICE_TYPE); }
        bool success() const { return available() && !has_nak(); }

        bool has_status() const { return (cmd1() == Cmd1::SENSE_RETURN) && (cmd2() == SenseReturn::STATUS_DATA); }

        uint16_t device_type() const { return device_no; }
        const Status& status() const { return sts; }
        const Errors& errors() const { return err; }
        bool error_count() const { return err_count; }

        uint8_t data(const uint8_t i) { return buffer[i + 2]; }
        const uint8_t* data() const { return buffer + 2; }
        uint8_t size() const { return next_size - 3; }

        void next()
        {
            clear();
            b_parsing = true;
        }

        void clear()
        {
            for (uint8_t i = 0; i < MAX_RESPONSE_SIZE; ++i) buffer[i] = 0;
            next_size = 0;
            curr_size = 0;
            b_parsing = false;
        }

        bool feed(const uint8_t d)
        {
            if (curr_size >= next_size) // unexpected response
            {
                next();
            }

            if (next_size == 0) // header byte
            {
                uint8_t type = d & (uint8_t)HeaderMask::CMD1;
                uint8_t size = d & (uint8_t)HeaderMask::SIZE;

                if ((type == (uint8_t)Cmd1::SYSTEM_CONTROL_RETURN) ||
                    (type == (uint8_t)Cmd1::SENSE_RETURN))
                {
                    next_size = size + 3; // header + cmd2 + size + checksum
                    buffer[curr_size++] = d;
                }
                else // this is not response headr
                {
                    Serial.println("response type error !");
                    clear();
                }
            }
            else if (curr_size < next_size)
            {
                buffer[curr_size++] = d;

                if (curr_size == next_size)
                {
                    uint8_t checksum = 0;
                    for (uint8_t i = 0; i < curr_size - 1; ++i)
                        checksum += buffer[i];

                    if (d == checksum)
                    {
                        b_parsing = false;
                        decode_response();
                        return true;
                    }
                    else
                    {
                        clear();
                        Serial.println("ERROR: checksum check failed!");
                    }
                }
            }
            else
            {
                Serial.println("ERROR: won't come here!");
                clear();
            }

            return false;
        }

        void print_nak()
        {
            Serial.print("NAK received ! err = ");
            Serial.println(buffer[2], BIN);
            if (err.b_unknown_cmd)    Serial.println("Unknown Command");
            if (err.b_checksum_error) Serial.println("Checksum Error");
            if (err.b_parity_error)   Serial.println("Parity Error");
            if (err.b_buffer_overrun) Serial.println("Buffer Overrun");
            if (err.b_framing_error)  Serial.println("Framing Error");
            if (err.b_timeout)        Serial.println("Timeout");
        }

        void print_status()
        {
            Serial.println("<Remote Status>");
            Serial.println("=================");
            Serial.print("Cassette Out : "); Serial.println(sts.b_cassette_out);
            Serial.print("Local        : "); Serial.println(sts.b_local);
            Serial.println("-----------------");
            Serial.print("Standby      : "); Serial.println(sts.b_standby);
            Serial.print("Stop         : "); Serial.println(sts.b_stop);
            Serial.print("Rewind       : "); Serial.println(sts.b_rewind);
            Serial.print("Forward      : "); Serial.println(sts.b_forward);
            Serial.print("Record       : "); Serial.println(sts.b_record);
            Serial.print("Play         : "); Serial.println(sts.b_play);
            Serial.println("-----------------");
            Serial.print("Servo Lock   : "); Serial.println(sts.b_servo_lock);
            Serial.print("Shuttle      : "); Serial.println(sts.b_shuttle);
            Serial.print("Jog          : "); Serial.println(sts.b_jog);
            Serial.print("Var          : "); Serial.println(sts.b_var);
            Serial.print("Direction    : "); Serial.println(sts.b_direction);
            Serial.print("Still        : "); Serial.println(sts.b_still);
            Serial.println("-----------------");
            Serial.print("Auto Mode    : "); Serial.println(sts.b_auto_mode);
            Serial.print("Aout Set     : "); Serial.println(sts.b_a_out_set);
            Serial.print("Ain Set      : "); Serial.println(sts.b_a_in_set);
            Serial.print("Out Set      : "); Serial.println(sts.b_out_set);
            Serial.print("In Set       : "); Serial.println(sts.b_in_set);
            Serial.println("-----------------");
            Serial.print("Select EE    : "); Serial.println(sts.b_select_ee);
            Serial.print("Full EE      : "); Serial.println(sts.b_full_ee);
            Serial.println("-----------------");
            Serial.print("Lamp Still   : "); Serial.println(sts.b_lamp_still);
            Serial.print("Lamp Fwd     : "); Serial.println(sts.b_lamp_fwd);
            Serial.print("Lamp Rev     : "); Serial.println(sts.b_lamp_rev);
            Serial.println("-----------------");
            Serial.print("Near EOT     : "); Serial.println(sts.b_near_eot);
            Serial.print("EOT          : "); Serial.println(sts.b_eot);
            Serial.println("=================");
        }

    private:

        void decode_response()
        {
            if (has_nak())
            {
                err_count++;
                err.b_unknown_cmd    = buffer[2] & NakMask::UNKNOWN_CMD;
                err.b_checksum_error = buffer[2] & NakMask::CHECKSUM_ERROR;
                err.b_parity_error   = buffer[2] & NakMask::PARITY_ERROR;
                err.b_buffer_overrun = buffer[2] & NakMask::BUFFER_OVERRUN;
                err.b_framing_error  = buffer[2] & NakMask::FRAMING_ERROR;
                err.b_timeout        = buffer[2] & NakMask::TIMEOUT;
                print_nak();
            }
            else if (has_device_type())
            {
                device_no = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
            }
            else if (has_status() && (size() == 9))
            {
                // byte 0
                sts.b_cassette_out = data(0) & StatusMask::CASSETTE_OUT;
                sts.b_local        = data(0) & StatusMask::LOCAL;
                // byte 1
                sts.b_standby = data(1) & StatusMask::STANDBY;
                sts.b_stop    = data(1) & StatusMask::STOP;
                sts.b_rewind  = data(1) & StatusMask::REWIND;
                sts.b_forward = data(1) & StatusMask::FORWARD;
                sts.b_record  = data(1) & StatusMask::RECORD;
                sts.b_play    = data(1) & StatusMask::PLAY;
                // byte 2
                sts.b_servo_lock = data(2) & StatusMask::SERVO_LOCK;
                sts.b_shuttle    = data(2) & StatusMask::SHUTTLE;
                sts.b_jog        = data(2) & StatusMask::JOG;
                sts.b_var        = data(2) & StatusMask::VAR;
                sts.b_direction  = data(2) & StatusMask::DIRECTION;
                sts.b_still      = data(2) & StatusMask::STILL;
                // byte 3
                sts.b_auto_mode = data(3) & StatusMask::AUTO_MODE;
                sts.b_a_out_set = data(3) & StatusMask::A_OUT_SET;
                sts.b_a_in_set  = data(3) & StatusMask::A_IN_SET;
                sts.b_out_set   = data(3) & StatusMask::OUT_SET;
                sts.b_in_set    = data(3) & StatusMask::IN_SET;
                // byte 4
                sts.b_select_ee = data(4) & StatusMask::SELECT_EE;
                sts.b_full_ee   = data(4) & StatusMask::FULL_EE;
                // byte 6
                sts.b_lamp_still = data(6) & StatusMask::LAMP_STILL;
                sts.b_lamp_fwd   = data(6) & StatusMask::LAMP_FWD;
                sts.b_lamp_rev   = data(6) & StatusMask::LAMP_REV;
                // byte 8
                sts.b_near_eot = data(8) & StatusMask::NEAR_EOT;
                sts.b_eot      = data(8) & StatusMask::EOT;
                print_status();
            }
        }

    };

} // namespace sony9pin
} // namespace rs422
} // namespace serial
} // namespace ht

#endif // HT_RS422_SONY9PINREMOTE_RESPONSE_H
