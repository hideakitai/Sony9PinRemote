# Sony9PinRemote

RS422 Sony 9-Pin Protocol Remote Controller of VTRs for Arduino

## Usage

```C++
#include <Sony9PinRemote.h>

Sony9PinRemote::Controller deck;

void setup()
{
    // serial should be set and attach as follows
    Serial1.begin(Sony9PinSerial::BAUDRATE, Sony9PinSerial::CONFIG);
    deck.attach(Serial1);

    // get device status
    deck.status_sense();
    if (deck.parse_until(1000)) // wait until response comes (timeout = 1000ms)
    {
        if (!deck.is_media_exist())
            Serial.println("ERROR: there is no media!");

        if (!deck.is_remote_enabled())
            Serial.println("ERROR: remote control is disabled!");

        if (!deck.is_disk_available())
            Serial.println("ERROR: removable media is not available!");

        if (!deck.is_stopping())
        {
            deck.stop();
            deck.parse_until(1000);
        }

        deck.device_type();
        if (deck.parse_until(1000))
        {
            Serial.print("device type = ");
            Serial.println(deck.device(), HEX);
        }
        else
            Serial.println("ERROR: device type request failed!!");
    }
    else
    {
        Serial.println("ERROR: device status request failed!!");
    }
}

void loop()
{
    // receive and parse reponse packet
    deck.parse();

    // if previous command has completed (response has come)
    if (deck.ready())
    {
        static bool b = false;
        if (b) deck.play();
        else   deck.stop();
        b = !b;
        delay(2000);
    }
}
```


## APIs

```C++
// Sony9PinRemote::Controller
void attach(StreamType& s);
void parse();
bool parse_until(const uint32_t timeout_ms);
bool ready() const;
bool available() const;
uint16_t device() const;
const Status& status() const;
const Errors& errors() const;
size_t error_count() const;
// 0 - System Control
void local_disable();
void device_type();
void lock_enable();
// 2 - Transport Control
void stop();
void play();
void record();
void standby_off();
void standby_on();
void eject();
void fast_forward();
void jog_forward(const uint8_t data1, const uint8_t data2 = 0);
void var_forward(const uint8_t data1, const uint8_t data2 = 0);
void shuttle_forward(const uint8_t data1, const uint8_t data2 = 0);
void fast_reverse();
void rewind();
void jog_reverse(const uint8_t data1, const uint8_t data2 = 0);
void var_reverse(const uint8_t data1, const uint8_t data2 = 0);
void shuttle_reverse(const uint8_t data1, const uint8_t data2 = 0);
void preroll();
void cue_data(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames);
void sync_play();
void prog_speed_play_plus(const uint8_t v);
void prog_speed_play_minus(const uint8_t v);
void preview();
void review();
void outpoint_preview();
void dmc_set_fwd(const uint8_t data1, const uint8_t data2);
void dmc_set_rev(const uint8_t data1, const uint8_t data2);
void full_ee_off();
void full_ee_on();
void select_ee_on();
void clearPlaylist();
// 4 - Preset/Select Control
void in_entry();
void out_entry();
void in_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames);
void out_data_preset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames);
void in_shift_plus();
void in_shift_minus();
void out_shift_plus();
void out_shift_minus();
void in_reset();
void out_reset();
void a_in_reset();
void a_out_reset();
void preroll_prset(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint8_t frames);
void auto_mode_off();
void auto_mode_on();
void set_playback_loop(const bool b_enable, const bool b_sel);
// 6 - Sense Request
void tc_gen_sense(const uint8_t data1);
void ub_gen_sense(const uint8_t data1);
void tc_ub_gen_sense(const uint8_t data1);
void in_data_sense();
void out_data_sense();
void a_in_data_sense();
void a_out_data_sense();
void status_sense(const uint8_t start = 0, const uint8_t size = 9);
void speed_sense();
void preroll_time_sense();
void timer_mode_sense();
void record_inhibit_sense();
// status check
bool is_media_exist() const;
bool is_remote_enabled() const;
bool is_disk_available() const;
bool is_stopping() const;
bool is_rewinding() const;
bool is_forwarding() const;
bool is_recoding() const;
bool is_playing() const;
bool is_servo_lock() const;
bool is_shuttle() const;
bool is_jog() const;
bool is_var() const;
bool is_reverse() const;
bool is_paused() const;
bool is_auto_mode() const;
bool is_a_out_set() const;
bool is_a_in_set() const;
bool is_out_set() const;
bool is_in_set() const;
bool is_select_ee() const;
bool is_full_ee() const;
bool is_lamp_still() const;
bool is_lamp_fwd() const;
bool is_lamp_rev() const;
bool is_near_eot() const;
bool is_eot() const;
```

### Response Structs

```C++
struct Errors
{
    bool b_unknown_cmd;
    bool b_checksum_error;
    bool b_parity_error;
    bool b_buffer_overrun;
    bool b_framing_error;
    bool b_timeout;
};

struct Status
{
    // byte 0
    bool b_cassette_out;
    bool b_local;
    // byte 1
    bool b_standby;
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
    bool b_direction;
    bool b_still;
    // byte 3
    bool b_auto_mode;
    bool b_a_out_set;
    bool b_a_in_set;
    bool b_out_set;
    bool b_in_set;
    // byte 4
    bool b_select_ee;
    bool b_full_ee;
    // byte 6
    bool b_lamp_still;
    bool b_lamp_fwd;
    bool b_lamp_rev;
    // byte 8
    bool b_near_eot;
    bool b_eot;
};
```

## License

MIT
