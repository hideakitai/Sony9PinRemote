#include <Sony9PinRemote.h>

Sony9PinRemote::Controller deck;

void setup() {
    Serial.begin(115200);
    Serial1.begin(Sony9PinSerial::BAUDRATE, Sony9PinSerial::CONFIG);
    delay(2000);

    deck.attach(Serial1);

    // get device status
    deck.status_sense();
    if (deck.parse_until(1000))  // wait until response comes (timeout = 1000ms)
    {
        if (!deck.is_media_exist())
            Serial.println("ERROR: there is no media!");

        if (!deck.is_remote_enabled())
            Serial.println("ERROR: remote control is disabled!");

        if (!deck.is_disk_available())
            Serial.println("ERROR: removable media is not available!");

        if (!deck.is_stopping()) {
            deck.stop();
            deck.parse_until(1000);
        }

        deck.device_type();
        if (deck.parse_until(1000)) {
            Serial.print("device type = ");
            Serial.println(deck.device_type(), HEX);

            if (deck.device_type() == Sony9PinDevice::BLACKMAGIC_HYPERDECK_STUDIO_MINI_NTSC) {
                Serial.println("this device is BlackMagic HyperDeck Studio Mini NTSC");
            }
        } else
            Serial.println("ERROR: device type request failed!!");
    } else {
        Serial.println("ERROR: device status request failed!!");
    }
}

void loop() {
    deck.parse();

    // if previous command has completed (response has come)
    if (deck.ready()) {
        if (Serial.available() >= 2) {
            using namespace Sony9PinRemote;

            char c = Serial.read();
            while (Serial.available()) Serial.read();

            switch (c) {
                case 'p': deck.play(); break;
                case 's': deck.stop(); break;
                case 'P': deck.sync_play(); break;
                case 'n': deck.auto_skip(1); break;   // next clip
                case 'N': deck.auto_skip(-1); break;  // prev clip
                case 'c': deck.set_playback_loop(true, LoopMode::SINGLE_CLIP); break;
                case 't': deck.set_playback_loop(true, LoopMode::TIMELINE); break;
                case 'l': deck.set_playback_loop(false); break;
                case '0': deck.set_stop_mode(StopMode::OFF); break;
                case '1': deck.set_stop_mode(StopMode::FREEZE_ON_LAST_FRAME); break;
                case '2': deck.set_stop_mode(StopMode::FREEZE_ON_NEXT_CLIP); break;
                case '3': deck.set_stop_mode(StopMode::SHOW_BLACK); break;
                case '9': deck.status_sense(); break;
                default:
                    Serial.print("NO CMD: ");
                    Serial.println(c);
                    break;
            }
        }
    }
}
