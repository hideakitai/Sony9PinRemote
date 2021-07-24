// #define SONY9PINREMOTE_DEBUGLOG_ENABLE
#include <Sony9PinRemote.h>

Sony9PinRemote::Controller deck;

void setup() {
    Serial.begin(115200);
    Serial1.begin(Sony9PinSerial::BAUDRATE, Sony9PinSerial::CONFIG);
    delay(2000);

    deck.attach(Serial1);

    // get device status
    deck.status_sense();
    if (deck.parse_until(1000)) {  // wait until response comes (timeout = 1000ms)
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

        deck.device_type_request();
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
    // if previous command has completed (response has come)
    if (deck.ready()) {
        static bool b = false;
        if (b)
            deck.play();
        else
            deck.stop();
        b = !b;
        delay(2000);
    }

    if (deck.parse()) {        // if some reply has come
        if (!deck.ack()) {     // if the reply is not ack
            deck.print_nak();  // print nak
        }
    }
}
