// #define SONY9PINREMOTE_DEBUGLOG_ENABLE
#include <Sony9PinRemote.h>

Sony9PinRemote::Controller deck;

void setup() {
    Serial.begin(115200);
    Serial5.begin(Sony9PinSerial::BAUDRATE, Sony9PinSerial::CONFIG);
    delay(2000);

    deck.attach(Serial5);

    deck.status_sense();
    if (deck.parse_until(1000))
        deck.print_status();
    else
        Serial.println("Status Sense: No Response");

    deck.device_type_request();
    if (deck.parse_until(1000)) {
        Serial.print("device type = ");
        Serial.println(deck.device_type(), HEX);
    } else
        Serial.println("Device Type: No Response");

    deck.current_time_sense_timer1();
    if (deck.parse_until(1000))
        deck.print_timecode_userbits();  // Black Magic Hyperdeck mini returns 8 bytes
    else
        Serial.println("Current Time Timer1: No Response");
}

void loop() {
}
