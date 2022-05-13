/*
  Copyright (c) 2021 Alan Yorinks All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
  Version 3 as published by the Free Software Foundation; either
  or (at your option) any later version.
  This library is distributed in the hope that it will be useful,f
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
  along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include <WiFiNINA.h>
#include "Telemetrix4Connect2040.h"
#include <Wire.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include <Arduino_LSM6DSOX.h>
#include <PDM.h>
#include <NeoPixelConnect.h>
#include <NanoConnectHcSr04.h>
#include <dhtnew.h>
#include <SPI.h>

extern "C" {
#include <hardware/watchdog.h>
};

// Modify the next two lines to match your network values
const char *ssid = "YOUR_SSID";
const char *password = "YOUR_PASSWORD";

// Default ip port value.
// Set the telemetrix or telemetrix-aio port to the same value

uint16_t PORT = 31335;

WiFiServer wifiServer(PORT);

// We define these here to provide a forward reference.
// If you add a new command, you must add the command handler
// here as well.

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void analog_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void are_you_there();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void i2c_begin();

extern void i2c_read();

extern void i2c_write();

extern void sonar_new();

extern void imu_enable();

extern void microphone_enable();

extern void rgb_write();

extern void stop_all_reports();

extern void set_analog_scanning_interval();

extern void enable_all_reports();

extern void reset_data_structures();

extern void init_neo_pixels();

extern void show_neo_pixels();

extern void set_neo_pixel();

extern void clear_all_neo_pixels();

extern void fill_neo_pixels();

extern void init_spi();

extern void write_blocking_spi();

extern void read_blocking_spi();

extern void set_format_spi();

extern void spi_cs_control();

extern void dht_new();

// This value must be the same as specified when instantiating the
// telemetrix client. The client defaults to a value of 1.
// This value is used for the client to auto-discover and to
// connect to a specific board regardless of the current com port
// it is currently connected to.
#define ARDUINO_ID 1

// Commands -received by this sketch
// Add commands retaining the sequential numbering.
// The order of commands here must be maintained in the command_table.
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define ANALOG_WRITE 3
#define MODIFY_REPORTING 4 // mode(all, analog, or digital), pin, enable or disable
#define GET_FIRMWARE_VERSION 5
#define ARE_U_THERE 6
#define SERVO_ATTACH 7
#define SERVO_WRITE 8
#define SERVO_DETACH 9
#define I2C_BEGIN 10
#define I2C_READ 11
#define I2C_WRITE 12
#define SONAR_NEW 13
#define RGB_WRITE 14
#define STOP_ALL_REPORTS 15
#define SET_ANALOG_SCANNING_INTERVAL 16
#define ENABLE_ALL_REPORTS 17
#define RESET_DATA_STRUCTURES 18
#define IMU_ENABLE 19
#define MICROPHONE_ENABLE 20
#define INIT_NEOPIXELS 21
#define SHOW_NEOPIXELS 22
#define SET_NEOPIXEL 23
#define CLEAR_NEOPIXELS 24
#define FILL_NEOPIXELS 25
#define SPI_INIT 26
#define SPI_WRITE_BLOCKING 27
#define SPI_READ_BLOCKING 28
#define SPI_SET_FORMAT 29
#define SPI_CS_CONTROL 30
#define DHT_NEW = 31

// When adding a new command update the command_table.
// The command length is the number of bytes that follow
// the command byte itself, and does not include the command
// byte in its length.
// The command_func is a pointer the command's function.
struct command_descriptor {
    // a pointer to the command processing function
    void (*command_func)(void);
};

// An array of pointers to the command functions

// If you add new commands, make sure to extend the siz of this
// array.
command_descriptor command_table[] =
        {
                {serial_loopback},
                {set_pin_mode},
                {digital_write},
                {analog_write},
                {modify_reporting},
                {get_firmware_version},
                {are_you_there},
                {servo_attach},
                {servo_write},
                {servo_detach},
                {i2c_begin},
                {i2c_read},
                {i2c_write},
                {sonar_new},
                {rgb_write},
                {stop_all_reports},
                {set_analog_scanning_interval},
                {enable_all_reports},
                {reset_data_structures},
                {imu_enable},
                {microphone_enable},
                {init_neo_pixels},
                {show_neo_pixels},
                {set_neo_pixel},
                {clear_all_neo_pixels},
                {fill_neo_pixels},
                {&init_spi},
                {&write_blocking_spi},
                {&read_blocking_spi},
                {&set_format_spi},
                {&spi_cs_control},
                {&dht_new}
        };

// Input pin reporting control sub commands (modify_reporting)
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

// maximum length of a command in bytes
#define MAX_COMMAND_LENGTH 30

// Pin mode definitions

// INPUT defined in Arduino.h = 0
// OUTPUT defined in Arduino.h = 1
// INPUT_PULLUP defined in Arduino.h = 2
// The following are defined for arduino_telemetrix (AT)
#define AT_ANALOG 3
#define AT_MODE_NOT_SET 255

// maximum number of pins supported
#define MAX_DIGITAL_PINS_SUPPORTED 28
#define MAX_ANALOG_PINS_SUPPORTED 8

// Reports - sent from this sketch
#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT ANALOG_WRITE
#define FIRMWARE_REPORT 5
#define I_AM_HERE 6
#define SERVO_UNAVAILABLE 7
#define I2C_TOO_FEW_BYTES_RCVD 8
#define I2C_TOO_MANY_BYTES_RCVD 9
#define I2C_READ_REPORT 10
#define SONAR_DISTANCE 11
#define IMU_REPORT 12
#define MICROPHONE_REPORT 13
#define DHT_REPORT 14
#define SPI_REPORT 15
#define DEBUG_PRINT 99

// firmware version - update this when bumping the version
#define FIRMWARE_MAJOR 1
#define FIRMWARE_MINOR 0
#define FIRMWARE_PATCH 1

// A buffer to hold i2c report data
byte i2c_report_message[64];

byte spi_report_message[64];

bool stop_reports = false; // a flag to stop sending all report messages

bool rebooting = false;

// To translate a pin number from an integer value to its analog pin number
// equivalent, this array is used to look up the value to use for the pin.
int analog_read_pins[4] = {A0, A1, A2, A3};
// NinaPin nina_analog_read_pins[4] = {A4, A5, A6, A7};

// a descriptor for digital pins
struct pin_descriptor {
    byte pin_number;
    byte pin_mode;
    bool reporting_enabled; // If true, then send reports if an input pin
    int last_value;         // Last value read for input mode
};

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

// a descriptor for digital pins
struct analog_pin_descriptor {
    byte pin_number;
    byte pin_mode;
    bool reporting_enabled; // If true, then send reports if an input pin
    int last_value;         // Last value read for input mode
    int differential;       // difference between current and last value needed
    // to generate a report
};

// an array of analog_pin_descriptors
analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

unsigned long current_millis;  // for analog input loop
unsigned long previous_millis; // for analog input loop
uint8_t analog_sampling_interval = 19;

// servo management
Servo servos[MAX_SERVOS];

// this array allows us to retrieve the servo object
// associated with a specific pin number
byte pin_to_servo_index_map[MAX_SERVOS];

// HC-SR04 Sonar Management
#define MAX_SONARS 6

struct Sonar {
    uint8_t trigger_pin;
    unsigned int last_value;
    NanoConnectHcSr04 *usonic;
};

// an array of sonar objects
Sonar sonars[MAX_SONARS];

byte sonars_index = 0; // index into sonars struct

// used for scanning the sonar devices.
byte last_sonar_visited = 0;

unsigned long sonar_current_millis;  // for analog input loop
unsigned long sonar_previous_millis; // for analog input loop
uint8_t sonar_scan_interval = 33;    // Milliseconds between sensor pings
// (29ms is about the min to avoid = 19;

// init dht command offsets
#define DHT_DATA_PIN 1

/* Maximum number of DHT devices */
#define MAX_DHTS 2

// DHT Report sub-types
#define DHT_DATA 0
#define DHT_READ_ERROR 1

boolean imu_enabled = false;
boolean imu_error = false;

// microphone variables
boolean microphone_enabled = false;
boolean microphone_error = false;
short sampleBuffer[512];
volatile int samplesRead;
//volatile bool get_microphone_samples = false;

NeoPixelConnect *np;

// DHT related
struct DHT {
    uint8_t pin;
    uint8_t dht_type;
    unsigned int last_value;
    DHTNEW *dht_sensor;
};

// an array of dht objects]
DHT dhts[MAX_DHTS];

byte dht_index = 0; // index into dht struct

unsigned long dht_current_millis;      // for analog input loop
unsigned long dht_previous_millis;     // for analog input loop
unsigned int dht_scan_interval = 2200; // scan dht's every 2 seconds

// buffer to hold incoming command data
byte command_buffer[MAX_COMMAND_LENGTH];

// wifi client connection
WiFiClient client;

// A method to send debug data across the serial link
void send_debug_info(byte id, int value) {
    byte debug_buffer[5] = {(byte) 4, (byte) DEBUG_PRINT, 0, 0, 0};
    debug_buffer[2] = id;
    debug_buffer[3] = highByte(value);
    debug_buffer[4] = lowByte(value);
    client.write(debug_buffer, 5);
}

// command functions
void serial_loopback() {
    byte loop_back_buffer[3] = {2, (byte) SERIAL_LOOP_BACK, command_buffer[0]};
    client.write(loop_back_buffer, 3);
}

void set_pin_mode() {
    byte pin;
    byte mode;
    pin = command_buffer[0];
    mode = command_buffer[1];

    switch (mode) {
        case INPUT:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[2];
            pinMode(pin, INPUT);
            break;
        case INPUT_PULLUP:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[2];
            pinMode(pin, INPUT_PULLUP);
            break;
        case OUTPUT:
            the_digital_pins[pin].pin_mode = mode;
            if (pin == 25 || pin == 26 || pin == 27) {
                break;
            }
            pinMode(pin, OUTPUT);
            break;
        case AT_ANALOG:
            the_analog_pins[pin].pin_mode = mode;
            the_analog_pins[pin].differential = (command_buffer[2] << 8) + command_buffer[3];
            the_analog_pins[pin].reporting_enabled = command_buffer[4];
            break;
        default:
            break;
    }
}

void set_analog_scanning_interval() {
    analog_sampling_interval = command_buffer[0];
}

void digital_write() {
    byte pin;
    byte value;
    pin = command_buffer[0];
    value = command_buffer[1];

    if (pin == 26 && value == 0) {
        digitalWrite(LEDB, LOW);
    } else if (pin == 26 && value == 1) {
        digitalWrite(LEDB, HIGH);
    } else if (pin == 25 && value == 0) {
        digitalWrite(LEDG, LOW);
    } else if (pin == 25 && value == 1) {
        digitalWrite(LEDG, HIGH);
    } else if (pin == 27 && value == 0) {
        digitalWrite(LEDR, LOW);
    } else if (pin == 27 && value == 1) {
        digitalWrite(LEDR, HIGH);
    } else {

        digitalWrite(pin, value);
    }

}

void analog_write() {
    // command_buffer[0] = PIN, command_buffer[1] = value_msb,
    // command_buffer[2] = value_lsb
    int pin; // command_buffer[0]
    int value;

    pin = (int) command_buffer[0];

    value = (int) command_buffer[1];
    if (pin == 26) {
        analogWrite(LEDB, value);
    } else if (pin == 25) {
        analogWrite(LEDG, value);
    } else if (pin == 27) {
        analogWrite(LEDR, value);
    } else {
        analogWrite(pin, value);
    }
}

void modify_reporting() {
    int pin = command_buffer[1];

    switch (command_buffer[0]) {
        case REPORTING_DISABLE_ALL:
            for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
                the_digital_pins[i].reporting_enabled = false;
            }
            for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
                the_analog_pins[i].reporting_enabled = false;
            }
            break;
        case REPORTING_ANALOG_ENABLE:
            if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET) {
                the_analog_pins[pin].reporting_enabled = true;
            }
            break;
        case REPORTING_ANALOG_DISABLE:
            if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET) {
                the_analog_pins[pin].reporting_enabled = false;
            }
            break;
        case REPORTING_DIGITAL_ENABLE:
            if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET) {
                the_digital_pins[pin].reporting_enabled = true;
            }
            break;
        case REPORTING_DIGITAL_DISABLE:
            if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET) {
                the_digital_pins[pin].reporting_enabled = false;
            }
            break;
        default:
            break;
    }
}

void reset_data_structures() {
    stop_all_reports();

    delay(100);
    if (command_buffer[0]) {
        rebooting = true;
        digitalWrite(LEDB, HIGH); //BLUE
        watchdog_reboot(0, 0, 0);
        watchdog_enable(100, 0);
        client.stop();
        while (1) {
            delay(3);
        }
    }
    // detach any attached servos
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (servos[i].attached() == true) {
            servos[i].detach();
        }
    }

    sonars_index = 0; // reset the index into the sonars array

    memset(sonars, 0, sizeof(sonars));

    dht_index = 0;

    memset(dhts, 0, sizeof(dhts));
}

void get_firmware_version() {
    byte report_message[5] = {4, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR, FIRMWARE_PATCH};
    client.write(report_message, 5);
}

void are_you_there() {
    byte report_message[3] = {2, I_AM_HERE, ARDUINO_ID};

    client.write(report_message, 3);
}

/***************************************************
   Servo Commands
 **************************************************/

// Find the first servo that is not attached to a pin
// This is a helper function not called directly via the API
int find_servo() {
    int index = -1;
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (servos[i].attached() == false) {
            index = i;
            break;
        }
    }
    return index;
}

void servo_attach() {

    byte pin = command_buffer[0];
    int servo_found = -1;

    int minpulse = (command_buffer[1] << 8) + command_buffer[2];
    int maxpulse = (command_buffer[3] << 8) + command_buffer[4];

    // find the first available open servo
    servo_found = find_servo();
    if (servo_found != -1) {
        pin_to_servo_index_map[servo_found] = pin;
        servos[servo_found].attach(pin, minpulse, maxpulse);
    } else {
        // no open servos available, send a report back to client
        byte report_message[2] = {SERVO_UNAVAILABLE, pin};
        client.write(report_message, 2);
    }
}

// set a servo to a given angle
void servo_write() {
    byte pin = command_buffer[0];
    int angle = command_buffer[1];
    servos[0].write(angle);
    // find the servo object for the pin
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (pin_to_servo_index_map[i] == pin) {

            servos[i].write(angle);
            return;
        }
    }
}

// detach a servo and make it available for future use
void servo_detach() {
    byte pin = command_buffer[0];

    // find the servo object for the pin
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (pin_to_servo_index_map[i] == pin) {

            pin_to_servo_index_map[i] = -1;
            servos[i].detach();
        }
    }
}

/***********************************
   i2c functions
 **********************************/

void i2c_begin() {
    Wire.begin();
}


void i2c_read() {
    // data in the incoming message:
    // address, [0]
    // register, [1]
    // number of bytes, [2]
    // stop transmitting flag [3]

    int message_size = 0;
    byte address = command_buffer[0];
    byte the_register = command_buffer[1];


    Wire.beginTransmission(address);
    Wire.write((byte) the_register);
    Wire.endTransmission(command_buffer[3]);      // default = true
    Wire.requestFrom(address, command_buffer[2]); // all bytes are returned in requestFrom

    // check to be sure correct number of bytes were returned by slave
    if (command_buffer[2] < Wire.available()) {
        byte report_message[4] = {3, I2C_TOO_FEW_BYTES_RCVD, 1, address};
        client.write(report_message, 4);
        return;
    } else if (command_buffer[2] > Wire.available()) {
        byte report_message[4] = {3, I2C_TOO_MANY_BYTES_RCVD, 1, address};
        client.write(report_message, 4);
        return;
    }

    // packet length
    i2c_report_message[0] = command_buffer[2] + 4;

    // report type
    i2c_report_message[1] = I2C_READ_REPORT;

    // number of bytes read
    i2c_report_message[2] = command_buffer[2]; // number of bytes

    // device address
    i2c_report_message[3] = address;

    // device register
    i2c_report_message[4] = the_register;

    // append the data that was read
    for (message_size = 0; message_size < command_buffer[2] && Wire.available(); message_size++) {
        i2c_report_message[5 + message_size] = Wire.read();
    }
    // send slave address, register and received bytes

    for (int i = 0; i < message_size + 5; i++) {
        client.write(i2c_report_message[i]);
    }
}

void i2c_write() {
    // command_buffer[0] is the number of bytes to send
    // command_buffer[1] is the device address
    // command_buffer[2] is the i2c port
    // additional bytes to write= command_buffer[3..];

    Wire.beginTransmission(command_buffer[1]);

    // write the data to the device
    for (int i = 0; i < command_buffer[0]; i++) {
        Wire.write(command_buffer[i + 2]);
    }
    Wire.endTransmission();
    delayMicroseconds(70);
}

/***********************************
   HC-SR04 adding a new device
 **********************************/

void sonar_new() {
    // command_buffer[0] = trigger pin,  command_buffer[1] = echo pin
    sonars[sonars_index].usonic = new NanoConnectHcSr04((uint8_t) command_buffer[0], (uint8_t) command_buffer[1], pio0,
                                                        1);
    sonars[sonars_index].trigger_pin = command_buffer[0];
    sonars_index++;
}

void rgb_write() {

}

// enable or disable the IMU
void imu_enable() {
    // command_buffer[0] = enable/disable flag (boolean)
    if (command_buffer[0]) {
        if (!IMU.begin()) {
            imu_error = true;
        } else {
            imu_enabled = true;
        }
    } else {
        imu_enabled = false;
        IMU.end();
    }
}

void onPDMdata() {

    // Query the number of available bytes
    int bytesAvailable = PDM.available();

    // Read into the sample buffer
    PDM.read(sampleBuffer, bytesAvailable);

    // 16-bit, 2 bytes per sample
    samplesRead = bytesAvailable / 2;

}

void microphone_enable() {
    if (command_buffer[0]) {
        microphone_enabled = true;
    } else {
        microphone_enabled = false;
    }
}

void init_neo_pixels() {
    byte pin_number, num_pixels;

    pin_number = command_buffer[0];
    num_pixels = command_buffer[1];

    np = new NeoPixelConnect(pin_number, num_pixels);
}

void show_neo_pixels() {
    np->neoPixelShow();
}

void set_neo_pixel() {

    np->neoPixelSetValue(command_buffer[0], command_buffer[1],
                         command_buffer[2], command_buffer[3],
                         command_buffer[4]);
}

void clear_all_neo_pixels() {
    np->neoPixelClear(command_buffer[0]);
}

void fill_neo_pixels() {
    np->neoPixelFill(command_buffer[0], command_buffer[1],
                     command_buffer[2], command_buffer[3]);
}

// initialize the SPI interface
void init_spi() {

    int cs_pin;

    //Serial.print(command_buffer[1]);
    // initialize chip select GPIO pins
    for (int i = 0; i < command_buffer[0]; i++) {
        cs_pin = command_buffer[1 + i];
        // Chip select is active-low, so we'll initialise it to a driven-high state
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);
    }
    SPI.begin();
}

// write a number of blocks to the SPI device
void write_blocking_spi() {
    int num_bytes = command_buffer[0];

    for (int i = 0; i < num_bytes; i++) {
        SPI.transfer(command_buffer[1 + i]);
    }
}

// read a number of bytes from the SPI device
void read_blocking_spi() {
    // command_buffer[0] == number of bytes to read
    // command_buffer[1] == read register

    // spi_report_message[0] = length of message including this element
    // spi_report_message[1] = SPI_REPORT
    // spi_report_message[2] = register used for the read
    // spi_report_message[3] = number of bytes returned
    // spi_report_message[4..] = data read

    // configure the report message
    // calculate the packet length
    spi_report_message[0] = command_buffer[0] + 3; // packet length
    spi_report_message[1] = SPI_REPORT;
    spi_report_message[2] = command_buffer[1]; // register
    spi_report_message[3] = command_buffer[0]; // number of bytes read

    // write the register out. OR it with 0x80 to indicate a read
    SPI.transfer(command_buffer[1] | 0x80);

    // now read the specified number of bytes and place
    // them in the report buffer
    for (int i = 0; i < command_buffer[0]; i++) {
        spi_report_message[i + 4] = SPI.transfer(0x00);
    }
    client.write(spi_report_message, command_buffer[0] + 4);
}

// modify the SPI format
void set_format_spi() {

#if defined(__AVR__)
    SPISettings(command_buffer[0], command_buffer[1], command_buffer[2]);
#else
    BitOrder b;

    if (command_buffer[1]) {
        b = MSBFIRST;
    } else {
        b = LSBFIRST;
    }
    SPISettings(command_buffer[0], b, command_buffer[2]);
#endif
}

// set the SPI chip select line
void spi_cs_control() {
    int cs_pin = command_buffer[0];
    int cs_state = command_buffer[1];
    digitalWrite(cs_pin, cs_state);
}

void dht_new() {
    if (dht_index < MAX_DHTS) {
        dhts[dht_index].dht_sensor = new DHTNEW(command_buffer[0]);

        dhts[dht_index].pin = command_buffer[0];
        dhts[dht_index].dht_type = command_buffer[1];
        dht_index++;
    }

}

void stop_all_reports() {
    stop_reports = true;
    delay(20);
    client.flush();
}

void enable_all_reports() {
    client.flush();
    stop_reports = false;
    delay(20);
}


void get_next_command() {
    byte command;
    byte packet_length;
    command_descriptor command_entry;

    // clear the command buffer
    memset(command_buffer, 0, sizeof(command_buffer));

    // if there is no command waiting, then return
    if (not client.available()) {
        return;
    }
    // get the packet length
    packet_length = (byte) client.read();

    while (not client.available()) {
        delay(1);
    }

    // get the command byte
    command = (byte) client.read();


    // uncomment the next line to see the packet length and command
    // send_debug_info(packet_length, command);
    command_entry = command_table[command];

    if (packet_length > 1) {
        // get the data for that command
        for (int i = 0; i < packet_length - 1; i++) {
            // need this delay or data read is not correct
            while (not client.available()) {
                delay(1);
            }
            command_buffer[i] = (byte) client.read();
            // uncomment out to see each of the bytes following the command
            // send_debug_info(i, command_buffer[i]);
        }
    }
    command_entry.command_func();
}

void scan_digital_inputs() {
    byte value;

    // report message

    // byte 0 = packet length
    // byte 1 = report type
    // byte 2 = pin number
    // byte 3 = value
    byte report_message[4] = {3, DIGITAL_REPORT, 0, 0};

    for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        if (the_digital_pins[i].pin_mode == INPUT ||
            the_digital_pins[i].pin_mode == INPUT_PULLUP) {
            if (the_digital_pins[i].reporting_enabled) {
                // if the value changed since last read
                value = (byte) digitalRead(the_digital_pins[i].pin_number);
                if (value != the_digital_pins[i].last_value) {
                    the_digital_pins[i].last_value = value;
                    report_message[2] = (byte) i;
                    report_message[3] = value;
                    client.write(report_message, 4);
                }
            }
        }
    }
}

void scan_analog_inputs() {
    int value;

    // report message

    // byte 0 = packet length
    // byte 1 = report type
    // byte 2 = pin number
    // byte 3 = high order byte of value
    // byte 4 = low order byte of value

    byte report_message[5] = {4, ANALOG_REPORT, 0, 0, 0};

    uint8_t adjusted_pin_number;
    int differential;

    current_millis = millis();
    if (current_millis - previous_millis > analog_sampling_interval) {
        previous_millis = current_millis;

        for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
            if (the_analog_pins[i].pin_mode == AT_ANALOG) {
                if (the_analog_pins[i].reporting_enabled) {
                    // if the value changed since last read
                    // adjust pin number for the actual read
                    if (i <= 3) {
                        value = analogRead((uint8_t)(analog_read_pins[i]));
                    } else {
                        switch (i) {
                            case 4:
                                value = analogRead(A4);
                                break;
                            case 5:
                                value = analogRead(A5);
                                break;
                            case 6:
                                value = analogRead(A6);
                                break;
                            case 7:
                                value = analogRead(A7);
                                break;
                            default:
                                break;
                        }
                    }

                    differential = abs(value - the_analog_pins[i].last_value);
                    if (differential >= the_analog_pins[i].differential) {
                        //trigger value achieved, send out the report
                        the_analog_pins[i].last_value = value;
                        // input_message[1] = the_analog_pins[i].pin_number;
                        report_message[2] = (byte) i;
                        report_message[3] = highByte(value); // get high order byte
                        report_message[4] = lowByte(value);
                        client.write(report_message, 5);
                        delay(1);
                    }
                }
            }
        }
    }

}

void scan_dhts() {
    // prebuild report for valid data
    // reuse the report if a read command fails

    // data returned is in floating point form - 4 bytes
    // each for humidity and temperature

    // byte 0 = packet length
    // byte 1 = report type
    // byte 2 = report sub type - DHT_DATA or DHT_ERROR
    // byte 3 = pin number
    // byte 4 = humidity positivity flag 0=positive, 1= negative
    // byte 5 = temperature positivity flag 0=positive, 1= negative
    // byte 6 = humidity integer portion
    // byte 7 = humidity fractional portion
    // byte 8 = temperature integer portion
    // byte 9= temperature fractional portion

    byte report_message[10] = {9, DHT_REPORT, DHT_DATA, 0, 0, 0, 0, 0, 0, 0};

    int rv;

    float humidity, temperature;

    // are there any dhts to read?
    if (dht_index) {
        // is it time to do the read? This should occur every 2 seconds
        dht_current_millis = millis();
        if (dht_current_millis - dht_previous_millis > dht_scan_interval) {
            // update for the next scan
            dht_previous_millis = dht_current_millis;

            // read and report all the dht sensors
            for (int i = 0; i < dht_index; i++) {
                //report_message[0] = 9; //message length
                report_message[1] = DHT_REPORT;
                // error type in report_message[2] will be set further down
                report_message[3] = dhts[i].pin;

                rv = dhts[i].dht_sensor->read();

                if (rv != DHTLIB_OK) {
                    rv = 0xff;
                }
                report_message[2] = (uint8_t) rv;

                // if rv is not zero, this is an error report
                if (rv) {
                    client.write(report_message, 10);
                    return;
                } else {
                    float j, f;
                    float humidity = dhts[i].dht_sensor->getHumidity();
                    if (humidity >= 0.0) {
                        report_message[4] = 0;
                    } else {
                        report_message[4] = 1;
                    }
                    f = modff(humidity, &j);
                    report_message[6] = (uint8_t) j;
                    report_message[7] = (uint8_t)(f * 100);


                    float temperature = dhts[i].dht_sensor->getTemperature();
                    if (temperature >= 0.0) {
                        report_message[5] = 0;
                    } else {
                        report_message[5] = 1;
                    }

                    f = modff(temperature, &j);

                    report_message[8] = (uint8_t) j;
                    report_message[9] = (uint8_t)(f * 100);
                    client.write(report_message, 10);

                }
            }
        }
    }
}

void scan_sonars() {
    float distance;
    float j, f;
    uint8_t integ, frac;

    if (sonars_index) {
        {
            sonar_current_millis = millis();
            if (sonar_current_millis - sonar_previous_millis > sonar_scan_interval) {
                sonar_previous_millis = sonar_current_millis;
                distance = sonars[last_sonar_visited].usonic->readSonar();
                if (distance != sonars[last_sonar_visited].last_value) {
                    sonars[last_sonar_visited].last_value = distance;

                    // byte 0 = packet length
                    // byte 1 = report type
                    // byte 2 = trigger pin number
                    // byte 3 = distance integer portion
                    // byte 4 = distance fractional portion

                    f = modff(distance, &j);

                    integ = (uint8_t) j;
                    frac = (uint8_t) f;
                    byte report_message[5] = {4, SONAR_DISTANCE, sonars[last_sonar_visited].trigger_pin,
                                              integ, frac
                    };
                    client.write(report_message, 5);
                }
                last_sonar_visited++;
                if (last_sonar_visited == sonars_index) {
                    last_sonar_visited = 0;
                }
            }
        }
    }
}

void scan_imu() {
    float ax, ay, az, gx, gy, gz = 0;
    float i, f;

    // report message will contain report length, report type, and 3 values
    // for results. Each result value consists of the integer portion, fractional
    // portion and a flag to indicated positivity.

    // Values reported as ax, ay, az, gx, gy, gz

    byte report_message[20] = {19, IMU_REPORT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0
    };


    if (imu_enabled) {
        // if there is an IMU error, set ax positivity to 99
        if (imu_error) {
            report_message[4] = 99;
            client.write(report_message, 20);
            return;
        }

        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            IMU.readAcceleration(ax, ay, az);
            IMU.readGyroscope(gx, gy, gz);


            f = modff(ax, &i);

            report_message[2] = (uint8_t) i;
            report_message[3] = (uint8_t)(f * 100);
            if (ax < 0) {
                report_message[4] = 1;
            }

            f = modff(ay, &i);

            report_message[5] = (uint8_t) i;
            report_message[6] = (uint8_t)(f * 100);
            if (ay < 0) {
                report_message[7] = 1;
            }

            f = modff(az, &i);

            report_message[8] = (uint8_t) i;
            report_message[9] = (uint8_t)(f * 100);
            if (az < 0) {
                report_message[10] = 1;
            }

            f = modff(gx, &i);

            report_message[11] = (uint8_t) i;
            report_message[12] = (uint8_t)(f * 100);
            if (gx < 0) {
                report_message[13] = 1;
            }

            f = modff(gy, &i);

            report_message[14] = (uint8_t) i;
            report_message[15] = (uint8_t)(f * 100);
            if (gy < 0) {
                report_message[16] = 1;
            }

            f = modff(gz, &i);

            report_message[17] = (uint8_t) i;
            report_message[18] = (uint8_t)(f * 100);
            if (gz < 0) {
                report_message[19] = 1;
            }
            client.write(report_message, 20);
        }
    }

}

void scan_microphone() {

    short data;
    byte report_message[4] = {3, MICROPHONE_REPORT, 0, 0,};
    if (microphone_enabled) {
        byte report_message[4] = {3, MICROPHONE_REPORT, 0, 0,};

        if (samplesRead) {
            for (int i = 0; i < samplesRead; i++) {
                data = sampleBuffer[i];
                report_message[2] = (byte)(data >> 8);
                report_message[3] = (byte)(data & 0xff);
                client.write(report_message, 4);
            }
        }
        samplesRead = 0;


    }
}

void setup() {

    Serial.begin(115200);

    WiFi.begin(ssid, password);

    delay(100);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    digitalWrite(LEDR, HIGH); //RED

    digitalWrite(LEDG, LOW); //GREEN

    digitalWrite(LEDB, LOW); //BLUE


    Serial.print("\n\nAllow 15 seconds for connection to complete..");

    while (WiFi.status() != WL_CONNECTED) {
        delay(10);
        //Serial.print(".");
    }
    wifiServer.begin();
    Serial.println();

    Serial.print("Connected to WiFi. IP Address: ");
    Serial.print(WiFi.localIP());

    Serial.print("  IP Port: ");
    Serial.println(PORT);
    digitalWrite(LEDR, LOW); // RED
    digitalWrite(LEDG, HIGH);


    // create an array of pin_descriptors for 100 pins
    // establish the digital pin array
    for (byte i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        the_digital_pins[i].pin_number = i;
        the_digital_pins[i].pin_mode = AT_MODE_NOT_SET;
        the_digital_pins[i].reporting_enabled = false;
        the_digital_pins[i].last_value = 0;
    }

    // establish the analog pin array
    for (byte i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
        the_analog_pins[i].pin_number = i;
        the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
        the_analog_pins[i].reporting_enabled = false;
        the_analog_pins[i].last_value = 0;
        the_analog_pins[i].differential = 0;
    }

    PDM.onReceive(onPDMdata);

    if (!PDM.begin(1, 16000)) {
        microphone_error = true;
    } else {
        microphone_error = false;
    }
}

void loop() {

    if (!rebooting) {

        client = wifiServer.available();

        if (client) {
            Serial.print("Client Connected to address: ");
            Serial.println(client.remoteIP());

            while (client.connected()) {
                delay(1);
                {
                    // keep processing incoming commands
                    get_next_command();

                    if (!stop_reports) { // stop reporting
                        scan_digital_inputs();
                        scan_analog_inputs();
                        scan_sonars();
                        scan_imu();
                        scan_microphone();
                        scan_dhts();
                    }
                }
            }
            client.stop();
            Serial.println("Client disconnected");
        }
    }
}
