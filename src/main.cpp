/***********************************************************************************************************************
 *
 * Written by Nabinho - 2023
 *
 *  Bluetooth Char Messages Reference - https://play.google.com/store/apps/details?id=braulio.calle.bluetoothRCcontroller&hl=pt_BR&gl=US
 *
 *  Hardware Reference:
 *    - RoboCore BlackBoard Edge - https://www.robocore.net/placa-robocore/blackboard-edge
 *
 **********************************************************************************************************************/

// Libraries
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <Arduino.h>
#include "BluetoothSerial.h"
#include "RoboCore_MMA8452Q.h"

// Bluetooth classic object
BluetoothSerial SerialBT;

// Accelerometer object
MMA8452Q accelerometer;

// Peripheral device parameters
uint8_t BT_address[4][6] = {{0x00, 0x06, 0x66, 0x4D, 0x61, 0x4F},  // BlueSMiRF Omni Robot V1.0
                            {0x98, 0xD3, 0x51, 0xFD, 0x90, 0xC8},  // Keyestudio HC-06
                            {0x00, 0x10, 0x06, 0x29, 0x00, 0x34},  // Bluetooth Bee 1
                            {0x00, 0x10, 0x06, 0x29, 0x00, 0x90}}; // Bluetooth Bee 2
bool connected;
uint8_t device = 0;

// Device selector DIP switch pins
const uint8_t DEVICE_SEL_PIN1 = 14;
const uint8_t DEVICE_SEL_PIN2 = 26;
bool reading_device_sel1;
bool reading_device_sel2;

// ESP32 SPI Pins
const uint8_t MY_MISO = 19;
const uint8_t MY_MOSI = 23;
const uint8_t MY_SCLK = 18;
const uint8_t MY_SS = 5;

// Radio Controller Object
RF24 radio(4, 5);
SPIClass *hspi = nullptr;

// Radios Addresses
uint8_t address[][6] = {"Ctrlr", "Robot"};

// Radio Number
const bool radio_number = 1;

// Variables structure
typedef struct
{
  uint8_t button1_reading;
  uint8_t button2_reading;
  uint8_t button3_reading;
  uint8_t button4_reading;
  uint8_t button5_reading;
  uint8_t button6_reading;
  uint16_t X1axis_reading;
  uint16_t Y1axis_reading;
  uint16_t X2axis_reading;
  uint16_t Y2axis_reading;
  uint16_t slider1_reading;
  uint16_t slider2_reading;
} controller_variables;
controller_variables controller;

// Variables for Message Receptions
uint8_t channel;
uint8_t bytes;

// Variables For Failsafe
unsigned long last_message = 0;
const uint16_t FAILSAFE_INTERVAL = 2000;

// Button Reading Variables
bool reading_button1;
bool button1_state;
bool last_button1_state = 0;
unsigned long last_debounce_time1;

// Button Reading Variables
bool reading_button2;
bool button2_state;
bool last_button2_state = 0;
unsigned long last_debounce_time2;

// Button Reading Variables
bool reading_button3;
bool button3_state;
bool last_button3_state = 0;
unsigned long last_debounce_time3;

// Button Reading Variables
bool reading_button4;
bool button4_state;
bool last_button4_state = 0;
unsigned long last_debounce_time4;

// Button Reading Variables
bool reading_button5;
bool button5_state;
bool last_button5_state = 0;
unsigned long last_debounce_time5;

// Button Reading Variables
bool reading_button6;
bool button6_state;
bool last_button6_state = 0;
unsigned long last_debounce_time6;

// Button Debounce
const uint8_t DEBOUNCE_TIME = 100;
uint8_t speed_max = 0;
uint8_t last_speed_max = 0;

// LED_BUILTIN pin
const uint8_t LED_PIN = 27;
const uint8_t SELECTOR_PIN = 25;

const uint8_t ACCEL_READ_TIME = 50;
unsigned long elapsed_time;

// Bluetooth message variable
char BT_message;

void setup()
{

  // Serial monitor initialization
  Serial.begin(9600);

  // LED_BUILTIN initialization
  pinMode(LED_PIN, OUTPUT);
  pinMode(SELECTOR_PIN, INPUT_PULLUP);
  pinMode(DEVICE_SEL_PIN1, INPUT_PULLUP);
  pinMode(DEVICE_SEL_PIN2, INPUT_PULLUP);

  // Reads device selector DIP switch
  reading_device_sel1 = digitalRead(DEVICE_SEL_PIN1);
  reading_device_sel2 = digitalRead(DEVICE_SEL_PIN2);
  Serial.println("\n\n");
  Serial.print("SEL1: ");
  Serial.print(reading_device_sel1);
  Serial.print(" | SEL2: ");
  Serial.println(reading_device_sel2);
  if (reading_device_sel1 == 1 && reading_device_sel2 == 1)
  {
    device = 0;
  }
  else if (reading_device_sel1 == 1 && reading_device_sel2 == 0)
  {
    device = 1;
  }
  else if (reading_device_sel1 == 0 && reading_device_sel2 == 1)
  {
    device = 2;
  }
  else if (reading_device_sel1 == 0 && reading_device_sel2 == 0)
  {
    device = 3;
  }
  
  Serial.print("Connecting to device: ");
  Serial.println(device);

  // Bluetooth classic initialization
  SerialBT.begin("Edge_RF24", true);
  SerialBT.setPin("1234");
  Serial.println("The device started in master mode, make sure remote BT device is on!");

  // Check if connected
  connected = SerialBT.connect(BT_address[device]);
  if (connected)
  {
    Serial.println("Connected Succesfully!");
  }
  else
  {
    while (!SerialBT.connected(10000))
    {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
    }
  }
  // Disconnect may take upto 10 secs max
  if (SerialBT.disconnect())
  {
    Serial.println("Disconnected Succesfully!");
  }
  // This would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();

  // SPI bus configuration
  hspi = new SPIClass(HSPI);
  hspi->begin(MY_SCLK, MY_MISO, MY_MOSI, MY_SS);

  // Radio Initialization
  if (!radio.begin(hspi))
  {
    Serial.println("Radio Initialization Failed!");
    while (!radio.begin(hspi))
    {
      Serial.print(F("."));
    }
  }

  // Configure Radio for Maximum Power
  radio.setPALevel(RF24_PA_MAX);

  // Configure Radio Payload Size
  radio.setPayloadSize(sizeof(controller));

  // Configure Radio Listening Pipe
  radio.openWritingPipe(address[radio_number]);

  // Configure Radio Channel Number
  radio.openReadingPipe(1, address[!radio_number]);

  // Configure Radio to Listen for Incoming Data
  radio.startListening();

  if (!accelerometer.init())
  {
    Serial.println("Accelerometer Initialization Failed!");
    while (!accelerometer.init())
    {
      Serial.print(F("."));
    }
  }

  // Wait for bluetooth connection
  while (!SerialBT.connected())
  {
    Serial.println("Connecting...");
  }
}

void loop()
{

  if (digitalRead(SELECTOR_PIN) == HIGH)
  {
    // Checks If New Reading Available
    if (radio.available(&channel) && SerialBT.connected())
    {
      // Reads Messages From Controller
      bytes = radio.getPayloadSize();
      radio.read(&controller, bytes);

      // Updates Lest Message Time
      last_message = millis();
      digitalWrite(LED_PIN, HIGH);

      //********************************************************************************************************************
      // Handle the Control when the Joysticks Heads Forward
      if (controller.Y2axis_reading > 550)
      {
        if (controller.X1axis_reading > 550)
        {
          BT_message = 'G';
        }
        else if (controller.X1axis_reading < 500)
        {
          BT_message = 'I';
        }
        else
        {
          BT_message = 'F';
        }
      }

      //********************************************************************************************************************
      // Handle the Control when the Joysticks Heads Backward
      else if (controller.Y2axis_reading < 500)
      {
        if (controller.X1axis_reading > 550)
        {
          BT_message = 'H';
        }
        else if (controller.X1axis_reading < 500)
        {
          BT_message = 'J';
        }
        else
        {
          BT_message = 'B';
        }
      }

      //********************************************************************************************************************
      // Handle the Control when the Joysticks Heads Left
      else if (controller.X1axis_reading > 550)
      {
        BT_message = 'L';
      }

      //********************************************************************************************************************
      // Handle the Control when the Joysticks Heads Left
      else if (controller.X1axis_reading < 500)
      {
        BT_message = 'R';
      }

      //********************************************************************************************************************
      else
      {
        BT_message = 'S';
      }

      //********************************************************************************************************************
      // Mode Control Changer
      reading_button1 = controller.button1_reading;
      if (reading_button1 != last_button1_state)
      {
        last_debounce_time1 = millis();
      }
      if ((millis() - last_debounce_time1) > DEBOUNCE_TIME)
      {
        if (reading_button1 != button1_state)
        {
          button1_state = reading_button1;
          if (button1_state == 1)
          {
            BT_message = 'X';
          }
          else
          {
            BT_message = 'x';
          }
        }
      }
      last_button1_state = reading_button1;

      //********************************************************************************************************************
      // Light Blink Enabling Changer
      reading_button2 = controller.button2_reading;
      if (reading_button2 != last_button2_state)
      {
        last_debounce_time2 = millis();
      }
      if ((millis() - last_debounce_time2) > DEBOUNCE_TIME)
      {
        if (reading_button2 != button2_state)
        {
          button2_state = reading_button2;
          if (button2_state == 1)
          {
            BT_message = 'V';
          }
          else
          {
            BT_message = 'v';
          }
        }
      }
      last_button2_state = reading_button2;

      //********************************************************************************************************************
      // Control the Front Light
      reading_button3 = controller.button3_reading;
      if (reading_button3 != last_button3_state)
      {
        last_debounce_time3 = millis();
      }
      if ((millis() - last_debounce_time3) > DEBOUNCE_TIME)
      {
        if (reading_button3 != button3_state)
        {
          button3_state = reading_button3;
          if (button3_state == 1)
          {
            BT_message = 'W';
          }
          else
          {
            BT_message = 'w';
          }
        }
      }
      last_button3_state = reading_button3;

      //********************************************************************************************************************
      // Control the Back Light
      reading_button4 = controller.button4_reading;
      if (reading_button4 != last_button4_state)
      {
        last_debounce_time4 = millis();
      }
      if ((millis() - last_debounce_time4) > DEBOUNCE_TIME)
      {
        if (reading_button4 != button4_state)
        {
          button4_state = reading_button4;
          if (button4_state == 1)
          {
            BT_message = 'U';
          }
          else
          {
            BT_message = 'u';
          }
        }
      }
      last_button4_state = reading_button4;

      //********************************************************************************************************************
      /*
      reading_button5 = controller.button5_reading;
      if (reading_button5 != last_button5_state)
      {
        last_debounce_time5 = millis();
      }
      if ((millis() - last_debounce_time5) > DEBOUNCE_TIME)
      {
        if (reading_button5 != button5_state)
        {
          button5_state = reading_button5;
          if (button5_state == 1)
          {
            digitalWrite(PIN_BUZZER, HIGH);
          }
          else
          {
            digitalWrite(PIN_BUZZER, LOW);
          }
        }
      }
      last_button5_state = reading_button5;
      */
      //********************************************************************************************************************
      /*
      reading_button6 = controller.button6_reading;
      if (reading_button6 != last_button6_state)
      {
        last_debounce_time6 = millis();
      }
      if ((millis() - last_debounce_time6) > DEBOUNCE_TIME)
      {
        if (reading_button6 != button6_state)
        {
          button6_state = reading_button6;
          if (button6_state == 1)
          {
          }
          else
          {
          }
        }
      }
      last_button6_state = reading_button6;
      */

      //********************************************************************************************************************
      // Speed Max Adjustment
      speed_max = map(((controller.slider1_reading + controller.slider2_reading) / 2), 1023, 0, 0, 10);
      if (speed_max != last_speed_max)
      {
        if (speed_max == 0)
        {
          BT_message = '0';
        }
        else if (speed_max == 1)
        {
          BT_message = '1';
        }
        else if (speed_max == 2)
        {
          BT_message = '2';
        }
        else if (speed_max == 3)
        {
          BT_message = '3';
        }
        else if (speed_max == 4)
        {
          BT_message = '4';
        }
        else if (speed_max == 5)
        {
          BT_message = '5';
        }
        else if (speed_max == 6)
        {
          BT_message = '6';
        }
        else if (speed_max == 7)
        {
          BT_message = '7';
        }
        else if (speed_max == 8)
        {
          BT_message = '8';
        }
        else if (speed_max == 9)
        {
          BT_message = '9';
        }
        else if (speed_max == 10)
        {
          BT_message = 'q';
        }
      }
      last_speed_max = speed_max;

      // Sends message
      SerialBT.write(BT_message);

      /*
      Serial.print("Message of ");
      Serial.print(bytes);
      Serial.print(" bytes received on channel ");
      Serial.print(channel);
      Serial.println(" content : ");
      Serial.print(controller.button1_reading);
      Serial.print(" | ");
      Serial.print(controller.button2_reading);
      Serial.print(" | ");
      Serial.print(controller.button3_reading);
      Serial.print(" | ");
      Serial.print(controller.button4_reading);
      Serial.print(" | ");
      Serial.print(controller.button5_reading);
      Serial.print(" | ");
      Serial.println(controller.button6_reading);
      Serial.print(controller.X1axis_reading);
      Serial.print(" | ");
      Serial.print(controller.Y1axis_reading);
      Serial.print(" | ");
      Serial.print(controller.X2axis_reading);
      Serial.print(" | ");
      Serial.print(controller.Y2axis_reading);
      Serial.print(" | ");
      Serial.print(controller.slider1_reading);
      Serial.print(" | ");
      Serial.println(controller.slider2_reading);
      Serial.print("Bluetooth message sent: ");
      Serial.println(BT_message);
      */
    }

    //********************************************************************************************************************
    // Handle Failsafe
    else if ((millis() - last_message) > FAILSAFE_INTERVAL)
    {
      digitalWrite(LED_PIN, LOW);
      SerialBT.write('S');
      Serial.println("RADIO FAILSAFE!!!");
    }

    //********************************************************************************************************************
    // Handle Bluetooth Disconnect
    else if (!SerialBT.connected())
    {
      digitalWrite(LED_PIN, LOW);
      Serial.println("BLUETOOTH DISCONNECTED!!!");
    }
  }
  else
  {
    if (!SerialBT.connected())
    {
      digitalWrite(LED_PIN, LOW);
      Serial.println("BLUETOOTH DISCONNECTED!!!");
    }
    else if ((millis() - elapsed_time) > ACCEL_READ_TIME)
    {
      // Reads accelerometer
      accelerometer.read();

      // Prints readings
      Serial.print(accelerometer.x);
      Serial.print('(');
      Serial.print(accelerometer.raw_x);
      Serial.print(")\t");
      Serial.print(accelerometer.y);
      Serial.print('(');
      Serial.print(accelerometer.raw_y);
      Serial.print(")\t");
      Serial.print(accelerometer.z);
      Serial.print('(');
      Serial.print(accelerometer.raw_z);
      Serial.println(')');

      // Control conditions
      if (accelerometer.y < -0.4 && accelerometer.x > 0.3)
      {
        BT_message = 'G';
      }
      else if (accelerometer.y < -0.4 && accelerometer.x < -0.3)
      {
        BT_message = 'I';
      }
      else if (accelerometer.y > 0.2 && accelerometer.x > 0.3)
      {
        BT_message = 'H';
      }
      else if (accelerometer.y > 0.2 && accelerometer.x < -0.3)
      {
        BT_message = 'J';
      }
      else if (accelerometer.y < -0.4)
      {
        BT_message = 'F';
      }
      else if (accelerometer.y > 0.2)
      {
        BT_message = 'B';
      }
      else if (accelerometer.x > 0.4)
      {
        BT_message = 'L';
      }
      else if (accelerometer.x < -0.4)
      {
        BT_message = 'R';
      }
      else
      {
        BT_message = 'S';
      }

      // Sends message
      SerialBT.write(BT_message);

      // Updates timer
      elapsed_time = millis();
      digitalWrite(LED_PIN, HIGH);
    }
  }
}
