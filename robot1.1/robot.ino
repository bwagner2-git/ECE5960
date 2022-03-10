
#include "BLECStringCharacteristic.h"
#include "EString.h" //will this redefine it?
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "bot.h"

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "9A48ECBA-2E92-082F-C079-9E75AAE428B1"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

/////IMU///////
#define AD0_VAL 0  
////////////////


//////////// Global Variables ////////////

BLEService testService(BLE_UUID_TEST_SERVICE);


BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

//BLEFloatCharacteristic roll(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
//BLEFloatCharacteristic pitch(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
//BLEFloatCharacteristic yaw(BLE_UUID_TX_FLOAT, BLERead | BLENotify);







// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
};

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */
             float float_c, float_d, float_e;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_d);
            if (!success)
                return;
            success=robot_cmd.get_next_value(float_e);
            if (!success)
                return;

            Serial.print("Three Floats: ");
            Serial.print(float_c);
            Serial.print(", ");
            Serial.print(float_d);
            Serial.print(", ");
            Serial.println(float_e);

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO: 
           {

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            //Serial.println(char_arr);
            tx_estring_value.clear();
            char *last=&char_arr[0];
            while (*last!=NULL){
              last+=1;
            }
            *last=';';
            *(last+1)=')';
            
//            for(int i=0;i<10;i++) Serial.println(char_arr[i]);
            tx_estring_value.append(char_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());


            /*
             * Your code goes here.
             */
            
            break;
          }
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}


#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensorOne;
SFEVL53L1X distanceSensorTwo;

ICM_20948_I2C myICM;


void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void sendLog(){   ///////FIX THIs
//  struct info *logger=ro.logPoint;
//  EString newLog;
  Serial.println("sending over log");
  for (int i=0;i<logLength;i++){
    tx_characteristic_string.writeValue(infoString(logs[i]).c_str());
  } 
}

void printLog(){ //////this seems to be working!!!
//  EString newLog;
  for (int i=0;i<logLength;i++){
//    newLog.clear();
//    newLog=infoString(logs[i]);
    Serial.println(infoString(logs[i]).c_str());
    Serial.println(i);
  } 
}


void
setup()
{
    Wire.begin();
//    Wire.setClock(400000); //potentially add this back in
    pinMode(2,OUTPUT);
    digitalWrite(2,LOW);
    distanceSensorTwo.setI2CAddress(0xAA);
    digitalWrite(2,HIGH);

 
    
    Serial.begin(115200);

    if (distanceSensorOne.begin() != 0) {
        Serial.println("Sensor one failed to begin. Please check wiring. Freezing...");
      } else {
        Serial.println("Sensor One online!");  
      }
       if (distanceSensorTwo.begin() != 0) {
        Serial.println("Sensor two failed to begin. Please check wiring. Freezing...");
      } else{
        Serial.println("Sensor two online!");
  }

    


    ///IMU
    bool initialized = false;
    while (!initialized)
    {
      myICM.begin(Wire, AD0_VAL);
  
  
      Serial.print(F("Initialization of the sensor returned: "));
      Serial.println(myICM.statusString());
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        Serial.println("Trying again...");
        delay(500);
      }
      else
      {
        initialized = true;
      }
    }
    /////////
    Serial.print("intializing Bluetooth");
    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

//    testService.addCharacteristic(roll);
//    testService.addCharacteristic(pitch);
//    testService.addCharacteristic(yaw);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);
//    yaw.writeValue(0.0);
//    roll.writeValue(0.0);
//    pitch.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
    Serial.println("leaving setup");
}

bot myBot(&distanceSensorOne,&distanceSensorTwo, &myICM);
int updateSuccess=0;
int logSuccess=1;
unsigned long timeStamp;
unsigned long startTime;
void
loop()
{
    // Listen for connections
//    Serial.println("entering loop");
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        startTime=millis();
        // While central is connected
        while (central.connected()) {
          
          updateSuccess=myBot.updatePosition();
          timeStamp=millis();
          logSuccess=myBot.logIt(timeStamp-startTime);
          if (logSuccess==0){
            break;
          }
        }
//        Serial.println(logs[0]);
        sendLog();
        printLog();
//        Serial.print("15th entry");
//        Serial.println(infoString((myBot.logPoint+15)).c_str()); ///dont use log point!!! the log it method already uses it
        while (1){
          
        }

        Serial.println("Disconnected");
    }
}
