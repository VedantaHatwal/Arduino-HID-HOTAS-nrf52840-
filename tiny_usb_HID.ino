#include <Adafruit_TinyUSB.h>
#include <SimpleKalmanFilter.h>
#include <MPU9250_WE.h>
#include <Wire.h>



#define MPU9250_ADDR 0x68
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);



const int CALIBRATION_SAMPLES= 1000;


const int16_t MAX_JOYSTICK_VALUE = 32767;
const int16_t MIN_JOYSTICK_VALUE = -32768;


const int button1Pin = 38; // P1.06
const int button2Pin = 36; // P1.04
const int button3Pin = 11; // P0.11
const int button4Pin = 32; // P1.00









float ax_offset = 0;
float az_offset = 0;
float yaw_offset = 0;
bool calibrated = false;

// Kalman filters for accelerometer axes
SimpleKalmanFilter simpleKalmanFilterX(2, 2, 0.4);
SimpleKalmanFilter simpleKalmanFilterZ(2, 2, 0.4);

//for yaw
SimpleKalmanFilter simpleKalmanFilterYaw(2, 2, 0.004);



float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh, float offset) {
    // Map the value to the new range
    float mappedValue = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;

    // Apply the offset after scaling
    mappedValue += offset;

    // Clamp the result within the target range
    if (toLow < toHigh) {
        return fmax(toLow, fmin(mappedValue, toHigh));
    } else {
        return fmax(toHigh, fmin(mappedValue, toLow));
    }
}

uint8_t const sixteen_bit_desc[] =
{
		    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
		    0x09, 0x05,                    // USAGE (Joystick)
		    0xa1, 0x01,                    // COLLECTION (Application)
		    0xa1, 0x00,                    //   COLLECTION (Physical)
		    //0x85, 0x01,                  //     REPORT_ID (1)
                    HID_REPORT_ID(1)               //     DYNAMIC REPORT ID WITH TINYUSB
		    0x05, 0x09,                    //     USAGE_PAGE (Button)
		    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
		    0x29, 0x20,                    //     USAGE_MAXIMUM (Button 32)
		    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
		    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
		    0x95, 0x20,                    //     REPORT_COUNT (32)
		    0x75, 0x01,                    //     REPORT_SIZE (1)
		    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
		    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
		    0x09, 0x30,                    //     USAGE (X)
		    0x09, 0x31,                    //     USAGE (Y)
		    0x09, 0x32,                    //     USAGE (Z)
		    0x09, 0x33,                    //     USAGE (Rx)
		    0x09, 0x34,                    //     USAGE (Ry)
	            0x09, 0x35,                    //     USAGE (Rz)
		    0x09, 0x37,                    //     USAGE (Dial)
		    0x09, 0x36,                    //     USAGE (Slider)
		    0x16, 0x00,0x80,                    //     LOGICAL_MINIMUM (-2^15)
		    0x26, 0xff, 0x7f,               //     LOGICAL_MAXIMUM (2^15-1)
		    0x36, 0x00,0x80,                    //     PHYSICAL_MINIMUM (-2^15)
		    0x46, 0xff, 0x7f,               //     PHYSICAL_MAXIMUM (2^15-1)        
		    0x75, 0x10,                    //     REPORT_SIZE (16)
		    0x95, 0x08,                    //     REPORT_COUNT (8)
		    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
		    0xc0,                          //   END_COLLECTION
                    0xc0                           // END_COLLECTION
      
};


struct  __attribute__((__packed__)) reportHID_t {
		//uint8_t id = 1;
		uint32_t buttons = 0;
		int16_t X = 0;
		int16_t Y = 0;
		int16_t Z = 0;
		int16_t RX = 0;
		int16_t RY = 0;
		int16_t RZ = 0;
		int16_t Dial = 0;
		int16_t Slider = 0;
};

reportHID_t reportHID;

// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval (1= 1000hz, 2= 500hz, 3= 333hz), use out endpoint
Adafruit_USBD_HID HID(sixteen_bit_desc, sizeof(sixteen_bit_desc), HID_ITF_PROTOCOL_NONE, 1, false);











void calibrateMPU() {
    float ax_sum = 0.0f;
    float az_sum = 0.0f;
    float yaw_sum = 0.0f;
    
    //Serial.println("Calibrating MPU9250 - please keep device flat and still...");
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        xyzFloat gValue = myMPU9250.getGValues();
        float ax = atan(gValue.x / gValue.y);
        float az = atan(gValue.z / gValue.y);
        float yaw= analogRead(2);
        
        ax_sum += ax;
        az_sum += az;
        yaw_sum += yaw;

    }
    
    ax_offset = ax_sum / CALIBRATION_SAMPLES;
    az_offset = az_sum / CALIBRATION_SAMPLES;
    yaw_offset = yaw_sum / CALIBRATION_SAMPLES;
    calibrated = true;
    
    //Serial.print("Calibration complete. Offsets - ax: ");
    //Serial.print(ax_offset);
    //Serial.print(", az: ");
    //Serial.println(az_offset);
}













void setup() {

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);
  pinMode(button4Pin, INPUT_PULLUP);

    pinMode(29, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);

  //analogReadResolution(12); //should be used with 12 bit adc supported boards
  HID.begin();
  //Serial.begin(9600);
  // pinMode(2, INPUT_PULLUP);
  // pinMode(3, INPUT_PULLUP);

    Wire.setPins(31,29);  // SDA = 22, SCL = 24
  Wire.begin();
    Wire.setPins(31,29); 


    myMPU9250.init();
    //Serial.println("MPU9250 does not respond");
  
  
  
  

  //Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  //myMPU9250.autoOffsets();
  //Serial.println("Done!");


  myMPU9250.setSampleRateDivider(0);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_4);




  analogReadResolution(12);  // 12-bit ADC (0–4095)  for yaw
  pinMode(2, INPUT);
    calibrateMPU(); // Perform calibration on startup

}
  
  

void loop() {
    xyzFloat gValue = myMPU9250.getGValues();

    // Retrieve and filter accelerometer data
    float ax = atan(gValue.x / gValue.y); // angle in radians
    float az = atan(gValue.z / gValue.y); // angle in radians
    int yaw = analogRead(2);

            if (calibrated) {
        ax -= ax_offset;
        az -= az_offset;
        yaw -=yaw_offset;

    }




    ax = simpleKalmanFilterX.updateEstimate(ax);
    az = simpleKalmanFilterZ.updateEstimate(az);
    yaw = simpleKalmanFilterYaw.updateEstimate(yaw);





    
    //Serial.println(yaw);













    // Map angle to joystick range (-32767 to 32767)
    float joystickX = mapFloat(1.5 * ax, -0.7, 0.7, MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE, 0);
    float joystickY = mapFloat(1.5 * az, 0.7, -0.7, MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE, 0);
    float joystickYaw = mapFloat(yaw, -220, 220, MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE, 0);

if (joystickYaw < 2000 && joystickYaw > -2000)
 {
  joystickYaw = 0;
}






    // Read buttons
    uint8_t buttons = 0;
    if (digitalRead(button1Pin) == LOW) buttons |= (1 << 0);
    if (digitalRead(button2Pin) == LOW) buttons |= (1 << 1);
    if (digitalRead(button3Pin) == LOW) buttons |= (1 << 2);
    if (digitalRead(button4Pin) == LOW) buttons |= (1 << 3);

    // Prepare and send HID report
    reportHID.Dial = 0;
    reportHID.buttons = buttons;
    reportHID.RX = 0;
    reportHID.RY = 0;
    reportHID.RZ = joystickYaw;
    reportHID.X = joystickX;
    reportHID.Y = joystickY;
    reportHID.Z = 0;

    HID.sendReport(1, &reportHID, sizeof(reportHID_t));
}