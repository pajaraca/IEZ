#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <jled.h>
#include <ESP32TimerInterrupt.h>
#include <AceButton.h>
using namespace ace_button;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define DEVICE_ID "JJR01A-001"
#define LED_PIN         22
#define BTN_PIN         0
#define I2C_SDA         17
#define I2C_SCL         21
#define MPU_01_INT_PIN  16
#define MPU_02_INT_PIN  23
#define TIMER_INTERVAL_MS 10



bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t MPU01InterruptStatus;   // holds actual interrupt status byte from MPU
uint8_t MPU02InterruptStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 a;         // [x, y, z]            accel sensor measurements
VectorInt16 g;         // [x, y, z]            accel sensor measurements

MPU6050 mpu01(0x68);
MPU6050 mpu02(0x69);
BluetoothSerial SerialBT;
auto led = JLed(LED_PIN).Off();
AceButton button((uint8_t) BTN_PIN);
ESP32Timer sampling_timer(0);


bool flag = false;
bool btn_pressed = false;

const uint8_t data_packet_size = 28;
uint8_t data_packet[data_packet_size] = {'$',          // start packet char 
                                          0,           // mpu_id
                                          0, 0, 0, 0,  // ts 
                                          0, 0,  // quaternion w
                                          0, 0,  // quaternion x
                                          0, 0,  // quaternion y
                                          0, 0,  // quaternion z
                                          0, 0,        // accel x
                                          0, 0,        // accel x
                                          0, 0,        // accel x
                                          0, 0,        // gyro x
                                          0, 0,        // gyro y
                                          0, 0,        // gyro z
                                          '\r', '\n'};  // end packet char

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool MPUInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void IRAM_ATTR on_timer()
{
  MPUInterrupt = true;
}

// The event handler for the button.
void btn_handler(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {
  switch (eventType) {
    case AceButton::kEventReleased:
      btn_pressed = true;
      break;
  }
}

void mpu_initialization(const char *name, MPU6050 *mpu) {
  uint8_t accel_fs = 0;
  uint8_t gyro_fs = 0;

  SerialBT.print(F("\r\nConfiguring "));
  SerialBT.print(name);
  SerialBT.println(F("..."));

  // get parameters
  SerialBT.print(F("Select acelerometer full-scale (0 - 2G; 1 - 4G; 2 - 8G; 3 - 16G): "));
  while (SerialBT.available() && SerialBT.read());  // empty buffer
  while (!SerialBT.available());                    // wait for data
  accel_fs = SerialBT.read() - 0x30; // read and subtracts 0x30 to convert from ASCII to int

  SerialBT.print(F("\r\nSelect gyroscope full-scale (0 - 250; 1 - 500; 2 - 1000; 3 - 2000): "));
  while (SerialBT.available() && SerialBT.read());  // empty buffer
  while (!SerialBT.available());                    // wait for data
  gyro_fs = SerialBT.read() - 0x30; // read and subtracts 0x30 to convert from ASCII to int

  // verifies inputed values, if it is invalid set default value to 0 (ACCEL - 2G and GYRO - 250)
  if(!(accel_fs >= 0 && accel_fs < 4)) {
    accel_fs = 0;
  }
  if(!(gyro_fs >= 0 && gyro_fs < 4)) {
    gyro_fs = 0;
  }

  SerialBT.printf("\r\n%s configured with values: accel=%d,gyro=%d", name, accel_fs, gyro_fs);

  mpu->initialize();

  // verify connection
  SerialBT.print(F("\r\nTesting connection of "));
  SerialBT.print(name);
  SerialBT.print(F("..."));
  SerialBT.println(mpu->testConnection() ? F(" connection successful!") : F(" connection failed!"));

  mpu->setFullScaleAccelRange(accel_fs);
  mpu->setFullScaleGyroRange(gyro_fs);
  
  // supply your own gyro offsets here, scaled for min sensitivity
  // mpu->setXGyroOffset(220);
  // mpu->setYGyroOffset(76);
  // mpu->setZGyroOffset(-85);
  // mpu->setZAccelOffset(1788); // 1688 factory default for my test chip

  // calibration time: generate offsets and calibrate our MPU6050
  // mpu->CalibrateAccel(6);
  // mpu->CalibrateGyro(6);
}

void read_mpu_data(MPU6050 *mpu, uint8_t mpu_id) {

  // our experiment package will have the following format
  // mpu_id, ts, q_w, q_x, q_y, q_z, a_x, a_y, a_z, g_x, g_y, g_z
  // q -> stands for quaternion
  // a -> stands for raw acceleration with gravity
  // g -> stands for raw gyro

  // compute orientation data
  // mpu->dmpGetQuaternion(&q, fifoBuffer);
  // accel e gyro from DMP
  // mpu->dmpGetAccel(&a, fifoBuffer);
  // mpu->dmpGetGyro(&g, fifoBuffer);
  // accel e gyro directly from sensors

  uint32_t ts = millis();

  mpu->getMotion6(&(a.x), &(a.y), &(a.z), &(g.x), &(g.y), &(g.z));


  data_packet[1] = mpu_id;     // mpu_id
  data_packet[2] = (uint8_t) ((ts >> 24) & 0x000000FF); // timestamp
  data_packet[3] = (uint8_t) ((ts >> 16) & 0x000000FF); // timestamp
  data_packet[4] = (uint8_t) ((ts >> 8)  & 0x000000FF); // timestamp
  data_packet[5] = (uint8_t) (ts & 0x000000FF); // timestamp

  // data_packet[6] = fifoBuffer[0];     // quaternion w
  // data_packet[7] = fifoBuffer[1];     // quaternion w
  // data_packet[8] = fifoBuffer[4];     // quaternion x
  // data_packet[9] = fifoBuffer[5];     // quaternion x
  // data_packet[10] = fifoBuffer[8];    // quaternion y
  // data_packet[11] = fifoBuffer[9];    // quaternion y
  // data_packet[12] = fifoBuffer[12];   // quaternion z
  // data_packet[13] = fifoBuffer[13];   // quaternion z

  data_packet[14] = (uint8_t) ((a.x >> 8) & 0x00FF);  // accel x
  data_packet[15] = (uint8_t) (a.x & 0x00FF);         // accel x
  data_packet[16] = (uint8_t) ((a.y >> 8) & 0x00FF);  // accel y
  data_packet[17] = (uint8_t) (a.y & 0x00FF);         // accel y
  data_packet[18] = (uint8_t) ((a.z >> 8) & 0x00FF);  // accel z
  data_packet[19] = (uint8_t) (a.z & 0x00FF);         // accel z

  data_packet[20] = (uint8_t) ((g.x >> 8) & 0x00FF);    // gyro x
  data_packet[21] = (uint8_t) (g.x & 0x00FF);           // gyro x
  data_packet[22] = (uint8_t) ((g.y >> 8) & 0x00FF);    // gyro y
  data_packet[23] = (uint8_t) (g.y & 0x00FF);           // gyro y
  data_packet[24] = (uint8_t) ((g.z >> 8) & 0x00FF);    // gyro z
  data_packet[25] = (uint8_t) (g.z & 0x00FF);           // gyro z

  SerialBT.write(data_packet, data_packet_size);
    
  // SerialBT.printf("%hhu,%u,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d\n", mpu_id, ts, q.w, q.x, q.y, q.z, a.x, a.y, a.z, g.x, g.y, g.z);
}

void start_capture_cycle() {
  
  led.On().Update();

  while(!btn_pressed) button.check();
  btn_pressed = false;

  // initialize device
  SerialBT.println(F("Initializing MPU devices..."));
  mpu_initialization(F("MPU-01"), &mpu01);
  mpu_initialization(F("MPU-02"), &mpu02);

  // set hardware timer interrupt
  // interval in microsecs, so MS to multiply by 1000
  // be sure to place this HW Timer well ahead blocking calls, because it needs to be initialized.
  if (sampling_timer.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, on_timer))
    SerialBT.println("\r\nConfiguring sampling timer OK, millis() = " + String(millis()));
  else
    SerialBT.println("\r\nCan't set sampling timer. Select another freq. or interval");
  
  sampling_timer.stopTimer();
  SerialBT.println(F("\r\nSend a character to start..."));
  while (SerialBT.available() && SerialBT.read());  // empty buffer
  while (!SerialBT.available());                    // wait for data
  while (SerialBT.available() && SerialBT.read());  // empty buffer again

  SerialBT.println(F("Wait until LED starts blinking to walk..."));
  delay(3000);
  sampling_timer.restartTimer();
  led.Blink(100, 100).Forever();
}


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties    

  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  while (!Serial);

  // configure button - it uses the built-in pull up register.
  pinMode(BTN_PIN, INPUT_PULLUP);

  // configure the ButtonConfig with the event handler, and enable events.
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(btn_handler);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);

  SerialBT.begin(DEVICE_ID); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  start_capture_cycle();
}

void loop() {

  if(!SerialBT.isReady()) return;

  if(btn_pressed) {
    sampling_timer.stopTimer();
    btn_pressed = false;
    start_capture_cycle();
  }
  
  if(MPUInterrupt) {
    MPUInterrupt = false;
    read_mpu_data(&mpu01, 1);
    read_mpu_data(&mpu02, 2);
  }

  led.Update();
  button.check();
}
