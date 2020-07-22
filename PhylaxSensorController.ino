/*
 * Phylax Sensor Controller Firmware
 * ROS publisher for IMU, Quadrature encoder data
 * 
 */
#include <WiFi.h>
#include <Wire.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <Icm20948.h>
#include <SensorTypes.h>
#include <Icm20948MPUFifoControl.h>
#include <Icm20948DataBaseControl.h>

#include <ESP32Encoder.h>

#define SERIAL_PORT Serial
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 

#define ICM_I2C_ADDR_REVB     0x69  /* I2C slave address for INV device on Rev B board */
#define AK0991x_DEFAULT_I2C_ADDR  0x0C  /* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E  /* The secondary I2C address for AK0991x Magnetometers */

#define MPU0_AD0_PIN 25 //14

uint8_t I2C_Address = 0x69;

const char* ssid     = "Get-c77fb4";
const char* password = "283742616";
IPAddress server(192,168,0,15);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
std_msgs::Int32 l_enc_msg;
std_msgs::Int32 r_enc_msg;
ros::Publisher imu_pub("imu/data",&imu_msg);
ros::Publisher left_wheel ("lwheel", &l_enc_msg);
ros::Publisher right_wheel ("lwheel", &r_enc_msg);

char eamessage[1024];

ESP32Encoder Left_encoder;
ESP32Encoder Right_encoder;

unsigned long encoder2lastToggled;
bool encoder2Paused = false;

static const uint8_t dmp3_image[] = 
{
#include "icm20948_img.dmp3a.h"
};

inv_icm20948_t icm_device;

int rc = 0;
#define THREE_AXES 3
static int unscaled_bias[THREE_AXES * 2];

/* FSR configurations */
int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

static const float cfg_mounting_matrix[9] = {
  1.f, 0, 0,
  0, 1.f, 0,
  0, 0, 1.f
};

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
  INV_SENSOR_TYPE_ACCELEROMETER,
  INV_SENSOR_TYPE_GYROSCOPE,
  INV_SENSOR_TYPE_RAW_ACCELEROMETER,
  INV_SENSOR_TYPE_RAW_GYROSCOPE,
  INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
  INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
  INV_SENSOR_TYPE_BAC,
  INV_SENSOR_TYPE_STEP_DETECTOR,
  INV_SENSOR_TYPE_STEP_COUNTER,
  INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
  INV_SENSOR_TYPE_ROTATION_VECTOR,
  INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
  INV_SENSOR_TYPE_MAGNETOMETER,
  INV_SENSOR_TYPE_SMD,
  INV_SENSOR_TYPE_PICK_UP_GESTURE,
  INV_SENSOR_TYPE_TILT_DETECTOR,
  INV_SENSOR_TYPE_GRAVITY,
  INV_SENSOR_TYPE_LINEAR_ACCELERATION,
  INV_SENSOR_TYPE_ORIENTATION,
  INV_SENSOR_TYPE_B2S
};

void inv_icm20948_sleep(int ms)
{
  delay(ms);
}

void inv_icm20948_sleep_us(int us)
{
  delayMicroseconds(us);
}

int i2c_master_write_register(uint8_t address, uint8_t reg, uint32_t len, const uint8_t *data)
{
  if (address != 0x69)
  {
    Serial.print("Odd address:");
    Serial.println(address);
  }

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data, len);
  Wire.endTransmission();
  
  return 0;
}

int i2c_master_read_register(uint8_t address, uint8_t reg, uint32_t len, uint8_t *buff)
{
  if (address != 0x69)
  {
    Serial.print("Odd read address:");
    Serial.println(address);
  }

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false); // Send repeated start

  uint32_t offset = 0;
  uint32_t num_received = Wire.requestFrom(address, len);

  if (num_received == len)
  {
    for (uint8_t i = 0; i < len; i++)
    {
      buff[i] = Wire.read();
    }
    return 0;
  }
  else
  {
    return -1;
  }
}

int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
  return i2c_master_read_register(I2C_Address, reg, rlen, rbuffer);
}

//---------------------------------------------------------------------

int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
  return i2c_master_write_register(I2C_Address, reg, wlen, wbuffer);
}

static void icm20948_set_fsr(void)
{
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

//---------------------------------------------------------------------
static void icm20948_apply_mounting_matrix(void)
{
  int ii;
  for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++)
  {
    inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, (inv_icm20948_sensor)ii);
  }
}

uint64_t inv_icm20948_get_time_us(void)
{
  return millis(); //InvEMDFrontEnd_getTimestampUs();
}


int icm20948_sensor_setup(void)
{
  int rc;
  uint8_t i, whoami = 0xff;

  /*
  * Just get the whoami
  */
  rc = inv_icm20948_get_whoami(&icm_device, &whoami);
  Serial.print("whoami = ");
  Serial.println(whoami);

  //delay(1000);

  /* Setup accel and gyro mounting matrix and associated angle for current board */
  inv_icm20948_init_matrix(&icm_device);

  Serial.print("dmp image size = ");
  Serial.println(sizeof(dmp3_image));
  
  rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
  if (rc != 0)
  {
    rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
    Serial.print("init got ");
    Serial.println(rc);
    return rc;
  }

  /* Initialize auxiliary sensors */
  inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);   //AK0991x_SECONDARY_I2C_ADDR); // AK0991x_DEFAULT_I2C_ADDR);

  rc = inv_icm20948_initialize_auxiliary(&icm_device);

  if (rc != 0)
  {
    Serial.print("compass not detected got ");
    Serial.println(rc);
  }
  else
  {
    Serial.println("compass detected");
  }
  icm20948_apply_mounting_matrix();

  icm20948_set_fsr();

  /* re-initialize base state structure */
  inv_icm20948_init_structure(&icm_device);
  return 0;
} //sensor_setup

void selectMPU(int m)
{

  digitalWrite(MPU0_AD0_PIN, HIGH);
  delay(25);
  digitalWrite(MPU0_AD0_PIN, LOW);
  delay(25);
  Serial.print("SELECTED AD0 on DEVICE ");
  Serial.print(m);
  Serial.print(" Pin ");
  Serial.println(MPU0_AD0_PIN);
}

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor)
{
  switch (sensor)
  {
     case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
       return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
     case INV_SENSOR_TYPE_RAW_GYROSCOPE:
       return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
     case INV_SENSOR_TYPE_ACCELEROMETER:
       return INV_ICM20948_SENSOR_ACCELEROMETER;
     case INV_SENSOR_TYPE_GYROSCOPE:
       return INV_ICM20948_SENSOR_GYROSCOPE;
     case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
       return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
     case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
       return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
     case INV_SENSOR_TYPE_BAC:
       return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
     case INV_SENSOR_TYPE_STEP_DETECTOR:
       return INV_ICM20948_SENSOR_STEP_DETECTOR;
     case INV_SENSOR_TYPE_STEP_COUNTER:
       return INV_ICM20948_SENSOR_STEP_COUNTER;
     case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
       return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
     case INV_SENSOR_TYPE_ROTATION_VECTOR:
       return INV_ICM20948_SENSOR_ROTATION_VECTOR;
     case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
       return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
     case INV_SENSOR_TYPE_MAGNETOMETER:
       return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
     case INV_SENSOR_TYPE_SMD:
       return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
     case INV_SENSOR_TYPE_PICK_UP_GESTURE:
       return INV_ICM20948_SENSOR_FLIP_PICKUP;
     case INV_SENSOR_TYPE_TILT_DETECTOR:
       return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
     case INV_SENSOR_TYPE_GRAVITY:
       return INV_ICM20948_SENSOR_GRAVITY;
     case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
       return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
     case INV_SENSOR_TYPE_ORIENTATION:
       return INV_ICM20948_SENSOR_ORIENTATION;
     case INV_SENSOR_TYPE_B2S:
       return INV_ICM20948_SENSOR_B2S;
     default:
       return INV_ICM20948_SENSOR_MAX;
  }//switch
}//enum sensortyp_conversion

void setup()
{
  pinMode(MPU0_AD0_PIN, OUTPUT);
  digitalWrite(MPU0_AD0_PIN, HIGH);
    
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  while(!SERIAL_PORT){};
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  Wire.begin();
  Wire.setClock(400000);

  selectMPU(0);
    
  struct inv_icm20948_serif icm20948_serif;
  icm20948_serif.context   = 0; /* no need */
  icm20948_serif.read_reg  = idd_io_hal_read_reg;
  icm20948_serif.write_reg = idd_io_hal_write_reg;
  icm20948_serif.max_read  = 1024 * 16; /* maximum number of bytes allowed per serial read */
  icm20948_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */

  icm20948_serif.is_spi = false;

  icm_device.base_state.serial_interface = SERIAL_INTERFACE_I2C;

  inv_icm20948_reset_states(&icm_device, &icm20948_serif);
  inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

  rc = icm20948_sensor_setup();

  if (icm_device.selftest_done && !icm_device.offset_done)
  {
    // If we've run selftes and not already set the offset.
    inv_icm20948_set_offset(&icm_device, unscaled_bias);
    icm_device.offset_done = 1;
  }

  //Set IMU output data rate (ODR) to 50Hz
  inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE),50);
  inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER),50);
  inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR),50);
  inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_RAW_MAGNETOMETER),50);
    
  //enable sensors
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), 1);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), 1);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), 1);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_RAW_MAGNETOMETER), 1);
    
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;

  // Attach pins for use as encoder pins
  Left_encoder.attachHalfQuad(19, 18);
  Right_encoder.attachHalfQuad(17, 16);

  //Clear encoder count
  Left_encoder.clearCount();
  Right_encoder.clearCount();

  //Advertise publishers
  nh.advertise(imu_pub);
  nh.advertise(left_wheel);
  nh.advertise(right_wheel);
}

static uint8_t icm20948_get_grv_accuracy(void)
{
  uint8_t accel_accuracy;
  uint8_t gyro_accuracy;

  accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
  gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
  return (min(accel_accuracy, gyro_accuracy));
}
void build_sensor_event_data(void *context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void *data, const void *arg)
{
  float raw_bias_data[6];
  inv_sensor_event_t event;
  (void)context;
  uint8_t sensor_id = convert_to_generic_ids[sensortype];

  memset((void *)&event, 0, sizeof(event));
  event.sensor = sensor_id;
  event.timestamp = timestamp;
  switch (sensor_id)
  {
  case INV_SENSOR_TYPE_GYROSCOPE:
    memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
    memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
    // sprintf(eamessage, "Gyro: [%f,%f,%f]", event.data.gyr.vect[0], event.data.gyr.vect[1], event.data.gyr.vect[2]);
    imu_msg.angular_velocity.x = event.data.gyr.vect[0];
    imu_msg.angular_velocity.y = event.data.gyr.vect[1];
    imu_msg.angular_velocity.z = event.data.gyr.vect[2];
    imu_msg.header.stamp = nh.now();
    //Serial.println(eamessage);
    break;
  case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
  case INV_SENSOR_TYPE_ACCELEROMETER:
    memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
    memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
    //sprintf(eamessage, "Accel: [%f,%f,%f]", event.data.acc.vect[0], event.data.acc.vect[0], event.data.acc.vect[0]);
    imu_msg.linear_acceleration.x = event.data.acc.vect[0];
    imu_msg.linear_acceleration.y = event.data.acc.vect[1];
    imu_msg.linear_acceleration.z = event.data.acc.vect[2];    
    //Serial.println(eamessage);
    break;
  case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
    memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
    event.data.quaternion.accuracy_flag = icm20948_get_grv_accuracy();
    //sprintf(eamessage, "Quaternion: (%f,%f,%f,%f)", event.data.quaternion.quat[0], event.data.quaternion.quat[1], event.data.quaternion.quat[2], event.data.quaternion.quat[3]);
    //Serial.println(eamessage);
    imu_msg.orientation.x = event.data.quaternion.quat[0];
    imu_msg.orientation.y = event.data.quaternion.quat[1];
    imu_msg.orientation.z = event.data.quaternion.quat[2];
    imu_msg.orientation.w = event.data.quaternion.quat[3];
    imu_msg.header.seq++;
    imu_msg.header.frame_id = "imu_link";
//    imu_msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6];
   // imu_msg.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6];
  //  imu_msg.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};
    imu_pub.publish(&imu_msg);   
    break;
    return;
  }
}

void loop()
{
  if (nh.connected()) {
    //fetch & publish imu data
    int rv = inv_icm20948_poll_sensor(&icm_device, (void *)0, build_sensor_event_data);

    //fetch & publish encoder data
    l_enc_msg.data = Left_encoder.getCount();
    r_enc_msg.data = Right_encoder.getCount();
    //Serial.println("Encoder count = "+String((int32_t)Left_encoder.getCount())+" "+String((int32_t)Right_encoder.getCount()));
    
    left_wheel.publish(&l_enc_msg);
    right_wheel.publish(&r_enc_msg);
  } else {     
    Serial.println("Not Connected");
  }
    nh.spinOnce();
    delay(1);
}
