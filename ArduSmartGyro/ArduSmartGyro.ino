/***************************************************************************************************************
* Nano AHRS Firmware v1.4.1

***************************************************************************************************************/

/*
  "9DOF Nano AHRS" hardware versions: 1.0

  ATMega328@5V, 16MHz

  ADXL345  : Accelerometer
  HMC5883L : Magnetometer
  ITG-3200 : Gyro

  Arduino IDE : Select board "Arduino Pro or Pro Mini (5v, 16Mhz) w/ATmega328"
*/

//*/

/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware
//#define HW__VERSION_CODE 10125
#define HW__VERSION_CODE 10736
//#define HW__VERSION_CODE 10183
//#define HW__VERSION_CODE 10321
//#define HW__VERSION_CODE 10724


// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 57600


#define COMPUTE__DATA_INTERVAL 10  // in milliseconds
#define OUTPUT__DATA_INTERVAL 100  // in milliseconds

#define OUTPUT__MODE_CALIBRATE_SENSORS 0
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float
#define OUTPUT__FORMAT_SHORT 2 // Outputs data as short text for float


int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_SHORT;


#define OUTPUT__STARTUP_STREAM_ON true


boolean output_errors = false;  // true or false

//for lock sensor
boolean output_lock_status_on;
boolean output_recorder_status_on;
float threshold_lock = 20.0f;  //default value
float threshold_unlock = -20.0f; //default value
float alphaForLock = -30.0f; //default is in N area

#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false


// SENSOR CALIBRATION
/*****************************************************************/

// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -261)
#define ACCEL_X_MAX ((float) 267)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 291)
#define ACCEL_Z_MIN ((float) -326)
#define ACCEL_Z_MAX ((float) 247)

// Magnetometer (standard calibration)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -274)
#define MAGN_X_MAX ((float) 476)
#define MAGN_Y_MIN ((float) -244)
#define MAGN_Y_MAX ((float) 665)
#define MAGN_Z_MIN ((float) -462)
#define MAGN_Z_MAX ((float) 489)


float gyro_avg_offset_x = 0;
float gyro_avg_offset_y = 0;
float gyro_avg_offset_z = 0;


// DEBUG OPTIONS

#define DEBUG__NO_DRIFT_CORRECTION false
#define DEBUG__PRINT_LOOP_TIME false


/*****************************************************************/
/****************** END  *********************/
/*****************************************************************/



// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
// Generate compile error
#error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Nano_AHRS.pde (or .ino)!
#endif

#include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
//sens 14.375, that is 1/GYRO_GAIN
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN))

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
//Pull down the 13(SCK) to gnd by wiring, so disable all of LED PIN output
#define STATUS_LED_PIN 13

#define GRAVITY 256.0f // "1G reference"
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

#define OUPUT_PIN_BY_MOSI 11
#define INPUT_PIN_BY_MISO 12

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3] = {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; // Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; // Omega Integrator
float Omega[3] = {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;


void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();

  // GET PITCH
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  // GET ROLL
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);

  roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;

  Serial.print("init attitude: Y=");
  Serial.print(TO_DEG(yaw));
  Serial.print("     P=");
  Serial.print(TO_DEG(pitch));
  Serial.print("     R=");
  Serial.println(TO_DEG(roll));

  // Init rotation matrix
  //    init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
  //lufei: do we really need to skip it?
  //lufei: use 0 to skip first DCM to make current attitude 0,0,0
  init_rotation_matrix(DCM_Matrix, 0, 0, 0);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
  // Compensate accelerometer error
  accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

  // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
  for (int i = 0; i < 3; i++)
    magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
  Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
  magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
  magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
  magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

  // Compensate gyroscope error
  Gyro_Vector[0] -= gyro_avg_offset_x;
  Gyro_Vector[1] -= gyro_avg_offset_y;
  Gyro_Vector[2] -= gyro_avg_offset_z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;

  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
}

void turn_output_stream_off()
{
  output_stream_on = false;
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}


void do_first_calibration()
{

  float timeMS_for_cali_offset = 3000.0;
  float time_interval = 0;
  float total_x = 0, total_y = 0, total_z = 0;
  float n = 0;
  bool led_ON_off = LOW;

  time_interval = 100;
  while (timeMS_for_cali_offset > 0)
  {
    delay(time_interval);

    read_sensors();
    compensate_sensor_errors();

    n++;
    total_x += Gyro_Vector[0];
    total_y += Gyro_Vector[1];
    total_z += Gyro_Vector[2];

    timeMS_for_cali_offset -= time_interval;
  }

  gyro_avg_offset_x = total_x / n;
  gyro_avg_offset_y = total_y / n;
  gyro_avg_offset_z = total_z / n;

  Serial.print("[CALI deg/s] "); Serial.print(gyro_avg_offset_x);
  Serial.print(",       "); Serial.print(gyro_avg_offset_y);
  Serial.print(",       "); Serial.println(gyro_avg_offset_z);
}


void setup()
{
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("AHRS is ready");

  //it is wired to GND, so here we set it to INPUT 
  //warning: output+high may cause drawing a lot of current
  pinMode (STATUS_LED_PIN, INPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();

  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  turn_output_stream_off();
  output_lock_status_on = true;
  output_recorder_status_on = false;

  //MOSI as Output to supply power of BLE chip, which should be <40mA
  pinMode (OUPUT_PIN_BY_MOSI, OUTPUT);
  digitalWrite(OUPUT_PIN_BY_MOSI, HIGH);

  //Double power supply
  pinMode (INPUT_PIN_BY_MISO, OUTPUT);
  digitalWrite(INPUT_PIN_BY_MISO, HIGH);

  //wait for 1 sec for being static
  delay(1000);
  do_first_calibration();

  init_lock_sensor(true);
}

void(* resetFunc) (void) = 0;//declare reset function at address 0

void do_command()
{
  if (Serial.available() >= 2)
  {
    if (Serial.read() == '#')
    {
      int command = Serial.read(); // Commands
      if (command == 'f')
      {
        output_single_on = true;
      }
      //to init threshold up and down
      //exmaple: "iu+170", "il-000", should be int value
      else if (command == 'i')
      {
        char output_param = readChar();
        float *thres = NULL;
        if (output_param == 'l')  //lock thres
          thres = &threshold_lock;
        else if (output_param == 'u')  //unlock thres
          thres = &threshold_unlock;
        else if (output_param == 'r')  //reset status
        {
          //待定
        }
        else
        {
          //          Serial.println("Error!");
          return;
        }

        if (thres != 0)
        {
          byte id[4]; //e.g. id = "+170"
          id[0] = readChar();
          id[1] = readChar();
          id[2] = readChar();
          id[3] = readChar();
          *thres = (id[1] - '0') * 100 + (id[2] - '0') * 10 + id[3] - '0';
          if (id[0] == '-')
            *thres = -*thres;

          Serial.print("!SET ");
          Serial.print(output_param == 'l' ? 'l' : 'u');
          Serial.print("=");
          Serial.println(*thres);
        }

        init_lock_sensor(output_param == 'l');

      }  //end if 'i'
      else if (command == 's')
      {
        // Read ID
        byte id[2];
        id[0] = readChar();
        id[1] = readChar();


        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      }
      else if (command == 'l')  // #l0 to close lock output, #l1 to open
      {
        char output_param = readChar();
        if (output_param == '0')
        {
          output_lock_status_on = false;
        }
        else if (output_param == '1')
          output_lock_status_on = true;
      }
      else if (command == 'r')  // #r0 to close recorder status output, #r1 to open
      {
        char output_param = readChar();
        if (output_param == '0')
        {
          output_recorder_status_on = false;
        }
        else if (output_param == '1')
          output_recorder_status_on = true;
      }
      else if (command == 'R')  // #R to reset arduino
      {
        resetFunc();  //call reset
      }
      else if (command == 'o')
      {
        char output_param = readChar();
        if (output_param == 'n')
        {
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 't')
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_TEXT;
        }
        else if (output_param == 'b')
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == 'f')
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_SHORT;
        }
        else if (output_param == 'c')
        {
          output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 's')
        {
          char values_param = readChar();
          char format_param = readChar();
          if (values_param == 'r')
            output_mode = OUTPUT__MODE_SENSORS_RAW;
          else if (values_param == 'c')
            output_mode = OUTPUT__MODE_SENSORS_CALIB;
          else if (values_param == 'b')
            output_mode = OUTPUT__MODE_SENSORS_BOTH;

          if (format_param == 't')
            output_format = OUTPUT__FORMAT_TEXT;
          else if (format_param == 'b')
            output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == '0')
        {
          turn_output_stream_off();
          reset_calibration_session_flag = true;
        }
        else if (output_param == '1')
        {
          reset_calibration_session_flag = true;
          turn_output_stream_on();
        }
        else if (output_param == 'e')
        {
          char error_param = readChar();
          if (error_param == '0') output_errors = false;
          else if (error_param == '1') output_errors = true;
          else if (error_param == 'c')
          {
            Serial.print("#AMG-ERR:");
            Serial.print(num_accel_errors); Serial.print(",");
            Serial.print(num_magn_errors); Serial.print(",");
            Serial.println(num_gyro_errors);
          }
        }
      } //end of 'o'
#if OUTPUT__HAS_RN_BLUETOOTH == true
      else if (command == 'C')
        turn_output_stream_on();
      else if (command == 'D')
        turn_output_stream_off();
#endif
    }
    else
    { }
  }
}


// Main loop
unsigned long time_d_output = 0;
void loop()
{
  //  int onOff = digitalRead(INPUT_PIN_BY_MISO);
  //  if (onOff == HIGH)
  //    turn_output_stream_on();
  //  else
  //    turn_output_stream_off();

  do_command();

  unsigned long time_d = 0;

  // Time to read
  time_d = millis() - timestamp;
  if (time_d >= COMPUTE__DATA_INTERVAL)
  {
    time_d_output += time_d;
    bool toOutput = time_d_output >= OUTPUT__DATA_INTERVAL;

    if (toOutput)
      time_d_output = 0;


    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run.
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      check_reset_calibration_session();
      if (output_stream_on || output_single_on)
        output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
    {

      compensate_sensor_errors();

      // Run DCM algorithm
      Compass_Heading();
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();

      //Output can be slower than sampling freq, as long as following the BLE sending freq
      if (toOutput)
      {
        if (output_stream_on || output_single_on)
          output_angles();

        check_lock_sensor();
        check_recoder_status();
      }
    }
    else  // Output sensor values
    {
      if (output_stream_on || output_single_on)
        output_sensors();
    }

    output_single_on = false;

#if DEBUG__PRINT_LOOP_TIME == true
    Serial.print("loop time (ms) = ");
    Serial.println(millis() - timestamp);
#endif
  }

#if DEBUG__PRINT_LOOP_TIME == true
  else
  {
    Serial.println("waiting...");
  }
#endif
}

