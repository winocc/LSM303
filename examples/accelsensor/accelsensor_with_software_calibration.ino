#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <math.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// To properly calibrate, the WiNo mustn't move on startup...
#define EARTH_GRAVITY_STANDARD 9.80665
double xNotCalibrated;
double yNotCalibrated;
double zNotCalibrated;
double gNotCalibrated;

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void)
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
 

  /* Display some basic information on this sensor */
  displaySensorDetails();

   // Calibration
  // // To properly calibrate, the WiNo mustn't move...
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  double xNotCalibrated = event.acceleration.x;
  double yNotCalibrated = event.acceleration.y;
  double zNotCalibrated = event.acceleration.z;
  gNotCalibrated = sqrt(pow(xNotCalibrated,2)+pow(yNotCalibrated,2)+pow(zNotCalibrated,2));
  Serial.print("Measured g without calibration :"); Serial.print(gNotCalibrated); Serial.print("  ");Serial.println("m/s^2 ");

}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  
  Serial.println("=====================");

  /* Display the results without calibration (acceleration is measured in m/s^2) */
  Serial.print("X not calibrated "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y not calibrated: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z not calibrated: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  // Compute measured earth gravity to see not calibrated datas :
  double accelerationVectorNormNotCalibrated = sqrt(pow(event.acceleration.x,2)+pow(event.acceleration.y,2)+pow(event.acceleration.z,2));
  Serial.print("Measured acceleration vector norm without calibration :"); Serial.print(accelerationVectorNormNotCalibrated); Serial.print("  ");Serial.println("m/s^2 ");
  
  Serial.println("=====================");
    // Calibration
  double xAccel = (event.acceleration.x)*EARTH_GRAVITY_STANDARD/gNotCalibrated;
  double yAccel = (event.acceleration.y)*EARTH_GRAVITY_STANDARD/gNotCalibrated;
  double zAccel = (event.acceleration.z)*EARTH_GRAVITY_STANDARD/gNotCalibrated;
  // display results
  Serial.print("X :"); Serial.print(xAccel); Serial.print("  ");
  Serial.print("Y :"); Serial.print(yAccel); Serial.print("  ");
  Serial.print("Z :"); Serial.print(zAccel); Serial.print("  ");Serial.println("m/s^2 ");

  // Compute norm of acceleration vector
  // Should be 9.81 when wino ins't moving or uniform motion
  double g = sqrt(pow(xAccel,2)+pow(yAccel,2)+pow(zAccel,2));
  Serial.print("Measured acceleration WITH calibration :"); Serial.print(g); Serial.print("  ");Serial.println("m/s^2 ");

  /* Note: You can also get the raw (non unified values) for */
  /* the last data sample as follows. The .getEvent call populates */
  /* the raw values used below. */
  //Serial.print("X Raw: "); Serial.print(accel.raw.x); Serial.print("  ");
  //Serial.print("Y Raw: "); Serial.print(accel.raw.y); Serial.print("  ");
  //Serial.print("Z Raw: "); Serial.print(accel.raw.z); Serial.println("");

  /* Delay before the next sample */
  delay(600);
}
