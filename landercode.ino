#include <Wire.h>
#include <SPI.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BMP280.h>
#include<SoftwareSerial.h>
#include<string.h>
#define filter_size 5
#define I2CADDR (0X76)
#define CHIPID (0x60)
#define WINDOW_SIZE 5

// Mode Codes
#define PRE_LAUNCH (100)
#define ASCENDING (200)
#define DESCENDING (300)
#define LANDED (400)


// HC-12 Wireless Transceiver
SoftwareSerial HC12(7, 6); // HC-12 TX Pin, HC-12 RX Pin

//Bosch Environmetal sensor (BME/BMP280)
Adafruit_BMP280 env_sensor;
Adafruit_Sensor *env_temp = env_sensor.getTemperatureSensor();
Adafruit_Sensor *env_pressure = env_sensor.getPressureSensor();

//variable to hold respective values
float temperature , pressure;
float x ,y,z;
float accel_x, accel_y , accel_z, prev_velx,velx, prev_vely,vely, prev_velz,velz;
unsigned long time_start,pre_launch_last_transmit_time,landed_last_transmit_time,up_time,down_time,land_time;
float prev_velTime=0;
float mass=0.2,forcex,forcey,forcez;
int INDEX = 0;
int VALUE = 0;
int SUM = 0;

// min/max variables
float min_temp, max_temp, min_pressure, max_pressure, min_alt, max_alt = 0;
float min_vel_x, max_vel_x, min_vel_y, max_vel_y, min_vel_z, max_vel_z = 0;
float min_accel_x, max_accel_x, min_accel_y, max_accel_y, min_accel_z, max_accel_z = 0;
float min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z = 0;

int avg_track_count;

float avg_velx,avg_vely,avg_velz,avg_vely_up,avg_vely_down;
float avg_forx,avg_fory,avg_forz,avg_alt,avg_accel_x,avg_accel_y,avg_accel_z;
float avg_temp,avg_pressure,sum_temp,sum_pressure;

float sum_velx,sum_vely,sum_velz,sum_vely_up,sum_vely_down;
float sum_forx,sum_fory,sum_forz,sum_alt;
float sum_accel_y,sum_accel_z,sum_accel_x;

int prev_mode, mode_code, mode = 0;

float takeoff_alt;

int READINGS_X[WINDOW_SIZE];
int READINGS_Y[WINDOW_SIZE];
int READINGS_Z[WINDOW_SIZE];
int AVERAGED = 0;
float  X_AVERAGED =0;
float  Y_AVERAGED =0;
float  Z_AVERAGED =0;

float pressure_avg, temp_avg = 0;



//functions
void transmit(String data_to_send){
  // Send data to PC for debugging
  Serial.println(data_to_send);
  
  // Send to HC-12 for wireless transmission
  HC12.println(data_to_send);

  delay(1);
}



void readFromAccelerometer(void){
  int raw_x = analogRead(A1);
  int raw_y = analogRead(A2);
  int raw_z = analogRead(A3);

  // Serial.print("x ");
  // Serial.println(raw_x);
  // Serial.print("y ");
  // Serial.println(raw_y);
  // Serial.print("z ");
  // Serial.println(raw_z);
  // Serial.println("");
  

  // String data1="";
  // data1=data1+"X "+raw_x;
  // transmit(data1);                      //battery callibration test
  //   data1="";
  // data1=data1+"Y "+raw_y;
  // transmit(data1);
  //   data1="";
  // data1=data1+"Z "+raw_z+"\n";
  // transmit(data1);
  // delay(750);


// Convert from raw accelerometer values to m/s^2 using pre-calibrated values

  //usb
  
  ////Define calibration range and the corresponding m/s^2 range (in this example, -9.81 to 9.81 m/s^2)
  // int raw_min_x = 290;    // Minimum raw value obtained during calibration for X-axis
  // int raw_max_x = 437; // Maximum raw value obtained during calibration for X-axis
  // int raw_min_y = 288;    // Minimum raw value obtained during calibration for Y-axis
  // int raw_max_y = 430; // Maximum raw value obtained during calibration for Y-axis
  // int raw_min_z = 294;    // Minimum raw value obtained during calibration for Z-axis
  // int raw_max_z = 438; // Maximum raw value obtained during calibration for Z-axis
  // int accel_min = -981; // Minimum m/s^2 value obtained during calibration
  // int accel_max = 981;

//battery
    
  //// Define calibration range and the corresponding m/s^2 range (in this example, -9.81 to 9.81 m/s^2)
  int raw_min_x = 270;    // Minimum raw value obtained during calibration for X-axis
  int raw_max_x = 406; // Maximum raw value obtained during calibration for X-axis
  int raw_min_y = 265;    // Minimum raw value obtained during calibration for Y-axis
  int raw_max_y = 400; // Maximum raw value obtained during calibration for Y-axis
  int raw_min_z = 272;    // Minimum raw value obtained during calibration for Z-axis
  int raw_max_z = 407; // Maximum raw value obtained during calibration for Z-axis
  int accel_min = -981; // Minimum m/s^2 value obtained during calibration
  int accel_max = 981;
  
  
  981; // Maximum m/s^2 value obtained during calibration
  
  // Map raw values to the range of m/s^2 using the calibration data
  float new_x = map(raw_x, raw_min_x, raw_max_x, accel_min, accel_max)/100;
  float new_y = map(raw_y, raw_min_y, raw_max_y,accel_min, accel_max)/100;
  float new_z = map(raw_z, raw_min_z, raw_max_z, accel_min, accel_max)/100;
  
  // Store value into global variables: accel_x, accel_y, accel_z.
  accel_x = new_x;
  accel_y = new_y-2;// calibration fosc y+=2 , z+=9
  accel_z = new_z;

//  Serial.print("acc: ");
//  Serial.print("X: ");
//  Serial.print(accel_x);
//  Serial.print(" Y:");
//  Serial.print(accel_y);
//  Serial.print(" Z:");
//  Serial.println(accel_z);
}


void readEnvironmental(void){

//read environmental sensor
sensors_event_t temp_event , pressure_event;
env_temp->getEvent(&temp_event);
env_pressure->getEvent(&pressure_event);

/* store value into variables: temperature (celcius), pressure(hPa) */
temperature = temp_event.temperature;
pressure = pressure_event.pressure;


}

float calcAltitude(float temperature, float pressure){
   
  float altitude;
 // Constants
  const float seaLevelPressure = 1013.25; // Standard sea-level pressure in hPa
  const float tempAtSeaLevel = 288.15;    // Standard temperature at sea level in K

  // Calculate altitude
  
  altitude = (((pow((seaLevelPressure/pressure), (1/5.257))-1) * (temperature + 273.15)) /0.0065)*-1;
  
  //float finalPressure = (seaLevelPressure/pressure)

 
  return altitude;
  
}

float calcVel(float timediff, float accel,float prev_vel){


  float vel= prev_vel + (accel*(timediff));
  prev_vel=vel;

 return vel;

}

float calcForce(float mass, float accel){


  float f=mass*accel;

  return f;


}
void smoothEnvSensorReading(float temperature, float pressure) {
    static float temperature_samples[filter_size];
    static float pressure_samples[filter_size];
    static float count = 0;

    // Store the current temperature and pressure in the arrays
    temperature_samples[(int)count] = temperature; // cast count to int to index the array
    pressure_samples[(int)count] = pressure; // cast count to int to index the array
    
    // Increment the index for next iteration
    count++;
    
    // If count reaches filter_size, calculate the average and reset the index
    if (count == filter_size) {
        float temperature_sum = 0; // Change to float
        float pressure_sum = 0; // Change to float
        
        // Calculate sum of temperature and pressure samples
        for (int i = 0; i < filter_size; i++) {
            temperature_sum += temperature_samples[i];
            pressure_sum += pressure_samples[i];
        }
        
        // Calculate averages
        temp_avg = temperature_sum / filter_size;
        pressure_avg = pressure_sum / filter_size;
        
        count = 0;
    }
}

void smoothAccelReading(){
  // Useful Example: https://maker.pro/arduino/tutorial/how-to-clean-up-noisy-sensor-data-with-a-moving-average-filter
  // Steps:
  // 1. index tells us which value we are replacing 
  // 2. Read in next Values
  // 3. calculate sum of array
  // 4. Store the averaged/smoothed sensor value into global variable
      // Read the next sensor value
          
      
  INDEX = (INDEX+1) % WINDOW_SIZE; 
   READINGS_X[INDEX] = accel_x;           // Add the newest reading to the window
  READINGS_Y[INDEX] = accel_y;           
  READINGS_Z[INDEX] = accel_z;  // Increment the index, and wrap to 0 if it exceeds the window size
  float sum_x =0, sum_y =0, sum_z =0;
  for(int i =0; i<WINDOW_SIZE; i++){
    sum_x += READINGS_X[i];
    sum_y += READINGS_Y[i];
    sum_z += READINGS_Z[i];
  }

  X_AVERAGED = sum_x /WINDOW_SIZE;
  Y_AVERAGED = sum_y / WINDOW_SIZE;
  Z_AVERAGED = sum_z /WINDOW_SIZE;

  accel_x = X_AVERAGED;
  accel_y = Y_AVERAGED;
  accel_z = Z_AVERAGED;
}

int detectMode(float curraltitude)
{
  // new mode
 
  float alt_diff;

  if (prev_mode == 0 & velx == 0){
  
   mode_code = 100;
   prev_mode=100; 
   //takeoff_alt=curraltitude;
   //Serial.println(prev_mode);

  }
  
  else if (prev_mode == 100){
    if (velx < -1 | velx > 1){
    up_time=millis() - time_start;
    
    mode_code=200;
    prev_mode=200;
   }
   // Serial.println(prev_mode);
  }

  else if(prev_mode==200 & vely<0){
  down_time=millis() - time_start;
    mode_code = 300;
    prev_mode = 300;
   // alt_diff=curraltitude - takeoff_alt;

  }

  else if (prev_mode == 300 & velx==0){
    land_time=millis() - time_start;
    mode_code=400;
    prev_mode=400;
  }

  // Serial.println(prev_mode);
  // Serial.println(mode_code);


  return mode_code;
}

// TRACK CALCULATED READINGS 
void trackAltitude(float curr_Time,float curraltitude){

    if (curraltitude > max_alt){
      max_alt = curraltitude;
    }
    if (avg_track_count == 1){
      min_alt = curraltitude;
    }
    else if (curraltitude < min_alt){
      min_alt = curraltitude;
    }
    sum_alt = sum_alt + curraltitude;
    avg_alt = sum_alt/avg_track_count;


}

void trackVelX(float curr_Time, float velx){

// record min, max, avg, velocity for X
    
    
    
    if (velx > max_vel_x){
      max_vel_x = velx;
    }
    if (avg_track_count == 1){
      min_vel_x = velx;
    }
    else if (velx < min_vel_x & velx != 0){
      min_vel_x = velx;
    }
    sum_velx = sum_velx + velx;
    avg_velx = sum_velx/avg_track_count;
    


}

void trackVelY(float curr_Time, float vely){

// record min, max, avg, velocity for Y

    if (vely > max_vel_y){
      max_vel_y = vely;
    }
    if (avg_track_count == 1){
      min_vel_y = vely;
    }
    else if (vely < min_vel_y & vely != 0){
      min_vel_y = vely;
    }
    sum_vely = sum_vely + vely;
    avg_vely = sum_vely/avg_track_count;

    if (vely>0){
    sum_vely_up = sum_vely_up + vely;
    avg_vely_up = sum_vely_up/avg_track_count;

    }
  else{
    sum_vely_down = sum_vely_down + vely;
    avg_vely_down = sum_vely_down/avg_track_count;

  }

}

void trackVelZ(float curr_Time,float velz){
 
// record min, max, avg, velocity for Z
    if (velz > max_vel_z){
      max_vel_z = velz;
    }
    if (avg_track_count == 1){
      min_vel_z = velz;
    }
    else if (velz < min_vel_z & velz != 0){
      min_vel_z = velz;
    }
    sum_velz = sum_velz + velz;
    avg_velz = sum_velz/avg_track_count;
}

void trackForceX(float curr_Time,float forcex){
 
// record min, max, avg, force for x
    if (forcex > max_force_x){
      max_force_x = forcex;
    }
    if (avg_track_count == 1){
      min_force_x = forcex;
    }
    else if (forcex < min_force_x & forcex != 0){
      min_force_x = forcex;
    }
    sum_forx = sum_forx + forcex;
    avg_forx = sum_forx/avg_track_count;
}

void trackForceY(float curr_Time,float forcey){
 
// record min, max, avg, force for y
    if (forcey > max_force_y){
      max_force_y = forcey;
    }
    if (avg_track_count == 1){
      min_force_y = forcey;
    }
    else if (forcey < min_force_y & forcey != 0){
      min_force_y = forcey;
    }
    sum_fory = sum_fory + forcey;
    avg_fory = sum_fory/avg_track_count;
}

void trackForceZ(float curr_Time,float forcez){
 
// record min, max, avg, force for z
    if (forcez > max_force_z){
      max_force_z = forcez;
    }
    if (avg_track_count == 1){
      min_force_z = forcez;
    }
    else if (forcez < min_force_z & forcez != 0){
      min_force_z = forcez;
    }
    sum_forz = sum_forz + forcez;
    avg_forz = sum_forz/avg_track_count;
}
void trackaccelY(float curr_Time){
 
// record min, max, avg, velocity for Y

    if (accel_y > max_accel_y){
      max_accel_y = accel_y;
    }
    if (avg_track_count == 1){
      min_accel_y = accel_y;
    }
    else if (accel_y < min_accel_y & accel_y != 0){
      min_accel_y = accel_y;
    }
    sum_accel_y = sum_accel_y + accel_y;
    avg_accel_y = sum_accel_y/avg_track_count;
}

void trackaccelX(float curr_Time){
 
// record min, max, avg, velocity for Y

    if (accel_x > max_accel_x){
      max_accel_x = accel_x;
    }
    if (avg_track_count == 1){
      min_accel_x = accel_x;
    }
    else if (accel_x < min_accel_x & accel_x != 0){
      min_accel_x = accel_x;
    }
    sum_accel_x = sum_accel_x + accel_x;
    avg_accel_x = sum_accel_x/avg_track_count;
}

void trackaccelZ(float curr_Time){
 
// record min, max, avg, velocity for Y

    if (accel_z > max_accel_z){
      max_accel_z = accel_z;
    }
    if (avg_track_count == 1){
      min_accel_z = accel_z;
    }
    else if (accel_z < min_accel_z & accel_z != 0){
      min_accel_z = accel_z;
    }
    sum_accel_z = sum_accel_z + accel_z;
    avg_accel_z = sum_accel_z/avg_track_count;
}

void tracktemp(float curr_Time){
 
// record min, max, avg, velocity for Y

    if (temperature > max_temp & temperature != 0){
      max_temp = temperature;
    }
    if (avg_track_count == 1){
      min_temp = temperature;
    }
    else if (temperature < min_temp & temperature!= 0){
      min_temp = temperature;
    }
    sum_temp = sum_temp + temperature;
    avg_temp = sum_temp/avg_track_count;
}
void trackpress(float curr_Time){
 
// record min, max, avg, velocity for Y

    if (pressure > max_pressure){
      max_pressure = pressure;
    }
    if (avg_track_count == 1){
      min_pressure = pressure;
    }
    else if (pressure < min_pressure & pressure != 0 ){
      min_pressure = pressure;
    }
    sum_pressure = sum_pressure + pressure;
    avg_pressure = sum_pressure/avg_track_count;
}
void setup(){
  delay(25000);
  time_start = millis();
   HC12.begin(9600);
  Serial.begin(9600);
 // Setup for the Environmental Sensor
  if (!env_sensor.begin(I2CADDR, CHIPID)) {
    Serial.println(F("Could not find a valid BME/BMP280 (Environmental) sensor, check CHIPID/libraries/wiring!"));
    while (1) delay(10);
  }

  // Default Adafruit_BMP280 settings from datasheet/
  env_sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                         Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                         Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                         Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                         Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  pre_launch_last_transmit_time = time_start;
  landed_last_transmit_time = time_start;
}

void loop(){
  float curr_Time=millis();
  avg_track_count++;

 readEnvironmental();
 readFromAccelerometer();
//smoothAccelReading();



 velx=0;
 vely=0;
 velz=0;
 float timediff = (curr_Time-prev_velTime)/1000;
// Serial.print("Vel: ");
 velx=calcVel(timediff,accel_x,prev_velx);


 //Serial.print("X:");
 //Serial.print(velx);
 
 vely=calcVel(timediff,accel_y,prev_vely);


// Serial.print(" Y:");
 //Serial.print(vely);
 
 velz=calcVel(timediff,accel_z,prev_velz);


 //Serial.print(" Z:");
 //Serial.println(velz);

 prev_velTime=curr_Time;


// Serial.print("Frc: ");
 forcex=calcForce(mass,accel_x);
// Serial.print("X:");
 //Serial.print(forcex);


 forcey=calcForce(mass,accel_y);
 //Serial.print(" Y:");
 //Serial.print(forcey);


 forcez=calcForce(mass,accel_z);
 //Serial.print(" Z:");
 //Serial.println(forcez);


 float curraltitude = calcAltitude(temperature,pressure) * -1; 
 //Serial.println(curraltitude);
 //Serial.println("");
 mode_code=detectMode(curraltitude);
 //Serial.println(mode_code);
  
if (mode_code != 400){
  trackAltitude(curr_Time,curraltitude);
  
  trackVelX(curr_Time,velx);
  trackVelY(curr_Time,vely);
  trackVelZ(curr_Time,velz);

  trackForceX(curr_Time,forcex);
  trackForceY(curr_Time,forcey);
  trackForceZ(curr_Time,forcez);
  trackaccelY(curr_Time);
  trackaccelX(curr_Time);
  trackaccelZ(curr_Time);
  tracktemp(curr_Time);
  trackpress(curr_Time);
}

 String data_to_send = "";
 //Serial.println(accel_x);
// delay(250);
 // Depending on mode - run certain calculations/ transmit particular data at different intervals
  switch(mode_code){
    case PRE_LAUNCH:
      // Only transmit once every 5s, use some kind of timer.
      if ((curr_Time - pre_launch_last_transmit_time) > 5000){
        data_to_send = "";
        data_to_send = data_to_send + "5 seconds since last PRE_LAUNCH transmit - transmit since start: " + curr_Time;
        Serial.println(data_to_send);
        data_to_send = "";
        // FORMAT: TIMESTAMP, MODE, VELOCITY (X,Y,Z), PRESSURE, ALTITUDE, TEMPERATURE
        data_to_send = data_to_send + "," + curr_Time + "," + mode_code + "," + velx + "," + vely + "," + velz + "," + pressure + "," + curraltitude + "," + temperature + "\n"; 

        // Transmit data
        transmit(data_to_send);

        // Record PRE_LAUNCH transmission time
        pre_launch_last_transmit_time = curr_Time;
        
      }
      else{
        Serial.println(F("5 seconds have not passed since previous PRE_LAUNCH transmit. Wait."));
      }
      break;
      
    case ASCENDING:
        data_to_send = "";
        data_to_send = data_to_send + "," + curr_Time + "," + mode_code + "," + velx + "," + vely + "," + velz + "," + pressure + "," + curraltitude + "," + temperature + "\n"; 

      // Transmit as fast as possible
      // Package data into a string
      // FORMAT: TIMESTAMP, MODE, VELOCITY (X,Y,Z), PRESSURE, ALTITUDE, TEMPERATURE
      //data_to_send = data_to_send + timestamp + "," + status + "," + x_vel + "," + y_vel + "," + z_vel + "," + pressure + "," + altitude + "," + temperature + "\n";  

      // Transmit data
      transmit(data_to_send);
      break;
      
    case DESCENDING:
        data_to_send = "";
        data_to_send = data_to_send + curr_Time + "," + mode_code + "," + velx + "," + vely + "," + velz +"," + pressure + "," + curraltitude + "," + temperature + "\n"; 
      // Transmit as fast as possible
      // Package data into a string
      // FORMAT: TIMESTAMP, MODE, VELOCITY (X,Y,Z), PRESSURE, ALTITUDE, TEMPERATURE
      //data_to_send = data_to_send + timestamp + "," + status + "," + x_vel + "," + y_vel + "," + z_vel + "," + pressure + "," + altitude + "," + temperature + "\n"; 
      // Transmit data
      transmit(data_to_send);
      break;

    case LANDED:
      if ((curr_Time - landed_last_transmit_time) > 10000) {
        // Provide summary e.g. of maximum values
      //  data_to_send = data_to_send + "Summary of results:" + "\n";
        data_to_send = data_to_send + "Altitude: Max: " + max_alt + " Min: "+min_alt+" AVG: "+avg_alt ;

        transmit(data_to_send);
        data_to_send = "";

        data_to_send = data_to_send + "Maximum upward acceleration (Y-axis): " + max_accel_y + "\n";
        data_to_send = data_to_send + "Maximum downward deceleration (Y-axis): " + min_accel_y + "\n";

       transmit(data_to_send);
        data_to_send = "";
       
        data_to_send = data_to_send + "average ascent velocity (Y axis): " + avg_vely_up + "\n";
        data_to_send = data_to_send + "average descent velocity (Y axis): " + avg_vely_down + "\n";
       // HC12.println(data_to_send);
       transmit(data_to_send);
        data_to_send = "";

        data_to_send = data_to_send + "Started pre-launch at: " + 0 + "s \n";
        data_to_send = data_to_send + "Started ascending at: " + up_time/1000 + "s";

        transmit(data_to_send);
        data_to_send = "";

        data_to_send = data_to_send + "Started descending at: " + down_time/1000 + "s \n";
        data_to_send = data_to_send + "landed at: " + land_time/1000 + "s \n";
        // Transmit other statistics...
        transmit(data_to_send);
        data_to_send = "";
        data_to_send = data_to_send + "X acceleration: Min: " + min_accel_x + " Max: " + max_accel_x + " AVG: "+ avg_accel_x ;
        transmit(data_to_send);
        data_to_send = "";
        data_to_send = data_to_send + "Y acceleration: Min: " + min_accel_y + " Max: " + max_accel_y + " AVG: "+ avg_accel_y ;
        transmit(data_to_send);
        data_to_send = "";
        data_to_send = data_to_send + "Z acceleration: Min: " + min_accel_z + " Max: " + max_accel_z + " AVG: "+ avg_accel_z +"\n";
        data_to_send = data_to_send + "\nX Velocity: Min: " + min_vel_x + " Max: " + max_vel_x + " AVG: "+ avg_velx ;
        transmit(data_to_send);
        data_to_send = "";
        data_to_send = data_to_send + "Y Velocity: Min: " + min_vel_y + " Max: " + max_vel_y + " AVG: "+ avg_vely +"\n";
        data_to_send = data_to_send + "Z Velocity: Min: " + min_vel_z + " Max: " + max_vel_z + " AVG: "+ avg_velz +"\n";
        transmit(data_to_send);
        data_to_send = "";
        data_to_send = data_to_send + "X Force: Min: " + min_force_x + " Max: " + max_force_x + " AVG: "+ avg_forx +"\n";
        data_to_send = data_to_send + "Y Force: Min: " + min_force_y + " Max: " + max_force_y + " AVG: "+ avg_fory ;
        transmit(data_to_send);
        data_to_send = "";
        data_to_send = data_to_send + "Z Force: Min: " + min_force_z + " Max: " + max_force_z + " AVG: "+ avg_forz +"\n";
        transmit(data_to_send);
        data_to_send = "";
        data_to_send = data_to_send + "Temperature: Min: " + min_temp + " Max: " + max_temp + " AVG: "+ avg_temp;
        transmit(data_to_send);
        data_to_send = "";
        data_to_send = data_to_send + "pressure: Min: " + min_pressure + " Max: " + max_pressure + " AVG: "+ avg_pressure +"\n";
        transmit(data_to_send);
        
        


        landed_last_transmit_time = curr_Time;
      }
      break;

    default:
      break;
  }
  delay(100);

}