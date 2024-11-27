/****************************************************************************
 * GPS ROBOT CAR
 * Navigate Autonomously GPS Waypoints Using Arduino
 * 
 * Magnetic Declination:
 * Fijate la declinacion magnetica de tu zona
 * http://www.magnetic-declination.com/
    (Buenos Aires -10* 2´)
 * The declination is provided in degrees and minutes format; you'll have to
 * convert to decimal degrees. Include the '-' sign if present.
 * Then, search this line and modify it:
 *const float mag_declination = -149.5°;
 * 
 * Digital Compass:
 * Pon el compass digital por lo menos 15cm por encima de todos los otros componentes
 * electronicos para evitar interferencias. The 'x' direction marked in the module
 * should point towards the front of the car.
 * 
 * GPS module:
 * Put the antenna near the center if possible, it doesn't matter much, though.
 * 
 * Motors:
 * Check the correct rotation direction of your motors. If any motor
 * is reversed, switch their terminals to correct.
 * 
 * I used global variables almost everywhere because I needed to print their
 * values with Print_Data() for debugging.
 * 
 * Additional resources for this project: http://bit.ly/GRCresources
 * or you can reach me at: raul@tecbolivia.com
*****************************************************************************/
// Libraries
#include <SoftwareSerial.h>
#include "TinyGPS.h"  // https://www.arduino.cc/reference/en/libraries/tinygps/
#include <Wire.h>
#include "I2Cdev.h"    // https://github.com/jrowberg/i2cdevlib
#include "HMC5883L.h"  // https://github.com/jrowberg/i2cdevlib
//librerias para la Estabilizacion
#include "MPU6050.h"//pines 21 scl y 20 sda
#include <Servo.h>
//__________________________________________________________defincion de los servos__________________________________________
Servo servo_1,servo_2;//el de la derecha adelante y atras respectivamente
Servo servo_3,servo_4;//el de la izquierda adelante y atras respectivamente
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
int s1,s2,s3,s4;
int a=1;
MPU6050 sensor;
int TiempoAhora = 0;
long tiempo_prev;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;
// Valores RAW (sin procesar)del giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;
#define TOLERANCE_RADIUS 1.0  // Tolerance radius for reaching the goal

// Robot car velocities. Change if you want to adjust the car's behavior
// (range of allowed velocities: 0 ~ 255)
#define adelante_VEL 230
#define SLOW_TURN_VEL 220
#define PIVOT_WHEEL_VEL 50
#define FAST_TURN_VEL 180

// Constants for defining error ranges for the stepped proportional control
#define MAX_HEADING_ANGLE 180
#define MIN_HEADING_ANGLE 5
#define ANGLE_RANGE_DIV 0.25

// Calibration constants for the motors in differential drive config.
// e. g. if the left motor is slower that the right one, you can add some
// % to the left and reduce the same % to the right, and viceversa.
#define K_RIGHT_MOTOR 1.0
#define K_LEFT_MOTOR 1.0

// Objects for interfacing with sensors
HMC5883L compass;                 // Create compass object
TinyGPS gps;                      // Create gps object
SoftwareSerial serial_gps(18,19);  // RX and TX pins in use
// Stores the next action the robot car should take (just for debugging)
String str_action;
// Control signal pins for the the motor driver x4
int pwma1 = 2 , pwma2 = 3; //derecha adelante
int en_a1= 22 , en_a2 = 22;
int pwmb1 = 4 , pwmb2 = 5; //izquierda adelante
int en_b1= 23 , en_b2 = 23;
int pwmc1 = 6 , pwmc2 = 7; //derecha atras
int en_c1= 24 , en_c2 = 24;
int pwmd1 = 8 , pwmd2 = 9; //izquierda atras
int en_d1= 25 , en_d2 = 25;
int led=13;
//pines correctamente configurados pwm
//____________________________________Ultrasonicos________________________________________________
int pd = 48;  // parlante derecha,trigger rigth
int pi = 49;  //parlante izquierda,trigger left
int md = 50;  // microfono derecha,echo right
int mi = 51;  //
int pc = 52;  //parlante centro,trigger mid
int mc = 53;  //microfono centro,echo mid
//pines no configurados
//variable del calculo
long dd, dc, di;  // distancia derecha,central,izquierda
long td, tc, ti;  // tiempo derecha,central,izquierda
//__________________________________________________________ Globals for storing waypoints__________________________________________________
// Struct to store waypoints (GPS coordinates):
struct t_waypoint {
  float lat;
  float lon;
};
//______________________________________________________________________IMPORTANTE______________________________________________________________
//cambiar las cordenadas por algunas de tu zona a las cuales quieras que viaje tu robot
// reach some place in my hometown! You can use Google Maps to define them:

// {lat, lon}
t_waypoint nav_waypoints[] = {
  { -17.4176, -66.13265 },   // Point 1
  { -17.41757, -66.13282 },  // Point 2
  { -17.41773, -66.13282 },  // Point 3
  { -17.41776, -66.13265 }   // Point 4
};


byte waypoint_index = 0;  // Index to the current waypoint in array
// Get the number of waypoints:
int num_waypoints = sizeof(nav_waypoints) / sizeof(nav_waypoints[0]);

// Auxiliary variables for computing the navigation vector:
float waypoint_lat_rad;
float waypoint_lon_rad;

//------- Globals for compass readings
int16_t mx, my, mz;   // Earth's magnetic field components
float compass_angle;  // Stores the angle retrieved from the compass

// La declinacion magnetica de tu zona la cual buscaste al inicio
// defined for your area/city or leave it at zero:
const float mag_declination = -149.5;

int heading_error;  // Stores the heading error

//____________________________________________________________Globals for GPS readings
float f_gps_reading_lat, f_gps_reading_lon;  // Stores the reading from the GPS receiver
float waypoint_angle;                        // Stores the azimuth to the current goal
float last_calc_dist = 0;                    // Las computed distance to goal (just for debugging)

// Auxiliaries for dumping GPS data
long lat, lon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;

//------- Globals for the moving average filter
// Increse NUM_FILTERING_POINTS if you want more filtering of the GPS readings,
// it also adds more delay to the system. Sometimes less could be better.
#define NUM_FILTERING_POINTS 15
float buffer_gps_lat[NUM_FILTERING_POINTS];  // Buffer for latitudes
float buffer_gps_lon[NUM_FILTERING_POINTS];  // Buffer for longitudes
t_waypoint gps_filtered;                     // For storing a filtered GPS reading
unsigned int buffer_fill_index = 0;          // Index for filling filter buffers

/*________________________________________________ Setup function_______________________________________________________________ */
void setup() {
  // Initialize control pins for the motor driver
  pinMode(pwma1, OUTPUT);  pinMode(pwma2, OUTPUT);
  pinMode(pwmb1, OUTPUT);  pinMode(pwmb2, OUTPUT);
  pinMode(pwmc1, OUTPUT);  pinMode(pwmc2, OUTPUT);
  pinMode(pwmd1, OUTPUT);  pinMode(pwmd2, OUTPUT);
  pinMode(en_a1, OUTPUT);  pinMode(en_a2, OUTPUT);
  pinMode(en_b1, OUTPUT);  pinMode(en_b2, OUTPUT);
  pinMode(en_c1, OUTPUT);  pinMode(en_c2, OUTPUT);
  pinMode(en_d1, OUTPUT);  pinMode(en_d2, OUTPUT);

  Stop();  // Stop the car

  Wire.begin();            // Initialize I2C comm. for the compass
  Serial.begin(9600);      // Initialize serial comm. for debugging
  serial_gps.begin(9600);  // Initialize serial comm. with the GPS module

  compass.initialize();  // Initialize the compass
  //Estabilizacion
  sensor.initialize();    //Iniciando el sensor
  //configuracion de los sensores
  servo_1.attach(2, 500, 2500);  servo_2.attach(4, 500, 2500);
  servo_3.attach(6, 500, 2500);  servo_4.attach(8, 500, 2500);//cambiar el Pin de cada uno(es el primer valor)
  //todos los servos en angulo 0
  servo_1.write(45); servo_2.write(45); servo_3.write(45); servo_4.write(45);
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
  delay(1000);
}

/*___________________________________________________________Main loop__________________________________________________________________ */
void loop() {
  // Uncomment the following line and comment the rest in this function if
  // you want to verify the rotation direction of your motors:
    while(a==1){
  //  adelante(adelante_VEL);
  bluetooth();
  MPU();
   TiempoAhora++;
  if(TiempoAhora = 100){
  Get_Compass_Heading();  // Read the digital compass

  if (Query_Gps()) {  // Query the GPS
    Gps_Dump(gps);    // Read the GPS

    // Store the new GPS reading in filter buffers
    Store_Gps_Reading(f_gps_reading_lat, f_gps_reading_lon);

    // Get filtered GPS latitude and longitude
    gps_filtered = Compute_Filtered_Gps();
    
    // Compute distance and heading to the goal
    Compute_Navigation_Vector(gps_filtered.lat, gps_filtered.lon);

    // Print data to the PC just for debugging
    Print_Data();
  }
  TiempoAhora =TiempoAhora-TiempoAhora; Serial.println(TiempoAhora);
  }
  // Move the car to the goal
  Control_Navigation();
  //evita que el auto choque
  esquivar();
    }}
    //___________________________________________Fin del Loop_____________________________________________________
/* Stores a GPS reading in the moving average filter's buffers */
void Store_Gps_Reading(float lat, float lon) {
  // Shift all buffer values towards the tail
  for (int i = (NUM_FILTERING_POINTS - 1); i > 0; --i) {
    buffer_gps_lat[i] = buffer_gps_lat[i - 1];
    buffer_gps_lon[i] = buffer_gps_lon[i - 1];
  }

  // Insert new values at the head
  buffer_gps_lat[0] = lat;
  buffer_gps_lon[0] = lon;

  // Increment the number of readings stored in buffers
  if (buffer_fill_index < NUM_FILTERING_POINTS) {
    ++buffer_fill_index;
  }
}
//__________________________________________________________________________________________________________________________________________
/* Computes filtered latitude and longitude using a moving average filter */
t_waypoint Compute_Filtered_Gps() {
  float lat_sum;
  float lon_sum;

  t_waypoint filtered_waypoint;

  lat_sum = 0;
  lon_sum = 0;

  // Add all values in each buffer
  for (int i = 0; i < buffer_fill_index; ++i) {
    lat_sum = lat_sum + buffer_gps_lat[i];
    lon_sum = lon_sum + buffer_gps_lon[i];
  }

  // Take the average
  filtered_waypoint.lat = lat_sum / float(buffer_fill_index);
  filtered_waypoint.lon = lon_sum / buffer_fill_index;

  return filtered_waypoint;  // Return filtered values
}
//_____________________________________________________________Consulta el gps__________________________________________________________
/* Queries the GPS receiver */
bool Query_Gps() {
  while (serial_gps.available()) {
    if (gps.encode(serial_gps.read())) {
      return true;
    }
  }
  return false;
}
//_____________________________________________________________Lectura del Compass____________________________________________________________
/* Gets a reading from the compass */
void Get_Compass_Heading(void) {
  // Obtain magnetic field components in x, y and z
  compass.getHeading(&mx, &my, &mz);

  // Calculate the X axis angle W.R.T. North
  compass_angle = atan2(my, mx);
  compass_angle = compass_angle * RAD_TO_DEG;       // RAD_TO_DEG = 180/M_PI
  compass_angle = compass_angle - mag_declination;  // Compensate for magnetic declination

  // Always convert to positive angles
  if (compass_angle < 0) {
    compass_angle = compass_angle + 360;
  }
}
//_____________________________________________________________________________________________________________________________________________
/* Computes the navigation vector */
void Compute_Navigation_Vector(float gps_lat, float gps_lon) {
  t_waypoint cur_waypoint;

  // Get current goal
  cur_waypoint = Get_Waypoint_With_Index(waypoint_index);

  float gps_f_lat_rad;
  float gps_f_lon_rad;
  float a_haversine = 0;
  float c_haversine = 0;
  float d_haversine = 0;
  float parcial = 0;
  float delta_lat = 0;
  float delta_lon = 0;

  // Compute the distance to the goal with Haversine formula
  //_____________________________________________________Distancia hasta la meta_________________________________________________
  delta_lat = radians(cur_waypoint.lat - gps_lat);
  gps_f_lat_rad = radians(gps_lat);
  waypoint_lat_rad = radians(cur_waypoint.lat);
  delta_lon = radians(cur_waypoint.lon - gps_lon);

  a_haversine = sin(delta_lat / 2.0) * sin(delta_lat / 2.0);
  parcial = cos(gps_f_lat_rad) * cos(waypoint_lat_rad);
  parcial = parcial * sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
  a_haversine += parcial;

  c_haversine = 2 * atan2(sqrt(a_haversine), sqrt(1.0 - a_haversine));

  d_haversine = 6371000.0 * c_haversine;  // Multiply by Earth's radius in meters

  last_calc_dist = d_haversine;

  // Check if we are inside the goal's tolerance radius
  if (d_haversine < TOLERANCE_RADIUS) {
    Stop();            // Stop the car
    delay(3000);       // Delay just to check visually were exactly the car reached the goal
    Serial.println("se alcanzo la meta");
    digitalWrite(led,HIGH);
    waypoint_index++;  // Switch to the next waypoint
    delay(100);  // Beep to signal 'waypoint reached'
  }

  // Check if we reached all waypoints
  if (waypoint_index == num_waypoints) {
    Stop();  // Stop the car
    delay(100);
    while (1)      ;  // Stop the program (reset Arduino board to repeat)
  }
  // Compute the adelante azimuth
  // **************************************************
  gps_f_lon_rad = radians(gps_lon);
  waypoint_lon_rad = radians(cur_waypoint.lon);

  waypoint_angle = atan2(sin(waypoint_lon_rad - gps_f_lon_rad) * cos(waypoint_lat_rad),
                         cos(gps_f_lat_rad) * sin(waypoint_lat_rad) - sin(gps_f_lat_rad) * cos(waypoint_lat_rad) * cos(waypoint_lon_rad - gps_f_lon_rad));

  waypoint_angle = waypoint_angle * 180 / PI;  // Convert from radians to degrees

  // Always convert to positive angles
  if (waypoint_angle < 0)
  {   waypoint_angle += 360; }
}
//_____________________________________________________________Camino a la Meta__________________________________________
/* Controls the robot car navigation towards the goal */
void Control_Navigation() {
  heading_error = (waypoint_angle - compass_angle);  // Compute the heading error

  // Correct angle for wrap around
  if (heading_error < -180) {
    heading_error = heading_error + 360;
  }
  if (heading_error > 180) {
    heading_error = heading_error - 360;
  }

  // ----- Stepped proportional control
  // The error is between +5 and +45 (for ANGLE_RANGE_DIV = 0.25):
  if (heading_error > MIN_HEADING_ANGLE && heading_error <= MAX_HEADING_ANGLE * ANGLE_RANGE_DIV) {
    // Turn right
    giro_derecha(SLOW_TURN_VEL);
    str_action = "Right";
  }

  // The error is between +45 and +180 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error > MAX_HEADING_ANGLE * ANGLE_RANGE_DIV && heading_error <= MAX_HEADING_ANGLE) {
    // Turn right fast
    giro_rapido_derecha(FAST_TURN_VEL);
    str_action = "Right fast";
  }

  // The error is between -5 and -45 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error < -MIN_HEADING_ANGLE && heading_error >= -MAX_HEADING_ANGLE * ANGLE_RANGE_DIV) {
    // Turn left
    giro_izquierda(SLOW_TURN_VEL);
    str_action = "Left";
  }

  // The error is between -45 and -180 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error < -MAX_HEADING_ANGLE * ANGLE_RANGE_DIV && heading_error >= -MAX_HEADING_ANGLE) {
    // Turn left fast
    giro_rapido_izquierda(FAST_TURN_VEL);
    str_action = "Left fast";
  }
  // The error is between -5 and +5 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error >= -MIN_HEADING_ANGLE && heading_error <= MIN_HEADING_ANGLE) {
    // adelante
    adelante(adelante_VEL);
    str_action = "adelante";
  }
  // Just for debugging:
  else {
    str_action = "Not defined";
  }
}
//_______________________________________________________________________________________________________________________________________________________
/*  TinyGPS auxiliary routine, comes with the library */
void Gps_Dump(TinyGPS &gps) {
  gps.get_position(&lat, &lon, &age);

  //  Query_Gps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&f_gps_reading_lat, &f_gps_reading_lon, &age);

  Query_Gps();

  gps.stats(&chars, &sentences, &failed);
  //  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: "); Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}
//___________________________________________________________________________________________________________________________________--

/* Returns a waypoint from the array */
struct t_waypoint Get_Waypoint_With_Index(int index) {
  return nav_waypoints[index];
}
//_________________________________________________________________________Movimiento______________________________________________________________--
/* Moves the car adelante */
void adelante(unsigned char vel) {
  analogWrite(pwma1, LOW);  analogWrite(pwma2, vel * K_RIGHT_MOTOR);
  analogWrite(pwmb1, LOW);  analogWrite(pwmb2, vel * K_LEFT_MOTOR);
  analogWrite(pwmc1, LOW)  ;  analogWrite(pwmc2, vel * K_RIGHT_MOTOR);
  analogWrite(pwmd1, LOW);  analogWrite(pwmd2, vel * K_LEFT_MOTOR);
}

/* Turns the car left slowly */
void giro_izquierda(unsigned char vel) {
  analogWrite(pwma1, LOW);  analogWrite(pwma2, vel * K_RIGHT_MOTOR);
  analogWrite(pwmb1, 0);    analogWrite(pwmb2, PIVOT_WHEEL_VEL * K_LEFT_MOTOR);
  analogWrite(pwmc1, LOW);  analogWrite(pwmc2, vel * K_RIGHT_MOTOR);
  analogWrite(pwmd1, 0);    analogWrite(pwmd2, PIVOT_WHEEL_VEL * K_LEFT_MOTOR);
}

/* Turns the car right slowly */
void giro_derecha(unsigned char vel) {
  analogWrite(pwma1, 0);    analogWrite(pwma2, PIVOT_WHEEL_VEL * K_RIGHT_MOTOR);
  analogWrite(pwmb1, LOW);  analogWrite(pwmb2, vel * K_LEFT_MOTOR);
  analogWrite(pwmc1, 0); analogWrite(pwmc2, PIVOT_WHEEL_VEL * K_RIGHT_MOTOR);
  analogWrite(pwmd1, LOW);  analogWrite(pwmd2, vel * K_LEFT_MOTOR);  
}

/* Turns the car left fast pivoting */
void giro_rapido_izquierda(unsigned char vel) {
  // Solo motor derecho gira con alta velocidad, el izquierdo se detiene
  analogWrite(pwma1, LOW);  analogWrite(pwma2, vel * K_RIGHT_MOTOR);
  analogWrite(pwmb1, vel * K_LEFT_MOTOR);  analogWrite(pwmb2, LOW);
  analogWrite(pwmc1, LOW); analogWrite(pwmc2, vel * K_RIGHT_MOTOR);
  analogWrite(pwmd1, vel * K_LEFT_MOTOR);  analogWrite(pwmd2, LOW);
}

/* Turns the car right fast pivoting */
void giro_rapido_derecha(unsigned char vel) {
  analogWrite(pwma1, vel * K_RIGHT_MOTOR);  analogWrite(pwma2, LOW);
  analogWrite(pwmb1, LOW);                 analogWrite(pwmb2, vel * K_LEFT_MOTOR);
  analogWrite(pwmc1, vel * K_RIGHT_MOTOR);  analogWrite(pwmc2, LOW);
  analogWrite(pwmd1, LOW); analogWrite(pwmd2, vel * K_LEFT_MOTOR);
}

//____________________________________________ Stop the car
void Stop(void) {
  analogWrite(pwma1, LOW);  analogWrite(pwma2, LOW);
  analogWrite(pwmb1, LOW);  analogWrite(pwmb2, LOW);
  analogWrite(pwmc1, LOW);  analogWrite(pwmc2, LOW);
  analogWrite(pwmd1, LOW);  analogWrite(pwmd2, LOW);
}

/* Prints data to the serial port for debugging */
/* Open the Serial Plotter in the Arduino IDE to see a graph of the unfiltered
   and filtered latitude. Comment/uncomment sections to see other data.
*/
void Print_Data(void) {
  // Original and filtered latitude (multiplied by a factor to help the
  // Serial Plotter's minimal capabilities!)
  Serial.print("GPSLAT: ");
  Serial.print(1000000 * f_gps_reading_lat, 1);
  //  Serial.print(" GPSLATF: "); Serial.print(1000000*gps_filtered.lat, 1);

  // Original and filtered longitude
  //  Serial.print(" GPSLON: "); Serial.print(1000000*(f_gps_reading_lon), 1);
  //  Serial.print(" GPSLONF: "); Serial.print(1000000*(gps_filtered.lon), 1);

  // Waypoint angle, robot car angle and heading error
  //  Serial.print(" CMPANG: "); Serial.print(compass_angle);
  //  Serial.print(" GPSANG: "); Serial.print(waypoint_angle);
  //  Serial.print(" HDERR: "); Serial.print(heading_error);

  // Put here other data you want to plot
  Serial.println();
}
//_________________________________________________________deteccion ultrasonicos__________________________________________________
void ultrasonicos() {
  //deteccion objetivo ultrasonico derecho
  //deteccion objetivo ultrasonico derecho
  digitalWrite(pd, HIGH);  delayMicroseconds(30); 
  digitalWrite(pd, LOW);
  td = pulseIn(md, HIGH);  dd = td / 59;
  Serial.print("distancia derecha=");  Serial.println(dd); // delay(30);
  //deteccion objetivo ultrasonico izquierdo
  digitalWrite(pi, HIGH);  delayMicroseconds(30);
  digitalWrite(pi, LOW);
  ti = pulseIn(mi, HIGH);  di = ti / 59;
  Serial.print("distancia izquierda=");  Serial.println(di);//  delay(30);
  //deteccion objetivo ultrasonico central
  digitalWrite(pc, HIGH);  delayMicroseconds(30); 
  digitalWrite(pc, LOW);
  tc = pulseIn(mc, HIGH);  dc = tc / 59;
  Serial.print("distancia medio=");  Serial.println(dc); // delay(30);
  //ang_x_prev=ang_x-ang_x;
  //ang_y_prev=ang_y-ang_y;
}
//______________________________________________________________Servos________________________________________________________________________
void MPU(){

  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  
  int dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
    Serial.println(ang_y_prev); 
    Serial.println(gy); 
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;

  //Mostrar los angulos separadas por un [tab]

  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x); 
  Serial.print("tRotacion en Y: ");
  Serial.println(ang_y);

  delay(10);
}
void Estabilizacion(){
//----------------------------------------------------combinados-------
if(gx<=-5 && gy>=5){
servo_4.write(s4++);   servo_3.write(s3++); 
servo_2.write(s2--);   servo_1.write(s1--);   Serial.println("Combinada 1"); } 
if(gx>=5 && gy<=-5){
servo_4.write(s4--);   servo_3.write(s3--); 
servo_2.write(s2++);   servo_1.write(s1++);   Serial.println("Combinada 2"); } 

//-------------------------------------------------Base---------------------
else if(gx<=-5)
{ servo_4.write(s4++);   servo_3.write(s3++);   Serial.println("Aumentando el angulo de las patas laterales derechas hasta estabilizar"); }
else if(gx>=5)
{ servo_2.write(s2--);   servo_1.write(s1--);   Serial.println("disminuyendo el angulo de las patas laterales izquierda hasta estabilizar"); }
if(gy<=-5)
{ servo_1.write(s1++);   servo_3.write(s3++);   Serial.println("Aumentando el angulo de las patas traseras hasta estabilizar"); 
 } //No es gx lo que tiene que aumentar si no el angulo del servo
else if(gy>=5)
{ servo_2.write(s2--);   servo_4.write(s4--);   Serial.println("disminuyendo el angulo de las patas traseras hasta estabilizar");  }
}
//_________________________________________________________Esquivar y seguir__________________________________________________________________
void esquivar(){
  if(di<<20 || dc<<20)
  {  giro_rapido_derecha(FAST_TURN_VEL);    delay(20);  }
  if(dd<<20)
  {  giro_rapido_izquierda(FAST_TURN_VEL);    delay(20); }
}
//___________________________________________________-BLUETOOTH_____________________________________________________
void bluetooth(){
    if (Serial.available())
  {
    char dato=Serial.read();
    Serial.print("Dato recibido: ");
    Serial.println(dato);
        switch(dato){
      case 'a':
        adelante(adelante_VEL);
        break;
      case 'r':
    //    Atras();
        break;
      case 'd':
        giro_derecha(SLOW_TURN_VEL);
        break;
      case 'i':
        giro_izquierda(FAST_TURN_VEL);
        break;
      case 'p':
        Stop();
        break;
    }    
  }
}