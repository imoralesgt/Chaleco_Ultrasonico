#include <Ultrasonic.h>

// ----------- MISC. CONSTANTS -----------
#define MIN_DISTANCE 75  //75 cm. 
#define MAX_DISTANCE 150 //150 cm.
#define AVERAGE 5 //Moving average filter size
#define SAMPLING_DELAY 20 //Delay between measurements


// ----------- PIN-RELATED CONSTANTS -----------
// Ultrasonic sensors
const byte TRIG_L = 8;
const byte ECHO_L = 7;
const byte TRIG_R = 16;
const byte ECHO_R = 15;

//Motors
const byte MOTOR_L = 5;
const byte MOTOR_R = 4;

//Min distance potentiometer
const byte DISTANCE_POT = A0;


// ----------- MEAN AVERAGE DISTANCES ARRAYS -----------
unsigned int distancesL[AVERAGE];
unsigned int distancesR[AVERAGE];

// ----------- CURRENT MIN DISTANCE -----------
unsigned int minDistance;

// ----------- ULTRASONIC SENSOR INSTANES -----------
Ultrasonic pingL(TRIG_L, ECHO_L); //Trigger Pin, Echo Pin  
Ultrasonic pingR(TRIG_R, ECHO_R); //Trigger Pin, Echo Pin




// ----------- PROTOTYPES -----------
void checkSensorsAndRun();
void motorInit();
void filterInit(unsigned int startDist, unsigned int *distVector);
unsigned int filterData(unsigned int newData, unsigned int *distVector);
unsigned int sampleDistance(byte sensor);
void setMotorStatus(unsigned int motor, unsigned int averageValue);
unsigned int sampleSetPoint();


void setup(){
  //Serial.begin(9600);
  pinMode(DISTANCE_POT, INPUT);
  pingL.init();
  pingR.init();
  filterInit(MAX_DISTANCE, distancesL);
  filterInit(MAX_DISTANCE, distancesR);
  motorInit();
}

void loop(){
/*  
  //x = pingL.distance(1); //Measure distance in mode 1
  Serial.print("L-sensor: ");
  Serial.print(sampleDistance(0)); //Left Sensor
  //x = pingR.distance(1);
  Serial.print(" cm.  ------   R-sensor: ");
  Serial.print(sampleDistance(1)); //Right sensor
  Serial.println("cm. ");
*/
  checkSensorsAndRun();
  delay(SAMPLING_DELAY);

}



void motorInit(){
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);
  digitalWrite(MOTOR_L, 0);
  digitalWrite(MOTOR_R, 0);
}

void filterInit(unsigned int startDist, unsigned int *distVector){
  unsigned int i;
  for(i = 0; i < AVERAGE; i++){
    distVector[i] = startDist;
  }
}

unsigned int filterData(unsigned int newData, unsigned int *distVector){
  volatile unsigned int sum;
  unsigned int i; 
  sum = distVector[0];
  for(i = 1; i < AVERAGE; i++){
    distVector[i] = distVector[i-1];
    sum += distVector[i];
  }
  distVector[0] = newData;
  return sum / AVERAGE;
}

unsigned int sampleDistance(byte sensor){
  unsigned int x;
  if(!sensor){ //Left sensor
    return pingL.distance(1); 
  }else{
    return pingR.distance(1);
  }
}

void setMotorStatus(unsigned int motor, unsigned int averageValue){
  if(averageValue < sampleSetPoint()){
    digitalWrite(motor, 1);
  }else{
    digitalWrite(motor, 0);
  }
}

void checkSensorsAndRun(){
  unsigned int l, r;
  l = filterData(sampleDistance(0), distancesL); //Left sensor
  r = filterData(sampleDistance(1), distancesR); //Right sensor
  setMotorStatus(MOTOR_L, l);
  setMotorStatus(MOTOR_R, r);
}

unsigned int sampleSetPoint(){
  volatile unsigned int x;
  x = analogRead(DISTANCE_POT);
  x = map(x, 0, 1023, MIN_DISTANCE, MAX_DISTANCE);
  return x;
}

