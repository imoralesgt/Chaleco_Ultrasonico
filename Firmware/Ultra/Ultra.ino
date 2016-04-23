#include <Ultrasonic.h>

// ----------- MISC. CONSTANTS -----------
#define MIN_DISTANCE 100 //100 cm. 
#define AVERAGE 30 //Moving average filter size
#define SAMPLING_DELAY 50 //Delay between measurements


// ----------- PIN-RELATED CONSTANTS -----------
// Ultrasonic sensors
const byte TRIG_L = 8;
const byte ECHO_L = 7;
const byte TRIG_R = 16;
const byte ECHO_R = 15;

//Motors
const byte MOTOR_L = 5;
const byte MOTOR_R = 6;




// ----------- MEAN AVERAGE DISTANCES ARRAYS -----------
unsigned int distancesL[AVERAGE];
unsigned int distancesR[AVERAGE];



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



void setup(){
  Serial.begin(9600);
  pingL.init();
  pingR.init();
  filterInit(MIN_DISTANCE, distancesL);
  filterInit(MIN_DISTANCE, distancesR);
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
  if(averageValue < MIN_DISTANCE){
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

