/*   Daniel Dixneuf 8 décembre 2021
 *   ES2_exercice2_V1.1.ino
 *   Goals: 
 *        1) test the motors supply by the driver board, and test the empulsional sensor on axes
 *        2) test the direction servomotor driven the 2 front wheels
 *        3) test the ultrasonic servomotor driven the ultrasonic sensor
 *_________________________________________________________________________        
 *   done:
 *        - the independant supply for the 2 rear motors
 *   doing:
 *        - odometric sensor on the 2 motors M1 M2
 *            * instal library CaptureTimer to pick up the pulses from sensors
 * 
 * LeftMotor=M1B (EnB, In3, In4)    RightMotor=M2A(EnA, In1, In2)
 */
#include <Arduino_FreeRTOS.h>
#include "task.h"
#include "Arduino.h"
#include <avr/interrupt.h>
#include <QMC5883LCompass.h>


#define  LeftMotor_signalA 48
#define  LeftMotor_signalB 15
#define  RightMotor_signalA 49
#define  RightMotor_signalB 14
#include <LiquidCrystal.h>
#include <Servo.h>  // on inclut la bibliothèque pour piloter un servomoteur
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TimerOne.h>
unsigned long time; 
long int seconds=0; 
int minutes=0; 
int hours=0; 
int set=0; 
int reset=0; 
float wattHours =0;
float conso=0;
Adafruit_INA219 ina219;

QMC5883LCompass compass;
const int rs = 28, en = 27, d4 = 22, d5 = 23, d6 = 24, d7 = 25;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

char timeline[16];
//
int CmptPulse_RightMotor=0;
int CmptOVL_RightMotor=0;
int CmptPulse_LeftMotor=0;
int CmptOVL_LeftMotor=0;
int compteur_droite = 0;
int compteur_gauche = 0;
int i_US = 45;

// CAPTEUR ULTRASON
/* Constantes pour les broches */
const byte TRIGGER_PIN = A13; // Broche TRIGGER
const byte ECHO_PIN = A12;    // Broche ECHO
 
/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;


//Variables for the displacement of the wheel
float pi = 3.1415926535;
float radius = 32.5;
float distance_gauche = 0;
float distance_droite = 0;
float distance ;
bool flag;
//direction
#define DirPin      A8 
#define UlSonsPin   A9

int increment = 1;        //incrément entre chaque position
bool Direction_NowAngle = false;//Envoi sur le port série la position courante du servomoteur


//Right Motor (A)
#define Right_PinMotor1  13  //go front (980Hz)
#define Right_PinMotor2   9  //go rear
#define Right_MotorEn     8  //Enable mvt

#define Left_PinMotor3   4  //go front (980Hz)
#define Left_PinMotor4   5  //go rear
#define Left_MotorEn     6  //Enable mvt
//sensor
#define RightMotor_SensorAM1   18  //able to do interruptions
#define RightMotor_SensorBM1   14  //no interruption on this pin; BM1 in late 1/4T if CS2 goes and is in early if CS2 go back
#define LeftMotor_SensorAM2   19   //able to do interruptions
#define LeftMotor_SensorBM2   15   //no interruption on this pina
#define RightMotor_SensorAM1_LevelLow  digitalRead(RightMotor_SensorAM1)==0
#define RightMotor_SensorBM1_LevelLow  digitalRead(LeftMotor_SensorBM2)==0

#define HIGHA HIGH     //change HIGH in LOW to stop all mvts
#define LOWA LOW
#define HIGHB HIGH
#define LOWB LOW

#define derive 26

#define Enable_RightMotorA_On digitalWrite(Right_MotorEn,HIGHA)
#define Enable_RightMotorA_Off digitalWrite(Right_MotorEn,LOW)
#define Enable_LeftMotorB_On digitalWrite(Left_MotorEn,HIGHB)
#define Enable_LeftMotorB_Off digitalWrite(Left_MotorEn,LOW)
//steering wheels
byte Dir_CentralAngle=95;//#define Dir_CentralAngle 90
byte DirUltSonic_CentralAngle=90;


//_______ Global Variables declarations
int unsigned Xtest,XBtest;
int RSpeed;
int speed_RightMotor = 20;   // -100%< <100% de Nmax (avoid -20%< <20% because wheels not really run)
int speed_LeftMotor = -20;
long IncMotorSensorM1;
byte SteeringWheelsposition;              
byte Dir_InitialAngle = 60;   //initial angle (wheels on left hand)
byte Dir_FinalAngle = 140;    //final angle (wheels on right hand)
int x, y, z, a, b;
char myArray[3];
byte DirUltrasonicPosition;
float distance_mm;
long measure_US ;
#define DirUlSon_InitialAngle 45
#define DirUlSon_FinalAngle   135
int Mode = 1 ; 

Servo  myservoDir;
Servo  myservoUlSons;

TaskHandle_t RightDriveMotor_Handle,LeftDriveMotor_Handle,TestMvt_Handle;
TaskHandle_t InitStWheels_Handle,SteeringWheels_Handle,Dir_Ultrasonic_Handle,InitDirUltrasonic_Handle, Affiche_handle,Compas_handle, US_handle,Affiche_LCD_handle;

// INTERRUPTION FOR ODOMETRIC SENSOR

// The 2 16bits Timer (TIMER4 and TIMER5) are used in this code, which uses 2 inputs capture (ICP4 and ICP5).
// We maintain the high order 48 bits here by incrementing the virtual timer.

///////////////////// TIMER5_CAPT_vect and TIMER4_CAPT_vect: Interrupt capture handlers
//__ ICP5 Interruption
ISR(TIMER5_CAPT_vect) {//right drive Motor
  CmptPulse_LeftMotor++;
    if(digitalRead(15)==1) {
      compteur_gauche++;
      CmptPulse_LeftMotor++; 
      if(CmptPulse_LeftMotor>=30000) {CmptPulse_LeftMotor=0;CmptOVL_LeftMotor++;}
    }else{
      compteur_gauche--;
      CmptPulse_LeftMotor--;
      if(CmptPulse_LeftMotor<=(-30000)) {CmptPulse_LeftMotor=0;CmptOVL_LeftMotor--;}
    }
    
    distance_roue(0);
    //reset Interrupt Flag
    TIFR5 |= (1<<ICF5);
}
//__ ICP5 Interruption
ISR(TIMER4_CAPT_vect) {//the left motor rotor rotates opposite to right motor rotor
    if(digitalRead(14)==0) {
      compteur_droite++;
      CmptPulse_RightMotor++; 
      if(CmptPulse_RightMotor>=30000) {CmptPulse_RightMotor=0;CmptOVL_RightMotor++;}
    }else{
      compteur_droite--;
      CmptPulse_RightMotor--;
      if(CmptPulse_RightMotor<=(-30000)) {CmptPulse_RightMotor=0;CmptOVL_RightMotor--;}
    }
    distance_roue(1);
    //reset Interrupt Flag
    TIFR4 |= (1<<ICF4);
}

//_______ Prototypes
static void RightDriveMotor(void * pvParameters);
static void LeftDriveMotor(void * pvParameters);
static void TestMvt(void * pvParameters);
void Steering_Wheels_Task(void * pvParameters);
void Init_StWheels_Task(void * pvParameters);
void Direction_Ultrasonic_Task(void * pvParameters);
void Init_DirUltrasonic_Task(void * pvParameters);
//___________________________________ SETUP
void setup(){
    Serial.begin(115200);
    //--Set pins as outputs
      compass.init();
    pinMode(DirPin, OUTPUT);
    pinMode(Right_PinMotor1, OUTPUT);
    pinMode(Right_PinMotor2, OUTPUT);
    pinMode(Right_MotorEn, OUTPUT);
    pinMode(Left_PinMotor3, OUTPUT);
    pinMode(Left_PinMotor3, OUTPUT);
    pinMode(Left_MotorEn, OUTPUT);
    myservoDir.attach(DirPin);
    myservoUlSons.attach(UlSonsPin);
    pinMode(RightMotor_SensorAM1,INPUT_PULLUP);
    pinMode(LeftMotor_SensorAM2,INPUT_PULLUP);
    pinMode(RightMotor_SensorBM1,INPUT_PULLUP);
    pinMode(LeftMotor_SensorBM2,INPUT_PULLUP);
    pinMode(26,OUTPUT);
    pinMode(12,OUTPUT);
    digitalWrite(26,HIGH);
    analogWrite(12,10);
    lcd.begin(16, 2);

    /* Initialise les broches */
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
    pinMode(ECHO_PIN, INPUT);

//    monServo.attach(A9); // on définit le Pin utilisé par le servomoteur

    Timer1.start();
  uint32_t currentFrequency;
  

  ina219.begin();

    
    //ODO SENSORS SETUP
    //Right Drive Motor M1
   pinMode(RightMotor_signalA, INPUT_PULLUP);pinMode(RightMotor_signalB, INPUT_PULLUP);
   //Left Drive Motor M2
   pinMode(LeftMotor_signalA, INPUT_PULLUP);pinMode(LeftMotor_signalB, INPUT_PULLUP);
   //init and start Timer
   //initTimer();
   //Serial.println("timing...");
    //sermotors
    myservoDir.attach(DirPin);
    myservoUlSons.attach(A9);
    myservoUlSons.write(45);  
    myservoDir.write(Dir_CentralAngle);   
    //--init
    Xtest=0;XBtest=0;
    RSpeed=0;
    IncMotorSensorM1=0;
    SteeringWheelsposition = 120;
    //--interruptions
    //--Tasks 
    //Serial.println("la"); 
    xTaskCreate(TestMvt, "TestMvt", 100, NULL, 5, &TestMvt_Handle);//vTaskSuspend(TestMvt_Handle);
    xTaskCreate(RightDriveMotor, "RightDriveMotor", 100, NULL, 1, &RightDriveMotor_Handle);vTaskSuspend(RightDriveMotor_Handle);
    xTaskCreate(LeftDriveMotor, "LeftDriveMotor", 100, NULL, 1, &LeftDriveMotor_Handle);vTaskSuspend(LeftDriveMotor_Handle);    
    xTaskCreate(Steering_Wheels_Task, "Steering_Wheels_Task", 100, NULL, 1, &SteeringWheels_Handle);vTaskSuspend(SteeringWheels_Handle);
    xTaskCreate(Init_StWheels_Task, "Init_StWheels_Task", 100, NULL, 1, &InitStWheels_Handle);vTaskSuspend(InitStWheels_Handle);    
    xTaskCreate(Direction_Ultrasonic_Task, "Direction_Ultrasonic_Task", 100, NULL, 1, &Dir_Ultrasonic_Handle);vTaskSuspend(Dir_Ultrasonic_Handle);
    xTaskCreate(Init_DirUltrasonic_Task, "Init_DirUltrasonic_Task", 100, NULL, 1, &InitDirUltrasonic_Handle);vTaskSuspend(InitDirUltrasonic_Handle);
    xTaskCreate(Affiche, "Affiche", 100, NULL, 3, &Affiche_handle);vTaskSuspend(Affiche_handle);   
    xTaskCreate(Affiche_LCD, "Affiche_LCD", 100, NULL, 3, &Affiche_LCD_handle);//vTaskSuspend(Affiche_LCD_handle);   
    xTaskCreate(Compas, "Compas", 100, NULL, 2, &Compas_handle);vTaskSuspend(Compas_handle);  
    xTaskCreate(US, "US", 100, NULL, 2, &US_handle); vTaskSuspend(US_handle);  
    xTaskCreate(Idle_Task, "Idle_Task", 100, NULL, 0, NULL);
    Serial.println("Setup");
    //--let's go
    vTaskStartScheduler();  
}
//___________________________ Functions Descriptions
//_____Main
//__


/////////////////////////////init TIMER
void initTimer(void) { 
  //__Input Capture setup Right Motor
  // ICNC4: Enable Input Capture Noise Canceler
  // ICES4: =1 for trigger on rising edge, =0 falling edge
  // CS40: =1 set prescaler to 1x system clock (F_CPU), the TCNT1 register overflow with Freq=16MHz/1024
  TCCR4A = 0;
  TCCR4B = (1<<ICNC4) | (0<<ICES4) | (1<<CS40); //(0<<ICNC4)
  TCCR4C = 0;
  //catchFallingEdge(); // initialize to catch
  { TCCR4B &= ~(1<<ICES4); TIFR4 |= (1<<ICF4); }
  // Interrupt setup
  // ICIE4: Input capture 
  // TOIE4: TIMER4 overflow
  TIFR4 = (1<<ICF4);// | (1<<TOV1);    // clear pending interrupt capture flag and overflow timer flag
  TIMSK4 = (1<<ICIE4);// | (1<<TOIE4); // and enable
  
  //__Input Capture setup Left Motor
  TCCR5A = 0;
  TCCR5B = (1<<ICNC5) | (0<<ICES5) | (1<<CS50); //(0<<ICNC5)
  TCCR5C = 0;
  //catchFallingEdge(); // initialize to catch
  { TCCR5B &= ~(1<<ICES5); TIFR5 |= (1<<ICF5);  }
  TIFR5 = (1<<ICF5);// | (1<<TOV1);    // clear pending interrupt capture flag and overflow timer flag
  TIMSK5 = (1<<ICIE5);// | (1<<TOIE5); // and enable
}
void Affiche_LCD(){ 
  while(1){ 
 lcd.setCursor(0,1);
 lcd.print(distance_mm);lcd.print(" mm");
 vTaskDelay(500/portTICK_PERIOD_MS);
  
  }
  

  
}

void Affiche(){
  while(1){
  Serial.print("distance gauche :");
  Serial.println(distance_gauche); Serial.println(compteur_gauche);
  Serial.print("distance droite :");
  Serial.println(distance_droite);Serial.println(compteur_droite);
  Serial.print("compass   :");
  Serial.print(a); Serial.print("   "); Serial.println(b);
 
 /*   Serial.print(" Direction: ");
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);

  Serial.println();
  Serial.print("Distance :  ");
  Serial.println(distance_mm);*/
   vTaskDelay(500/portTICK_PERIOD_MS);
  }
}

void Compas(){
  while(1){ 
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  a = compass.getAzimuth();
 /*
  if(a+derive > 360 ){
    a = 360-(a+derive);
  }else{ 
    a= a+derive;
  }*/
  b = compass.getBearing(a);
  compass.getDirection(myArray, a);
  vTaskDelay(500/portTICK_PERIOD_MS);
  Serial.println("tache compass");
}
}

void US() {
  while(1){
    
    /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  measure_US = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
   
  /* 3. Calcul la distance à partir du temps mesuré */
  if(measure_US / 2.0 * SOUND_SPEED > 0){ 
    distance_mm = measure_US / 2.0 * SOUND_SPEED;
  }
  Serial.println("tache US");
  i_US = i_US + 3;
  myservoUlSons.write(i_US);
  //Scan du servo
  if(i_US == 135){
    i_US=45;
    myservoUlSons.write(i_US);
    flag = 1;
   // vTaskResume(TestMvt_Handle);
   //vTaskSuspend(US_handle);
       lcd.clear();
       lcd.setCursor(0,0);

       lcd.print("Distance US :");
       lcd.setCursor(0,1);
       lcd.print(distance_mm);
       lcd.print(" mm");
  }
  vTaskDelay(150/portTICK_PERIOD_MS);
  }
}

void Direction_Ultrasonic_Task(void * pvParameters){
  while(1){
           for (DirUltrasonicPosition = DirUlSon_InitialAngle; DirUltrasonicPosition <=DirUlSon_FinalAngle; DirUltrasonicPosition ++){ 
              myservoUlSons.write(DirUltrasonicPosition);  
              vTaskDelay(50/portTICK_PERIOD_MS);
              
           }
           for (DirUltrasonicPosition = DirUlSon_FinalAngle; DirUltrasonicPosition >=DirUlSon_InitialAngle; DirUltrasonicPosition --){ 
              myservoUlSons.write(DirUltrasonicPosition);  
              vTaskDelay(50/portTICK_PERIOD_MS);
           }
             
           //Serial.println("-------- Dir UltSonic");         
  }
}
void Init_DirUltrasonic_Task(void * pvParameters){
    //Serial.println("--------Dir UltSonic");
  while(1){
        if(DirUltrasonicPosition<=DirUltSonic_CentralAngle){
           for (DirUltrasonicPosition = DirUltrasonicPosition; DirUltrasonicPosition <=Dir_CentralAngle; DirUltrasonicPosition ++){ 
              myservoUlSons.write(DirUltrasonicPosition);  
              vTaskDelay(10/portTICK_PERIOD_MS);
           }
        }else{
           for (DirUltrasonicPosition = DirUltrasonicPosition; DirUltrasonicPosition >=Dir_CentralAngle; DirUltrasonicPosition --){ 
              myservoUlSons.write(DirUltrasonicPosition);  
              vTaskDelay(10/portTICK_PERIOD_MS);
           }          
        }
        vTaskSuspend(NULL);
  }
}
//__
void Steering_Wheels_Task(void * pvParameters){
  //Serial.println("-------------Init Steering_Wheels_Task");
  while(1){
          for (SteeringWheelsposition = Dir_InitialAngle; SteeringWheelsposition <=Dir_FinalAngle; SteeringWheelsposition ++){ 
              myservoDir.write(SteeringWheelsposition);  
              vTaskDelay(50/portTICK_PERIOD_MS);
              
           }
           for (SteeringWheelsposition = Dir_FinalAngle; SteeringWheelsposition >=Dir_InitialAngle; SteeringWheelsposition --){ 
              myservoDir.write(SteeringWheelsposition);  
              vTaskDelay(50/portTICK_PERIOD_MS);
           }    
  }
}

void Init_StWheels_Task(void * pvParameters){
    //Serial.println("--------Init StWheels");
  while(1){
        if(SteeringWheelsposition<=Dir_CentralAngle){
           for (SteeringWheelsposition = SteeringWheelsposition; SteeringWheelsposition <=Dir_CentralAngle; SteeringWheelsposition ++){ 
              myservoDir.write(SteeringWheelsposition);  
              vTaskDelay(10/portTICK_PERIOD_MS);
           }
        }else{
           for (SteeringWheelsposition = SteeringWheelsposition; SteeringWheelsposition >=Dir_CentralAngle; SteeringWheelsposition --){ 
              myservoDir.write(SteeringWheelsposition);  
              vTaskDelay(10/portTICK_PERIOD_MS);
           }          
        }
        vTaskSuspend(NULL);
  }
}
//__
static void TestMvt(void * pvParameters){
  vTaskResume(RightDriveMotor_Handle);
  vTaskResume(LeftDriveMotor_Handle);
  vTaskResume(InitStWheels_Handle);
  vTaskResume(InitDirUltrasonic_Handle);
  while(1){
  Serial.println("tache MVT");
   switch(XBtest){
    case 0:
        RSpeed=20;
    
        speed_LeftMotor=RSpeed;speed_RightMotor=1.1*RSpeed;
        vTaskResume(RightDriveMotor_Handle);
        vTaskResume(LeftDriveMotor_Handle);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        XBtest = 1;
        break; 
        
      case 1 :
          speed_LeftMotor=0;speed_RightMotor=0;
          Mode = 4; 
          vTaskResume(RightDriveMotor_Handle);
          vTaskResume(LeftDriveMotor_Handle);
          vTaskResume(US_handle);
          //vTaskSuspend(TestMvt_Handle);

          if(flag){
            vTaskSuspend(US_handle);
            flag = 0;
            XBtest = 2;
            Mode = 1;
          }
        break;

      case 2: 
        RSpeed=20;
        myservoDir.write(140);  
        vTaskDelay(50/portTICK_PERIOD_MS);
        speed_LeftMotor=RSpeed;speed_RightMotor=1.1* RSpeed;
        vTaskResume(RightDriveMotor_Handle);
        vTaskResume(LeftDriveMotor_Handle);
        vTaskDelay(1000/portTICK_PERIOD_MS);
          XBtest = 3;
        
        break;
        
       case 3 :
          myservoDir.write(95);  
          vTaskDelay(50/portTICK_PERIOD_MS);
          speed_LeftMotor=0;speed_RightMotor=0;
          vTaskResume(RightDriveMotor_Handle);
          vTaskResume(LeftDriveMotor_Handle);
          vTaskDelay(1000/portTICK_PERIOD_MS);
          XBtest = 0;
        break;
      }
    //Serial.print("----SMR=");Serial.print(speed_RightMotor);Serial.print(" ----SML=");Serial.print(speed_LeftMotor);Serial.print(" ----");Serial.print(Xtest);Serial.print("----");Serial.println(XBtest);
    vTaskDelay(300/portTICK_PERIOD_MS);   
  }
}
//_____Motors
static void RightDriveMotor(void * pvParameters){ //RightMotor=M2A(EnA, In1, In2)
int speed_LM;
  while(1){
    if(speed_RightMotor==0) {
      Enable_RightMotorA_Off;
    }
    else if (speed_RightMotor>0) {
        speed_LM=map(speed_RightMotor,0,100,255,0);
        analogWrite(Right_PinMotor2, 255); analogWrite(Right_PinMotor1, speed_LM); 
        Enable_RightMotorA_On;  
      }  
    else if (speed_RightMotor<0) {
        speed_LM=map(speed_RightMotor,-100,0,255,0);
        analogWrite(Right_PinMotor2, 0); analogWrite(Right_PinMotor1, speed_LM);
        Enable_RightMotorA_On;
      }  
     vTaskSuspend(NULL);       
    }
}
//__
static void LeftDriveMotor(void * pvParameters){// LeftMotor=M1B (EnB, In3, In4)
int speed_LM;
  while(1){
    if(speed_LeftMotor==0) {
      Enable_LeftMotorB_Off;
    }
    else if(speed_LeftMotor>0) {
        speed_LM=map(speed_LeftMotor,0,100,255,0);
        analogWrite(Left_PinMotor4, 255); analogWrite(Left_PinMotor3, speed_LM);  
        Enable_LeftMotorB_On;  
      }
    else if(speed_LeftMotor<0) {
        speed_LM=map(speed_LeftMotor,-100,0,255,0);
        analogWrite(Left_PinMotor4, 0); analogWrite(Left_PinMotor3, speed_LM);
        Enable_LeftMotorB_On;
      }
    vTaskSuspend(NULL);      
    }
}

 void distance_roue(bool sens) {
 int compteur;
 int compteur_2;
 float distance_2;
  if(sens == 0) {
   compteur = CmptPulse_LeftMotor;
   compteur_2 = CmptOVL_LeftMotor;
  } else {
    compteur = CmptPulse_RightMotor;
    compteur_2 = CmptOVL_RightMotor;
  }
  distance = (compteur * 2 * pi * radius) / 360;// There is 360 impulsions per rotation of the motor of the car
  distance_2 = ((compteur_2 * pi * radius) / 360) *30000;
  if(sens == 0) {
    distance_gauche = distance+distance_2;
  } else {
    distance_droite = distance+distance_2;
  }
}

//_________________________________ LOOP IDLE
void loop() {
    Serial.println("LOOP");
}

//__Idle Task
static void Idle_Task(void* pvParameters)
{ //Nothing here, for this application 
  while(1)
   { 
    Serial.println("idle task");
    //vTaskDelay(100/portTICK_PERIOD_MS);
   }  
}
