
/*************************************************************************
  v1.13 06/03/2019 Elimino librería GkBuzzer.h y GeekeBrain.h q venia de makeblock q
        solo se utilizaba para el servo y no lo gestionaba bien. Encima se reduce un monton el tamaño del compilado.                    
  
  v1.12 26/02/2019 Cambiar loops en base a timeStamp
                    - Ahora GkEngine Motor 1 paso +/- 1 centimetro
  v1.11 20/02/2019 Cambiar el sentido del GkEngineMotor.

  v1.10 24/01/2019 Ajustes del GkEngineMotor.
        Cambiarle el sentido de giro.
        Ajustar 100 pasos -> 1 vuelta

  v1.09 28/12/2018 Incorpora GkGyro.
 
  v1.08 13/12/2018 Mejorar gestión de bloques asincronos.

  v1.7 07/12/2018 No enviaba datos el GKSonic.

  v1.6 28/11/2018 No devolvía bien getDigital para los pulsadores.

  v1.5.1 21/09/2018 Mejora ultrasonidos.

  v1.5 31/05/2018 Mejora ultrasonidos.

  v1.4 15/05/2018 Lectura de digital, analogico, ultrasonidos.

  v1.3 13/05/2018 Cambios para GkEngine S en modo sin Bluetooth

  v1.2 07/05/2018 Cambios para GkEngine M en modo sin Bluetooth

  v1.1 23/04/2018 Correcciones GkEngine con puente H evitando PWM.

  v1.0 17/04/2018 Correcciones GkEngine+.

  v0.6 02/04/2018 Se añade el GkEngine+ servos 360 para las ruedas.

  v0.5 20/02/2018 Se añade control de servo sin utilizar PWM para no afectar al puente H de los motores.

**************************************************************************/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Servo.h>

// Poner versión en formato 1.09 para que quitando el . se convierta en 109 sea facil chequear la versión
String versionGlobal = "1.09";

//Servo servos[8];

//GkBuzzer buzzer;

typedef struct MeModule {
  int device;
  int port;
  int slot;
  int pin;
  int index;
  float values[3];
} MeModule;

union {
  byte byteVal[4];
  float floatVal;
  long longVal;
} val;

union {
  byte byteVal[8];
  double doubleVal;
} valDouble;

union {
  byte byteVal[2];
  short shortVal;
} valShort;

MeModule modules[12];


boolean isAvailable = false;
boolean isBluetooth = false;

int len = 52;
char buffer[52];
char bufferBt[52];
byte index = 0;
byte dataLen;
byte modulesLen = 0;
boolean isStart = false;
unsigned char irRead;
char serialRead;
#define VERSION 0
#define ULTRASONIC_SENSOR 1
#define TEMPERATURE_SENSOR 2
#define LIGHT_SENSOR 3
#define POTENTIONMETER 4
#define JOYSTICK 5
#define GYRO 6
#define SOUND_SENSOR 7
#define RGBLED 8
#define SEVSEG 9
#define MOTOR 10
#define SERVO 11
#define ENCODER 12
#define IR 13
#define IRREMOTE 14
#define PIRMOTION 15
#define INFRARED 16
#define LINEFOLLOWER 17
#define IRREMOTECODE 18
#define SHUTTER 20
#define LIMITSWITCH 21
#define BUTTON 22
#define HUMITURE 23
#define FLAMESENSOR 24
#define GASSENSOR 25
#define COMPASS 26
#define DIGITAL 30
#define ANALOG 31
#define PWM 32
#define SERVO_PIN 33
#define TONE 34
#define PULSEIN 37
#define ULTRASONIC_ARDUINO 36
#define STEPPER 40
#define LEDMATRIX 41
#define TIMER 50
#define TOUCH_SENSOR 51
#define ENGINE_S 60
#define GKLIGHT_SENSOR 100

//#define GK_MOTOR 100

#define GET 1
#define RUN 2
#define RESET 4
#define START 5
//float angleServo = 90.0;
int servo_pins[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char prevc = 0;
double lastTime = 0.0;
double currentTime = 0.0;
uint8_t keyPressed = 0;
uint8_t command_index = 0;

//------------------------------------------------------------------
// GeekeBrain T E R O
//
// Equivalencia puertos/pines Version Tero
int pinOfPortGlobal[9] = {0,    3, 4, 7, 6,    10, 11, 12, 13};
// En la placa GeekeBrain 1.0 en moto 8 puertos, aún no funciona el A0, y A1,
// porque esta en el primero que no se suelda
// TODO: redirigir en la placa el A0 y el A1 al puerto 4 del Tero para q este disponible
int pinAnalogAOfPort[8] = {0,   A2, A7, A4, A0};
int pinAnalogBOfPort[8] = {0,   A3, A6, A5, A1};
//------------------------------------------------------------------


unsigned long millisGlobal = 0;

// Servo sin libreria
//volatile int loopsServo[13] = {0, 0, 0, 0, 0, 0, 0, 0};
// En el GKTero solo hay 8 posibles puertos para el servo.
int servos180AngleGlobal[8] = { -1, -1, -1, -1, -1, -1, -1, -1};
int servos360Vel = 0;
int servos360Steps = 0;
int servos360Loops = 0;


Servo servos[13];
//Servo servoI;
//Servo servoD;


/****************************************************************************************
   GkServo 
*/
bool servoIsActiveGlobal = false;

void servoStop() {
    servoIsActiveGlobal = false;
}

void servoInit(byte port, byte angle) {
    servoIsActiveGlobal = true;
    servos180AngleGlobal[port] = angle;

    servo180Move(port, angle);
}


void servoLoop_descartado() {    
    // Control Servos sin libreria Servo.h
    //TODO: Solo recorrer matriz de pines validos
    
    for (int i=0; i<8; i++){
        if (servos180AngleGlobal[i] > -1){
            servo180Move(i, servos180AngleGlobal[i]);
        }
    }

    /*
    // Control servos 360, ruedas.
    if (servos360Vel != 0 && servos360Steps > 0){
      if (servos360Loops > servos360Steps){
          //Serial.println("Para");
          servoI.write(90);
          servoD.write(90);
          servos360Steps = 0;
          servos360Loops = 0;
      }else{
        servos360Loops++;
        //Serial.println(servos360Loops);
      }
    } 
    */   
}


void servo180Move(byte port, byte angle) {
    byte pin = pinOfPortGlobal[port];
    
    //servos[pin].attach(pin, 480, 2100); // MG90S
    //servos[pin].attach(pin, 550, 2100); // SG90
    servos[pin].attach(pin);
    servos[pin].write(angle);
}


/* Test servo
void servoTest(){
    if (numLoops100Global < 10){
        servo180Move(1, 0);
        servo180Move(2, 0);
        servo180Move(3, 0);
        servo180Move(4, 0);
        servo180Move(5, 0);
        servo180Move(6, 0);
        servo180Move(7, 0);
        servo180Move(8, 0);
            Serial.println("0");

    } else if (numLoops100Global > 10 && numLoops100Global < 20){
        servo180Move(1, 90);
        servo180Move(2, 90);
        servo180Move(3, 90);
        servo180Move(4, 90);
        servo180Move(5, 90);
        servo180Move(6, 90);
        servo180Move(7, 90);
        servo180Move(8, 90);
            Serial.println("90");
        
    } else if (numLoops100Global > 20){
        servo180Move(1, 180);
        servo180Move(2, 180);
        servo180Move(3, 180);
        servo180Move(4, 180);
        servo180Move(5, 180);
        servo180Move(6, 180);
        servo180Move(7, 180);
        servo180Move(8, 180);
            Serial.println("180");
        
    }
    delay(1000);
}
*/


// DESCARTADO por el momento
// Al utizar los servo se inuiliza el PWM pero
// si simulamos el PWM de los motores en el Loop, no debería haber comflicos.


// Para no tener que utilizar la librería time, llamamos a la funcion de servo
// un numero de loop para que le de tiempo al servo a ponerse en la posición adecuada,
// sin tampoco bloquear los timer que requiere la librería Servo que nos haría perder
// así el uso del PWM para el puent H.
// El SG90 tiene un ciclo de 2500 microsg. y un SG90S de 2000 microsg.

// TODO: Habría que montar una matriz si queremos utilizar varios servos a la vez

// En microsegundos, 1000 microsegundos = 1 milisegundo (ms).
//#define SERVO_SG90_CICLE 20000
//#define SERVO_SG90_BEGIN_DUTY_CICLE 505 
//#define SERVO_SG90_END_DUTY_CICLE 2170  

#define SERVO_MG90S_CICLE           20000
#define SERVO_MG90S_MIN_DUTY_CICLE    450 //  400
#define SERVO_MG90S_MAX_DUTY_CICLE   2080 // 1800 

void servo180Move_Descartado(int port, int angle) {
    servos180AngleGlobal[port] = angle;

    int pin = pinOfPortGlobal[port];
    float pause;
    pinMode(pin, OUTPUT);
    
    // Calculamos el ancho del pulso aplicando la regla de tres
    pause = map(angle, 0, 180, SERVO_MG90S_MIN_DUTY_CICLE, SERVO_MG90S_MAX_DUTY_CICLE);

    Serial.print("ángulo: ");
    Serial.println(angle);
    Serial.print("pausa: ");
    Serial.println(pause);
    Serial.print("pin: ");
    Serial.println(pin);
    Serial.println("-------------------------------");


    // Minimo 50 Hz (50 veces) para activar
    for (int hz = 0; hz < 50; hz++) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(pause);
        digitalWrite(pin, LOW);
        delayMicroseconds(SERVO_MG90S_CICLE - pause);      // Completamos el ciclo de y empezamos uno nuevo para crear asi el tren de pulsos
    }
}









/****************************************************************************************
    GKGyro
*/ 
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
bool    gyroIsActiveGlobal = false;
float   gyroXGlobal;
float   gyroYGlobal;
float   gyroZGlobal;
long    gyroLastTimeGlobal;

void gyroStop() {
    gyroIsActiveGlobal = false;
}

void gyroInit(){
    // Inicializa la primera vez
    Wire.begin(); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    gyroIsActiveGlobal = true;
    gyroXGlobal = 0;
    gyroYGlobal = 0;
    gyroZGlobal = 0;

    tone(8, 300, 30);
    tone(8, 100, 30);
    tone(8, 300, 30);
}

void gyroLoop(){
    //MPU-6050 da los valores en enteros de 16 bits
    //Valores RAW
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
     
    //Angulos
    float Acc[2];
    float Gy[3];
    
    float dt;

    
    //Leer los valores del Acelerometro de la IMU
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
    AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    
    //A partir de los valores del acelerometro, se calculan los angulos Y, X
    //respectivamente, con la formula de la tangente.
    Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
    Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
    
    //Leer los valores del Giroscopio
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
    GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();
    
    //Calculo del angulo del Giroscopio
    Gy[0] = GyX/G_R;
    Gy[1] = GyY/G_R;
    Gy[2] = GyZ/G_R;
    
    dt = (millis() - gyroLastTimeGlobal) / 1000.0;
    gyroLastTimeGlobal = millis();
    
    //Aplicar el Filtro Complementario
    gyroXGlobal = 0.98 *(gyroXGlobal+Gy[0]*dt) + 0.02*Acc[0];
    gyroYGlobal = 0.98 *(gyroYGlobal+Gy[1]*dt) + 0.02*Acc[1];
    
    //Integración respecto del tiempo paras calcular el YAW
    gyroZGlobal = gyroZGlobal+Gy[2]*dt;

    //buzzer.tone(8, 300, 5);

    //Mostrar los valores por consola
    //Serial.println("90, " +String(gyroXGlobal) + "," + String(gyroYGlobal) + "," + String(gyroZGlobal) + ", -90");
    
    //delay(10);
}



/*******************************************************************************************************
   GkColor
*/

bool colorIsActiveGlobal = false; 
byte colorRedGlobal = 0; 
byte colorGreenGlobal = 0;
byte colorBlueGlobal = 0;
unsigned long colorBeginTimeGlobal = 0;
void colorLoop() {    
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);

    if (colorBeginTimeGlobal == 0 || (millisGlobal - colorBeginTimeGlobal) > 10){
        colorBeginTimeGlobal = millisGlobal;
    }
    Serial.println(colorBeginTimeGlobal);
    if ((millisGlobal - colorBeginTimeGlobal) < colorRedGlobal/10) {
        digitalWrite(11, HIGH);
    } else {
        digitalWrite(11, LOW);
    }
    if ((millisGlobal - colorBeginTimeGlobal) < colorGreenGlobal/10) {
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }
    if ((millisGlobal - colorBeginTimeGlobal) < colorBlueGlobal/10) {
        digitalWrite(12, HIGH);
    } else {
        digitalWrite(12, LOW);
    }
}



/*******************************************************************************************************
    GkEngineMotor

    engineType = 0 >> Desactivado
    engineType = 10 >> GkEngineM motores DC por Steeps
    engineType = 20 >> GkEngineM motores DC por velocity
    engineType = 30 >> GkEngineS servos
*/
const float ENGINE_VELOCITY_FACTOR = 0.2;
const float ENGINE_STEPS_FACTOR = 70;

byte engineTypeGlobal = 0;

// Potencia de 0-100 que se le da al pin para que ande el motor
bool engineMPin5Global = 0;
bool engineMPin6Global = 0;
bool engineMPin9Global = 0;
bool engineMPin10Global = 0;
bool engineMSpeedGlobal = 0;

// Numero de pasos que tiene que dar el motor, calculo aprox. en base a numero de loops
unsigned long engineTargetTimeGlobal = 0;
unsigned long engineBeginTimeGlobal = 0; // Tiempo en milis desde que arranca el motor
float engineNextTimeGlobal = 0; // Tiempo entre paradas para Velocity

byte engineLastDirGlobal = 255; // Tiempo entre paradas para Velocity
byte enginePowerGlobal = 0; // Se va incrementando en cada ciclo para mejorar agarre


void engineMLoop() { 

    // Incrementa potencia para agarre
    if (enginePowerGlobal < 255){
        enginePowerGlobal += 1;
    }

    // Control por Speed
    if (engineTypeGlobal == 10) {    
        if (millisGlobal < engineTargetTimeGlobal){
            analogWrite(5, engineMPin5Global * enginePowerGlobal);
            analogWrite(6, engineMPin6Global * enginePowerGlobal);
            analogWrite(9, engineMPin9Global * enginePowerGlobal);
            analogWrite(10, engineMPin10Global * enginePowerGlobal);
            
        }else{
            digitalWrite(5, 0);
            digitalWrite(6, 0);
            digitalWrite(9, 0);
            digitalWrite(10, 0);
            delay(2);
            // Calcula la siguiente parada    
            engineTargetTimeGlobal = millisGlobal + engineNextTimeGlobal;

        }
        
    // Control por Steeps -------------------------------  
    }else if(engineTypeGlobal == 20){
        if (millisGlobal < engineTargetTimeGlobal){
            analogWrite(5, engineMPin5Global * enginePowerGlobal);
            analogWrite(6, engineMPin6Global * enginePowerGlobal);
            analogWrite(9, engineMPin9Global * enginePowerGlobal);
            analogWrite(10, engineMPin10Global * enginePowerGlobal);

            /*/ El primer segundo va despacio para no patinar
            if (millisGlobal < (engineBeginTimeGlobal + 100)){
                // Para hacerlo exponencial
                //if (20 - abs(millisGlobal - engineBeginTimeGlobal) > 0){
                //    delay(20 - abs(millisGlobal - engineBeginTimeGlobal));
                //}
                delay(10);
                digitalWrite(5, 0);
                digitalWrite(6, 0);
                digitalWrite(9, 0);
                digitalWrite(10, 0);
            }
            */
            
        }else{
            digitalWrite(5, LOW);
            digitalWrite(6, LOW);
            digitalWrite(9, LOW);
            digitalWrite(10, LOW);
            engineMPin5Global = 0;
            engineMPin6Global = 0;
            engineMPin9Global = 0;
            engineMPin10Global = 0;            
            engineTypeGlobal = 0;
        }
        
    }else{
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        digitalWrite(9, LOW);
        digitalWrite(10, LOW);
        engineMPin5Global = 0;
        engineMPin6Global = 0;
        engineMPin9Global = 0;
        engineMPin10Global = 0;
        
    }

}


void engineMInit(){
    // Parar motores???
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);

    engineTypeGlobal = 0;
    engineMPin5Global = 0;
    engineMPin6Global = 0;
    engineMPin9Global = 0;
    engineMPin10Global = 0;
    engineMSpeedGlobal = 0;
    engineTargetTimeGlobal = 0;
    engineBeginTimeGlobal = 0; 
    engineNextTimeGlobal = 0; 
    
}


void engineM(int motorDir, float motorSpeed, int motorSteps) {

    // Inicializa a 0 para agarre
    enginePowerGlobal = 0;

    // Control por Steps
    engineBeginTimeGlobal = millis();
    if (motorSpeed > 0) {
        engineTypeGlobal = 10;
        engineNextTimeGlobal = (motorSpeed * ENGINE_VELOCITY_FACTOR);
        engineTargetTimeGlobal = engineBeginTimeGlobal + engineNextTimeGlobal;
        
    } else {
        engineTypeGlobal = 20;
        engineTargetTimeGlobal = engineBeginTimeGlobal + (motorSteps * ENGINE_STEPS_FACTOR);
        motorSpeed = 100;
        
    }
    
    // Motor Izquierda
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    // Motor Derecha
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    
    // Para que no queden a mita de ciclo y da como un sobresalto de velocidad
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);

    // Cuando se manda varias veces para adelante/atras al final el arduino se resetea. 
    // Debe crearse una corriente q sobrecarga el sistema. Con esto se soluciona.
    if (
        (engineLastDirGlobal == 1 && motorDir == 4) ||        
        (engineLastDirGlobal == 4 && motorDir == 1) ||
        (engineLastDirGlobal == 2 && motorDir == 3) ||
        (engineLastDirGlobal == 3 && motorDir == 2)
    ){
        delay(200);
    }
    engineLastDirGlobal = motorDir;
    
        
    // Avanzar
    if (motorDir == 1) {
        engineMPin5Global = 1;
        engineMPin6Global = 0;
        engineMPin9Global = 0;
        engineMPin10Global = 1;
    
    // A la Derecha
    } else if (motorDir == 2) {
        engineMPin5Global = 0;
        engineMPin6Global = 0;
        engineMPin9Global = 0;
        engineMPin10Global = 1;
    
    // A la Izquierda
    } else if (motorDir == 3) {
        engineMPin5Global = 1;
        engineMPin6Global = 0;
        engineMPin9Global = 0;
        engineMPin10Global = 0;

    // Atras
    } else if (motorDir == 4) {
        engineMPin5Global = 0;
        engineMPin6Global = 1;
        engineMPin9Global = 1;
        engineMPin10Global = 0;
        
    // Para
    } else if (motorDir == 0) {
        engineMPin5Global = 0;
        engineMPin6Global = 0;
        engineMPin9Global = 0;
        engineMPin10Global = 0;
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        digitalWrite(9, LOW);
        digitalWrite(10, LOW);
    
    // Error
    } else {
        tone(8, 100, 400);
    
    }
}


/*
  // DESCARTADO
  // Control de ruedas con encoders
  // Derecha
  void runMotorStopWhell1(){
    /* Para utilizar los encoders
    if (engineSteepsGlobal >= 0){
        pinMode(2, INPUT);
        pinMode(3, INPUT);
        attachInterrupt(digitalPinToInterrupt(2), runMotorStopWhell1, CHANGE); // Interrupt 0 en nano es el pin 2  CHANGE
        attachInterrupt(digitalPinToInterrupt(3), runMotorStopWhell2, CHANGE); // Interrupt 1 en nano es el pin 3
    }
    //motorNow5 = engineMPwmPin5Global;
    //motorNow6 = engineMPin6Global;
    //motorNow9 = engineMPin9Global;
    //motorNow10 = engineMPin10Global;
    analogWrite(5, engineMPin5Global);
    analogWrite(6, engineMPin6Global);
    analogWrite(9, engineMPin9Global);
    analogWrite(10, engineMPin10Global);

    if (motorSteps > 0){
        delay(motorSteps * 100);
        analogWrite(5, 0);
        analogWrite(6, 0);
        analogWrite(9, 0);
        analogWrite(10, 0);
    }

    motorRoundsWhell1++;

    if (motorRoundsWhell1 > motorRoundsWhellTarget){
        digitalWrite(9, LOW);
        digitalWrite(10, LOW);

        detachInterrupt(digitalPinToInterrupt(2));

    } else {
        if ((motorRoundsWhell1 - motorRoundsWhell2) > 1){
            motorNow9 -= 2;
            motorNow10 -= 2;
        }else{
            if(motorNow9 < engineMPin9Global){
                motorNow9 += 2;
                motorNow10 += 2;
            }
        }
        if(engineMPin9Global > 0){
          analogWrite(9, motorNow9);
        }
        if(engineMPin10Global > 0){
          analogWrite(10, motorNow10);
        }
    }
  }
*/



/*
  // DESCARTADO
  // Control de ruedas con encoders
  // Izquierda
  void runMotorStopWhell2(){
    motorRoundsWhell2++;

   if (motorRoundsWhell2 > motorRoundsWhellTarget){
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);

        detachInterrupt(digitalPinToInterrupt(3));

    } else {
        if ((motorRoundsWhell2 - motorRoundsWhell1) > 1){
            motorNow5 -= 2;
            motorNow6 -= 2;
            //motorMedia9_10
        }else{
            if(motorNow5 < engineMPin5Global){
                motorNow5 += 2;
                motorNow6 += 2;
            }
            //motorNow5  = engineMPin5Global;
            //motorNow6 = engineMPin6Global;
        }
        if(engineMPin5Global > 0){
            analogWrite(5, motorNow5);
        }
        if(engineMPin6Global > 0){
            analogWrite(6, motorNow6);
        }
    }
  }
*/


/* DESCARTADO
// Para no tener que utilizar la librería time, llamamos al la funcion de servo
// un numero de loop para que le de tiempo al servo a ponerse en la posición adecuada,
// sin tampoco bloquear los timer que requiere la librería Servo que nos haría perder
// así el uso del PWM para el puent H.
// El FS90R tiene un ciclo de 2500 microsg.

// TODO: Habría que montar una matriz si queremos utilizar varios servos a la vez

// En microsegundos, 1000 microsegundos = 1 milisegundo (ms).
#define SERVO_FS90R_CICLE 20000
#define SERVO_FS90R_BEGIN_DUTY_CICLE 1000  //544
#define SERVO_FS90R_MID_DUTY_CICLE 1450   //2400
#define SERVO_FS90R_END_DUTY_CICLE 2000   //2400

void servo360MoveOPTIMIZADO__() {
  // Calculamos el ancho del pulso aplicando la regla de tres
  int pauseDer = map(servos360Vel, 0, 10, SERVO_FS90R_BEGIN_DUTY_CICLE, SERVO_FS90R_MID_DUTY_CICLE);
  int pauseIzq = SERVO_FS90R_BEGIN_DUTY_CICLE - pauseDer + SERVO_FS90R_BEGIN_DUTY_CICLE;

  Serial.print("Vel: ");
  Serial.print(servos360Vel);
  Serial.print("  pausa Der: ");
  Serial.print(pauseDer);
  Serial.print("  pausa Iz: ");
  Serial.println(pauseIzq);

  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(6, HIGH);
  delayMicroseconds(pauseDer);
  digitalWrite(6, LOW);
  digitalWrite(10, HIGH);
  delayMicroseconds(pauseIzq);
  digitalWrite(10, LOW);
  // TODO: seguramente se podría eliminar esta pausa por que el propio loop debe tardar mas de 20 ms.
  delayMicroseconds(SERVO_FS90R_CICLE - pauseDer);      // Completamos el ciclo de y empezamos uno nuevo para crear asi el tren de pulsos
}
*/


/*********************************************************************************************
   GkEngine Servo
   Con motores servo 360 FS90R
*/
void runGkEngineS(int motorDir, int motorSpeed, int motorSteps) {
    engineTypeGlobal = 2;
    
    int angle = map(motorSpeed, 0, 10, 90, 180);
    
    servos[6].attach(6);
    servos[10].attach(10);
    
    // Avanzar
    if (motorDir == 1) {
        servos[10].write(angle);
        servos[6].write(180 - angle);
        
    // A la Derecha
    } else if (motorDir == 2) {
        servos[10].write(angle);
        servos[6].write(90);
    
    // A la Izquierda
    } else if (motorDir == 3) {
        servos[10].write(90);
        servos[6].write(180 - angle);
    
    // Atras
    } else if (motorDir == 4) {
        servos[10].write(180 - angle);
        servos[6].write(angle);
    
    // Para
    } else if (motorDir == 0) {
        servos[10].write(90);
        servos[6].write(90);
    
    // Error
    } else {
        blinkLed(3);
    
    }
    
    if (motorSteps > 0) {
        delay(motorSteps * 100);
        servos[10].write(90);
        servos[6].write(90);
    }

}

/************************************************************************************************************************************************************
    GkSonic
*/ 

byte sonicRead(byte pin) {
    //buzzer.tone(8, 100, 5);
    
    byte distance = -10;
    byte lastDistance = -10;
    byte i = 0;
    
    // Si entre dos tomas hay una diferencia de mas de 5 cm vuelve a medir
    //while ((abs(lastDistance - distance) < 5) && (i < 10)) {
        pinMode(pin, OUTPUT); // Trigger
        digitalWrite(pin, LOW);
        delayMicroseconds(4);
        digitalWrite(pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pin, LOW);
    
        pinMode(pin, INPUT); // echo
        //distance = pulseIn(echo, HIGH, 30000) / 58.0;
        distance = pulseIn(pin, HIGH) / 58.0;

        // Algunos ultrasonidos cuando la distancia es mayor del rango devuelve 0.
        // Ponemos como distancia máxima de medida 200 cm
        if (distance == 0 || distance > 200) {
            // Volvemos a medir
            pinMode(pin, OUTPUT); // Trigger
            digitalWrite(pin, LOW);
            delayMicroseconds(4);
            digitalWrite(pin, HIGH);
            delayMicroseconds(10);
            digitalWrite(pin, LOW);
        
            pinMode(pin, INPUT);
            //distance = pulseIn(echo, HIGH, 30000) / 58.0;
            distance = pulseIn(pin, HIGH) / 58.0;
        }
        //buzzer.tone(8, (distance * 10), 1);

        if (distance > 200){
            distance = 200;
        }

        //lastDistance = distance;
        //i++;
    //}

    return distance;
}

/*
 * Buzzer
 */


/********************************************************************
   BlickLed
*/

void blinkLed(int num) {
  if (num == 0) {
    tone(8, 300, 200);
  }

  for (int i = 0; i < num; i++) {
    digitalWrite(13, HIGH);
    tone(8, 250, 200);
    digitalWrite(13, LOW);
    delay(200);
  }
  delay(500);
}




//************************************************************************************************************************************************************

unsigned char readBuffer(int index) {
  return isBluetooth ? bufferBt[index] : buffer[index];
}


void writeBuffer(int index, unsigned char c) {
  if (isBluetooth) {
    bufferBt[index] = c;
  } else {
    buffer[index] = c;
  }
}

void writeSerial(unsigned char c) {
  Serial.write(c);
}

void writeHead() {
  writeSerial(0xff);
  writeSerial(0x55);
}

void writeEnd() {
  Serial.println();
}

void readSerial() {
  isAvailable = false;
  if (Serial.available() > 0) {
    isAvailable = true;
    isBluetooth = false;
    serialRead = Serial.read();
  }
}

void callOK() {
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}

void sendByte(char c) {
  writeSerial(1);
  writeSerial(c);
}

void sendString(String s) {
  int l = s.length();
  writeSerial(4);
  writeSerial(l);
  for (int i = 0; i < l; i++) {
    writeSerial(s.charAt(i));
  }
}

void sendFloat(float value) {
  writeSerial(0x2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

void sendShort(double value) {
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}

void sendDouble(double value) {
  writeSerial(2);
  valDouble.doubleVal = value;
  writeSerial(valDouble.byteVal[0]);
  writeSerial(valDouble.byteVal[1]);
  writeSerial(valDouble.byteVal[2]);
  writeSerial(valDouble.byteVal[3]);
}

short readShort(int idx) {
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx + 1);
  return valShort.shortVal;
}

float readFloat(int idx) {
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.floatVal;
}

long readLong(int idx) {
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.longVal;
}


/*
  ff 55 len idx action device port  slot  data a
  0  1  2   3   4      5      6     7     8
*/
void parseData() {
    isStart = false;
    int idx = readBuffer(3);
    command_index = (uint8_t)idx;
    int action = readBuffer(4);
    int device = readBuffer(5);
    
    if (action == GET) {
        //if (device != ULTRASONIC_SENSOR) {
          writeHead();
          writeSerial(idx);
        //}
        readSensor(device);
        writeEnd();

    
    } else if(action == RUN) {
        runModule(device);
        callOK();


    } else if(action == RESET) {
        tone(8, 50, 300);
        gyroStop();
        engineMInit();
        servoStop();
        callOK();


    } else if(action == START) {
        tone(8, 400, 300);
        gyroInit();
        servoStop();
        engineMInit();
        callOK();
    }
}


void readSensor(int device) {
    /**************************************************
      ff 55 len idx action device port slot data a
      0  1  2   3   4      5      6    7    8
    ***************************************************/
    float value = 0.0;
    //int slot;
    //Puertos/Pines Digitales  {0,    3, 4, 7, 6,    10, 11, 12, 13};
    //              Analógicos A {0,   A2, A6, A4, A0};
    //                         B {0,   A3, A7, A5, A1};
    
    int port = readBuffer(6);
    int pinDigital = pinOfPortGlobal[port];

    /* Debug puerto o Pin
    for (int i=1; i<=pinDigital; i++){
        buzzer.tone(8, 150, 200);
        buzzer.tone(8, 300, 200);
    }
    buzzer.tone(8, 500, 500);
    */
    // Poner el orden de los IF de mas utilizado a menos utilizado    
    if (device == DIGITAL) {
        pinMode(pinDigital, INPUT);
        bool state = digitalRead(pinDigital);
        if (state == HIGH){
            sendByte(true);
        }else{
            sendByte(false);            
        }
        
    }else if (device == ANALOG) {
        int pinAnalog = pinAnalogAOfPort[port];
        pinMode(pinAnalog, INPUT);
        sendShort(map(analogRead(pinAnalog), 0, 1023, 0, 100));

        
    }else if (device == ULTRASONIC_ARDUINO) {
        sendByte(sonicRead(pinDigital));


    }else if (device == GYRO){
        // Utiliza el puerto 3 q es donde está el I2C en el GK Tero
        if (!gyroIsActiveGlobal){
            gyroInit();
        }

        // X = 0, Y = 1, Z = 2 
        int axis = readBuffer(7);
        // TODO: Optimizar enviando un short en lugar de un float
        // Estan cambiados el X original del sensor por el Y.
        if (axis == 0){
            sendFloat(round(gyroXGlobal));           
        }else if (axis == 1){
            sendFloat(round(gyroYGlobal)*-1);           
        }else if (axis == 2){
            sendFloat((round(gyroZGlobal)*-1)+90);           
        }    

    }else if (device == PULSEIN) {
        int pw = readShort(7);
        pinMode(pinDigital, INPUT);
        sendShort(pulseIn(pinDigital, HIGH, pw));

    }else if (device == TIMER) {
        sendFloat((float)currentTime);
       
    }else if (device == VERSION) {
        tone(8, 300, 200);
        sendString(versionGlobal);
        
    }
}


//************************************************************************************************************************************************************
void runModule(int device) {
    /**************************************************
      ff 55 len idx action device port slot data a
      0  1  2   3   4      5      6    7    8
    ***************************************************/
    //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
    
    
    //Puertos/Pines {0,    3, 4, 7, 6,    10, 11, 12, 13};
    int port = readBuffer(6);
    int pin = pinOfPortGlobal[port];
//  if (device != TONE) {
//    buzzer.tone(8, 150, 5);
//  }

    if (device == MOTOR) {
        int motorDir = readShort(7);
        int motorSpeed = readShort(9);
        int motorSteps = readShort(11);
        engineM(motorDir, motorSpeed, motorSteps);

    
    } else if(device == ENGINE_S) {
        int motorDir = readShort(7);
        int motorSpeed = readShort(9);
        int motorSteps = readShort(11);
        runGkEngineS(motorDir, motorSpeed, motorSteps);

    
    } else if(device == DIGITAL) {
        pinMode(pin, OUTPUT);
        int v = readBuffer(7);

        if (v == 1) {
            digitalWrite(pin, HIGH);
        } else {
            digitalWrite(pin, LOW);
        }

        
    } else if(device == PWM) {
        pinMode(pin, OUTPUT);
        int v = readBuffer(7);
        analogWrite(pin, v);

    
    } else if(device == TONE) {
        // Buzzer interno
        if (pin == 0) {
            pin = 8;
        }
        pinMode(pin, OUTPUT);
        int hz = readShort(7);
        int ms = readShort(9);
        if (ms > 0) {
            tone(pin, hz, ms);
        } else {
            noTone(pin);
        }

    
    } else if(device == SERVO_PIN) {
        int angulo = readBuffer(7);
        servoInit(port, angulo);


    } else if(device == TIMER) {
        lastTime = millis() / 1000.0;

    
    } else if(device == RGBLED) {
        colorRedGlobal = readShort(7);
        colorGreenGlobal = readShort(9);
        colorBlueGlobal = readShort(11);
        colorIsActiveGlobal = true;
        colorLoop();
    
    } 
}


/****************************************************************************************************************
 * Main 
 */

void setup() {
    Serial.begin(115200);
    Serial.print("Version: ");
    Serial.println(versionGlobal);

    // Inicializaciones
    engineMInit();  


    // Bienvenida
    tone(8, 250, 300);
    delay(350);
    tone(8, 250, 300);
    delay(800);
    
    tone(8, 250, 300);
    delay(300);
    tone(8, 300, 300);
    delay(300);
    
    tone(8, 250, 300);
    delay(350);
    tone(8, 250, 300);


    // Pruebas

/*    
    engineM(1, 0, 2);
    colorRedGlobal = 0;
    colorGreenGlobal = 0;
    colorBlueGlobal = 100;
    colorIsActiveGlobal = true;
    colorLoop();
*/    
}


void loop() {
    // Lectura de datos ---------------------------------------------------------
    currentTime = millis() / 1000.0 - lastTime;
    readSerial();

    if (isAvailable) {
        unsigned char c = serialRead & 0xff;
        // Empieza a leer un nuevo paquete de datos
        if (c == 0x55 && isStart == false) {
            if (prevc == 0xff) {
                index = 1;
                isStart = true;
            }
        } else {
            prevc = c;
            if (isStart) {
                if (index == 2) {
                    dataLen = c;
                } else if (index > 2) {
                    dataLen--;
                }
                writeBuffer(index, c);
            }
        }
        index++;
        if (index > 51) {
            index = 0;
            isStart = false;
        }
        if (isStart && dataLen == 0 && index > 3) {
            isStart = false;
            parseData();
            index = 0;
        }
    }



    // Controles de bloques asincronos ---------------------------------------------------------
    millisGlobal = millis();
    
    if (colorIsActiveGlobal){
        colorLoop();
    }

    /*
    if (servoIsActiveGlobal) {
        servoLoop();
    }
    */

    if (gyroIsActiveGlobal){
        if ((millisGlobal - gyroLastTimeGlobal) > 10) {
            gyroLoop();
        }
    }

    if (engineTypeGlobal != 0){
        engineMLoop();
    }

}
