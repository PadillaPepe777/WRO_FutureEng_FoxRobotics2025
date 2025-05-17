#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>

VL53L0X sensor1; //derecha
VL53L0X sensor2; //delante
VL53L0X sensor3;//izquierda


#define XSHUT1  19  // Pin para apagar sensor 1
#define XSHUT2  5  // Pin para apagar sensor 2
#define XSHUT3  17 // Pin para apagar sensor 3

float DERECHA, ENFRENTE, IZQUIERDA;

int dif,pos;
//Para Control PD
unsigned long currentTime, previousTime;
float elapsedTime;
double error, lastError, cumError, rateError;
//Distancia deseada
float Setpoint=0;
//Variables a Cambiar
float kp=5.0, kd=0 ;

//IMU
int16_t Gyr_rawZ; //Read first Raw Data
float EgZ; //Data different from 0, avoids nan
float GradoRaw;
float AnguloZ=0.0;

unsigned long time1, timePrev, despliegue; //millis for integrals
//despliegue for showing angle every given time
float elapsedTime2;
int i;
float rad_to_deg = 180/3.141592654;

float desired_angle = 0;

//Servo
int servoPin = 18;
Servo servo;


//Motor
const int STBY=13;
const int PWM=27;
const int IN1=12;
const int IN2=26;
const int Vel=120;


void setup() {


    Serial.begin(115200);
    Wire.begin(); //  SDA y SCL 40kHz
    delay(100);

    // Configurar pines XSHUT como salida
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    pinMode(XSHUT3, OUTPUT);

    // Apagar ambos sensores
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    digitalWrite(XSHUT3, LOW);
    delay(10);

    // Encender y configurar el primer sensor
    digitalWrite(XSHUT1, HIGH);
    delay(10);
    sensor1.setAddress(0x30);  // Cambiar dirección del sensor 1
    if (!sensor1.init()) {
        Serial.println("❌ Error al iniciar Sensor 1");
    } else {
        Serial.println("✅ Sensor 1 iniciado correctamente");
    }

    // Encender y configurar el segundo sensor
    digitalWrite(XSHUT2, HIGH);
    delay(10);
    sensor2.setAddress(0x31);  // Cambiar dirección del sensor 2
    if (!sensor2.init()) {
        Serial.println("❌ Error al iniciar Sensor 2");
    } else {
        Serial.println("✅ Sensor 2 iniciado correctamente");
    }

    digitalWrite(XSHUT3, HIGH);
    delay(10);
    sensor3.setAddress(0x32);  // Cambiar dirección del sensor 2
    if (!sensor3.init()) {
        Serial.println("❌ Error al iniciar Sensor 3");
    } else {
        Serial.println("✅ Sensor 3 iniciado correctamente");
    }

    // Iniciar medición en ambos sensores
    sensor1.startContinuous();
    sensor2.startContinuous();
    sensor3.startContinuous();

delay(200);

EgZ=0.0;

 Wire.beginTransmission(0x68);
    if (Wire.endTransmission() == 0) {  
        Wire.beginTransmission(0x68);
        Wire.write(0x6B);
        Wire.write(0);  // Sacar de modo sleep
        Wire.endTransmission(true);
        Serial.println("✅ MPU6050 iniciado correctamente");
    } else {
        Serial.println("❌ Error: MPU6050 no detectado.");
    }

    for(i=0;i<600;i++)
    {
    anguloIMU();
    EgZ+=Gyr_rawZ;
    delay(1);
    }
    
    EgZ=EgZ/600.0;

    Serial.print("Offsets: ");
    Serial.println(EgZ);

    delay(100);

    timePrev=millis();
    anguloIMUYa();
    Serial.print("Valores al momento: ");
   
        Serial.println(Gyr_rawZ);
        delay(1000);

  servo.setPeriodHertz(50);             // 50 Hz para servo
  servo.attach(servoPin, 500, 2400);    // valores típicos: 500us (0°), 2400us (180°)
  Serial.println("Servo listo");

    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(STBY,OUTPUT);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(STBY, HIGH);
    //analogWrite(PWM, Vel);
    //Esta dando problemas este pin, hay que ver que show


}


void loop() {

    DERECHA=sensor1.readRangeContinuousMillimeters()/10.0;
    delay(5);
    ENFRENTE=sensor2.readRangeContinuousMillimeters()/10.0;
    delay(5);
    IZQUIERDA=sensor3.readRangeContinuousMillimeters()/10.0;
    delay(5);

    Serial.print("Distancia Sensor 1: ");
    Serial.print(DERECHA);

    Serial.print("   Distancia Sensor 2: ");
    Serial.print(ENFRENTE);

    Serial.print("   Distancia Sensor 3: ");
    Serial.println(IZQUIERDA);

//Para Hacer lo del angulo cuando se cumpla la condicion de pared enfrente, libre al lateral
//Seguir hasta haber una diferencia de 80 grados min entre inicial y actual

    if(ENFRENTE<=30 && IZQUIERDA>=60)
    {
    pos=ENFRENTE*2;
    servo.write(pos);
    Serial.print("Servo a: ");
    Serial.println (pos);
      delay(1);

      timePrev=millis();
      
      while(abs(AnguloZ)<83.5)
      {
        anguloIMUYa();
        Serial.println(AnguloZ);
      }

      
      Serial.println("Ya salio");
      
    Serial.println("Servo a: 90");

      servo.write(90);
      delay(1000);
    }
    else
    {
    dif= (DERECHA-IZQUIERDA);
    pos=PID_Centro (dif);
    if(abs(pos)>5)
    {
    pos=90+pos;
    }
    else
    {
        pos=90;
    }

    servo.write(pos);
    Serial.print("Servo a: ");
    Serial.println (pos);
    }

    delay(100);
}

int PID_Centro (int input) {
 
    currentTime = millis();                          // obtener el tiempo actual
    elapsedTime = (currentTime - previousTime);     // calcular el tiempo transcurrido
        
    error = input;                               // determinar el error entre la consigna y la medición
    rateError = (error - lastError) / elapsedTime;       // calcular la derivada del error
 
    int output = kp*error + kd*rateError;     // calcular la salida del PID

    if(output>70)
    {
      output=70;
    }
    if(output<-70)
    {
      output=-70;
    }

    lastError = error;                                    // almacenar error anterior
    previousTime = currentTime;                           // almacenar el tiempo anterior
 
    return output;
}

void anguloIMUYa () {
   // the previous time is stored before the actual time read
    time1 = millis();  // actual time read
    elapsedTime2 = (time1 - timePrev) / 1000.0; 
    
   Wire.beginTransmission(0x68);
   Wire.write(0x47); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,2,true); //Just  registers
   
   Gyr_rawZ=Wire.read()<<8|Wire.read();

   GradoRaw=(Gyr_rawZ-EgZ)/131.0;

   if(abs(GradoRaw*elapsedTime2)>=0.06)
   AnguloZ+= GradoRaw*elapsedTime2;

    timePrev = time1;
}


void anguloIMU () {
   // the previous time is stored before the actual time read
    time1 = millis();  // actual time read
    elapsedTime2 = (time1 - timePrev) / 1000.0; 
    
   Wire.beginTransmission(0x68);
   Wire.write(0x47); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,2,true); //Just  registers
   
   Gyr_rawZ=Wire.read()<<8|Wire.read();

  timePrev = time1;
}