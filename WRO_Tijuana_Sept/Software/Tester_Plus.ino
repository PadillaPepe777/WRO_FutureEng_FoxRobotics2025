#include <Arduino.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


VL53L0X sensor1; //derecha
VL53L0X sensor2; //delante
VL53L0X sensor3;//izquierda

#define XSHUT1  17  // Pin para apagar sensor 1
#define XSHUT2  5  // Pin para apagar sensor 2
#define XSHUT3  19 // Pin para apagar sensor 3

#define MOTOR_PWM_CHANNEL 2 // canal diferente al del servo
#define MOTOR_PWM_FREQ 12000
#define MOTOR_PWM_RESOLUTION 8

//Comunicacion Serial Colores
#define TX_PIN2 14  // Pin TX personalizado
#define RX_PIN2 15  // No se usa en el transmisor

#define Boton 36

bool Ready=false;
bool Ready2=false;

#define Trigger 33
#define Echo 25

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 32 

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


int Numeros=0, Sector=0, Color=0;

int contador=1;

String NOMBRE_COLORES[4] = {"N","R", "V", "M"};

//IMU
int16_t Gyr_rawZ; //Read first Raw Data
float EgZ; //Data different from 0, avoids nan
float GradoRaw;
float AnguloVuelta=75.0;
float AnguloZ=0.0;

//Evitar estrellarse
float kp=5.0;
float error;

unsigned long time1, timePrev, despliegue, Tvuelta ; //millis for integrals
//despliegue for showing angle every given time
float elapsedTime2;
float rad_to_deg = 180/3.141592654;

//Servo
int servoPin = 18;
Servo servo;
int pos;

//Motor
const int STBY=13;
const int PWM=26;
const int IN1=12;
const int IN2=27;
int Vel=180;

//Para serial
String mensaje;

//Filtro distancias
float ProD, ProE, ProI, Pro_US;
float a=0.81;
float DERECHA, ENFRENTE, IZQUIERDA, US_ENFRENTE, ENFRENTE_V;
unsigned long UT;
bool PROM=false;

float dif; //entre izq y derecha

//Auxiliares
int i;
float Acumulador;
int ColorAnt, SectorAnt;
float auxColor, auxSector;
bool ModoColor=false;
bool estadoAnterior = true;
int casos=0;

//Limite+multiplicador enfrente angulo servo
float DLim=60, LLim=20.0, MinL=80.0, mult=0.95;

//valores iniciales + contador vueltas
int Esq, Vueltas;
float Inicio;

unsigned long displayTimer;

//hay 1.6 cm de diferencia entre UltraSonico y VL-2

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
        delay(5000);
    } else {
        Serial.println("✅ Sensor 1 iniciado correctamente");
    }

    // Encender y configurar el segundo sensor
    digitalWrite(XSHUT2, HIGH);
    delay(10);
    sensor2.setAddress(0x31);  // Cambiar dirección del sensor 2
    if (!sensor2.init()) {
        Serial.println("❌ Error al iniciar Sensor 2");
        delay(5000);
    } else {
        Serial.println("✅ Sensor 2 iniciado correctamente");
    }

    digitalWrite(XSHUT3, HIGH);
    delay(10);
    sensor3.setAddress(0x32);  // Cambiar dirección del sensor 2
    if (!sensor3.init()) {
        Serial.println("❌ Error al iniciar Sensor 3");
        delay(5000);
    } else {
        Serial.println("✅ Sensor 3 iniciado correctamente");
    }
Serial.println("Todo bien aqui");
    // Iniciar medición en ambos sensores
    sensor1.startContinuous();
    sensor2.startContinuous();
    sensor3.startContinuous();

delay(200);
    // Iniciar la pantalla para su uso
  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

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

  servo.setPeriodHertz(50);             // 50 Hz para servo
  servo.attach(servoPin, 500, 2400);    // valores típicos: 500us (0°), 2400us (180°)
  Serial.println("Servo listo");

  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);

  digitalWrite(Trigger, LOW);

  pinMode(Boton, INPUT);

    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(STBY,OUTPUT);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(STBY, HIGH);

ledcAttachChannel(PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION, MOTOR_PWM_CHANNEL);

ledcAttachChannel(18, 50, 10, 1);


    timePrev=millis();
    anguloIMUYa();
    Serial.print("Valores al momento: ");
   
        Serial.println(Gyr_rawZ);
        delay(1000);

 Serial2.begin(115200, SERIAL_8N1, RX_PIN2, TX_PIN2);
 delay(100);

EgZ=0.0;
AnguloZ=0.0;

  pos=90;
  SetServo();
  delay(80);

despliegue=millis();
}

void loop() {
  bool estadoActual=digitalRead(Boton);

  /*while(Ready==false){
  Ready=digitalRead(Boton);
  delay(40);
  if (Ready==true)
  ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits
  }*/

  if (estadoAnterior == false && estadoActual == true) {
  contador++;
  if (contador >6)
  contador=1;

  delay(20);
  }

  estadoAnterior = estadoActual;

  Distancias();
  anguloIMUYa();


  if(millis()-despliegue>=150)
  {
    ledcWrite(PWM, 0);
    display.clearDisplay();

      switch(contador){
        case 1:
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0,8);
        display.println(ProD);
        display.display();

        break;
        case 2:
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0,0);
        display.println(String(ProE));
        display.setCursor(0,16);
        display.println(String(Pro_US));
        display.display();
       
        break;
        case 3:
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0,8);
        display.println(ProI);
        display.display();
        
        break;
        case 4:
        display.setTextSize(2);
        display.setTextColor(WHITE);
        ReceiveSerialFull();
        if(Sector==0)
        Sector=2;
        else
        Sector=Sector-1;

        display.setCursor((Sector*27),8);
        display.println(NOMBRE_COLORES[Color]);
        display.display();
        
        break;
        case 5:
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0,8);
        display.println(AnguloZ);
        display.display();
        
        break;

        case 6:
        ledcWrite(PWM, Vel);
        pos=pos+10;
        if(pos>160)
        pos=20;
        SetServo();

        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0,8);
        display.println(pos);
        display.display();

        break;

        default:
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(10,8);
        display.println("ERROR");
        display.display();
        break;
      }

      delay(30);



    /*
    if(pos>120)
    pos=30;

    pos=pos+1;
    SetServo();

    Serial.print("Sensor 1: ");
    Serial.print(ProD);

    Serial.print("   Sensor 2: ");
    Serial.print(ProE);
    
    Serial.print("   Sensor UT: ");
    Serial.print(Pro_US);

    Serial.print("   Sensor 3: ");
    Serial.print(ProI);

    Serial.print("  Color: ");
    Serial.print(Numeros);

    Serial.print("   Angulo: ");
    Serial.println(AnguloZ);
    */

    despliegue=millis();

  }

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

  //Serial.print("   Angulo ");
  //Serial.println(AnguloZ);

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

void setLongRangeMode() {
  sensor2.stopContinuous();
  sensor2.setSignalRateLimit(0.1); // menor = más alcance
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor2.startContinuous();
  Serial.println("Modo: Largo alcance activado");
}

void setNormalMode() {
  sensor2.stopContinuous();
  sensor2.setSignalRateLimit(0.25); // valor por defecto
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  sensor2.startContinuous();
  Serial.println("Modo: Normal restaurado");
}

void Distancias(){
  //Implementamos filtros Pasabajos
  //bool VL_Bueno=true;

    DERECHA=(sensor1.readRangeContinuousMillimeters()/10.0)-5.7;
    ENFRENTE=(sensor2.readRangeContinuousMillimeters()/10.0)-3.15;
    IZQUIERDA=(sensor3.readRangeContinuousMillimeters()/10.0)-7.65;
    delay(1);

    digitalWrite(Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigger, LOW);
    UT=pulseIn(Echo, HIGH);
    US_ENFRENTE=UT/58.2;

    if(DERECHA>250.0)
    DERECHA=250.0;

    if(IZQUIERDA>250.0)
    IZQUIERDA=250.0;

    if(US_ENFRENTE>250.0)
    US_ENFRENTE=250.0;

    ProD=(a*DERECHA)+((1-a)*ProD);
    if(ENFRENTE>250.0)
      ENFRENTE=250.0;
      //VL_Bueno=false;

    ProE=(a*ENFRENTE)+((1-a)*ProE);
    
    ProI=(a*IZQUIERDA)+((1-a)*ProI);
    Pro_US=(a*US_ENFRENTE)+((1-a)*Pro_US);

    if(ProE<180.0)
    ENFRENTE_V=(Pro_US+ProE)/2.0;
  else
  ENFRENTE_V=min(ProE, Pro_US);

    delay(1);
    
    //return VL_Bueno;
}

void ReceiveSerial(){

    unsigned long timeout=millis();
    mensaje="";
    while(mensaje.length() == 0) {
    Serial.println("Esperando...");
    if(millis()-timeout>=350)
    break;
    delay(1);
    if (Serial2.available()) {
        mensaje = Serial2.readStringUntil('\n');
        mensaje.trim();
        Numeros = mensaje.toInt();
    }
}
}

void SetServo(){
    if(pos>160)
    pos=160;

    if(pos<20)
    pos=20;

    servo.write(pos);
    Serial.print("Servo a: ");
    Serial.println (pos);
    delay(5);
}


void ReceiveSerialFull(){

  unsigned long timeout=millis();
  mensaje="";
  while(Serial2.available()){
  Serial2.read();}
    while(mensaje.length() == 0) 
    {
   // Serial.println("Esperando...");
    if(millis()-timeout>=350)
    break;
    
    if (Serial2.available()) {
        mensaje = Serial2.readStringUntil('\n');
        mensaje.trim();
        Numeros = mensaje.toInt();

        Sector=Numeros%10;
      
          if(Sector==0)
          Serial.println("No hay Color");
          else
          Color=(Numeros/10);

    }
    else
    {
      Color=0;
      Sector=0;
    }
    }
    delay(5);
}

void Pantalla(){
  display.clearDisplay();
  switch(Color){
    case 1: 
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,8);
    if(Sector==1)
    display.println("R         ");
    display.display();
    if(Sector==2)
    display.println("  R       ");
    display.display();    
    if(Sector==3)
    display.println("     R    ");
    display.display();   
    if(Sector==4)
    display.println("       R  ");
    display.display();    
    if(Sector==5)
    display.println("         R");
    display.display();    
    break;

    case 2: 
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,8);
    if(Sector==1)
    display.println("V         ");
    display.display();
    if(Sector==2)
    display.println("  V       ");
    display.display();    
    if(Sector==3)
    display.println("     V    ");
    display.display();   
    if(Sector==4)
    display.println("       V  ");
    display.display();    
    if(Sector==5)
    display.println("         V");
    display.display();    
    break;

    case 3:
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,8);
    if(Sector==1)
    display.println("M         ");
    display.display();
    if(Sector==2)
    display.println("  M       ");
    display.display();    
    if(Sector==3)
    display.println("     M    ");
    display.display();   
    if(Sector==4)
    display.println("       M  ");
    display.display();    
    if(Sector==5)
    display.println("         M");
    display.display();    
    break;

    default:
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,8);
    display.println("          ");
    display.display();   
    break;
  }
}

void ActoColor(){

  switch(Color){
    case 1: //Rojo esquivar por derecha
    if(Sector==1)
    pos=90;
    if(Sector==2)
    pos=105;
    if(Sector==3)
    pos=115;
    if(Sector==4)
    pos=125;
    if(Sector==5)
    pos=140;

    break;

    case 2: //Verde esquivar por izquierda
    if(Sector==1)
    pos=40;
    if(Sector==2)
    pos=48;
    if(Sector==3)
    pos=60;
    if(Sector==4)
    pos=78;
    if(Sector==5)
    pos=90;

    break;

    case 3: //magenta para aparcarse
    Serial.println("Lo ignoramos por ahora");
    break;

    default:
    Serial.println("Esto no debería pasar");
    break;

    SetServo();
  }
}

int PD_Centro (int input) {
 
    error = input;                               // determinar el error entre la consigna y la medición
 
    int output = kp*error;

    if(output>60)
    {
      output=60;
    }
    if(output<-60)
    {
      output=-60;
    }

    output=output+90;
 
    return output;
}
