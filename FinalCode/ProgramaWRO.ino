#include <Arduino.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>

VL53L0X sensor1; //derecha
VL53L0X sensor2; //delante
VL53L0X sensor3;//izquierda


#define XSHUT1  19  // Pin para apagar sensor 1
#define XSHUT2  5  // Pin para apagar sensor 2
#define XSHUT3  17 // Pin para apagar sensor 3

#define MOTOR_PWM_CHANNEL 2 // canal diferente al del servo
#define MOTOR_PWM_FREQ 12000
#define MOTOR_PWM_RESOLUTION 8

//Comunicacion Serial Colores
#define TX_PIN2 14  // Pin TX personalizado
#define RX_PIN2 15  // No se usa en el transmisor

int Numeros=0, Sector=0, Color=0;

String NOMBRE_COLORES[4] = {"Nada","Rojo", "Verde", "Magenta"};

float DERECHA, ENFRENTE, IZQUIERDA;

int dif,pos;
//Para Control PD
unsigned long currentTime, previousTime;
float elapsedTime;
double error, lastError, cumError, rateError;
//Distancia deseada
float Setpoint=0;
//Variables a Cambiar
float kp=3.8, kd=0.0001 ;

//IMU
int16_t Gyr_rawZ; //Read first Raw Data
float EgZ; //Data different from 0, avoids nan
float GradoRaw;
float AnguloZ=0.0;

unsigned long time1, timePrev, despliegue; //millis for integrals
//despliegue for showing angle every given time
float elapsedTime2;
float rad_to_deg = 180/3.141592654;
unsigned long Tvuelta;

float desired_angle = 0;

//Servo
int servoPin = 18;
Servo servo;

bool listo=false, listo2=false;
int casos=0;
float inicial=0.0;
int Esq=1, Vuelta=0;


//Motor
const int STBY=13;
const int PWM=27;
const int IN1=12;
const int IN2=26;
int Vel=170;

//Auxiliares
int i;
int ColorAnt, SectorAnt;
float auxColor, auxSector;
bool ModoColor=false;
float DLim=50, mult=0.75;

//Para serial
String linea;
String mensaje;


void setup() {


    Serial.begin(115200);
    delay(100);
    Serial2.begin(115200, SERIAL_8N1, RX_PIN2, TX_PIN2);
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
        Serial.println("Error al iniciar Sensor 1");
    } else {
        Serial.println("Sensor 1 iniciado correctamente");
    }

    // Encender y configurar el segundo sensor
    digitalWrite(XSHUT2, HIGH);
    delay(10);
    sensor2.setAddress(0x31);  // Cambiar dirección del sensor 2
    if (!sensor2.init()) {
        Serial.println("Error al iniciar Sensor 2");
    } else {
        Serial.println("Sensor 2 iniciado correctamente");
    }

    digitalWrite(XSHUT3, HIGH);
    delay(10);
    sensor3.setAddress(0x32);  // Cambiar dirección del sensor 2
    if (!sensor3.init()) {
        Serial.println("Error al iniciar Sensor 3");
    } else {
        Serial.println("Sensor 3 iniciado correctamente");
    }

    // Iniciar medición en ambos sensores
    sensor1.startContinuous();
    sensor2.startContinuous();
    sensor3.startContinuous();

delay(100);

EgZ=0.0;

 Wire.beginTransmission(0x68);
    if (Wire.endTransmission() == 0) {  
        Wire.beginTransmission(0x68);
        Wire.write(0x6B);
        Wire.write(0);  // Sacar de modo sleep
        Wire.endTransmission(true);
        Serial.println("MPU6050 iniciado correctamente");
    } else {
        Serial.println("Error: MPU6050 no detectado.");
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
        delay(100);

  servo.setPeriodHertz(50);             // 50 Hz para servo
  servo.attach(servoPin, 500, 2400);    // valores típicos: 500us (0°), 2400us (180°)
  Serial.println("Servo listo");

    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(STBY,OUTPUT);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(STBY, HIGH);

ledcAttachChannel(PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION, MOTOR_PWM_CHANNEL);

ledcAttachChannel(18, 50, 10, 1);

  servo.setPeriodHertz(50);
  servo.attach(18, 500, 2400);

 // ledcWrite(PWM, 200);  // valor entre 0-255 para 8 bits

linea ="";
while (linea.length() == 0) {
  if (Serial2.available()) {
    linea = Serial2.readStringUntil('\n');
    linea.trim(); // Limpia espacios y saltos de línea
    Serial.println("Recibido: " + linea);
  }
  delay(50);

}
}


void loop() {

//Habran switch cases para sentido horario, antihorario y colores con ambas variantes
//Para Hacer lo del angulo cuando se cumpla la condicion de pared enfrente, libre al lateral
//Seguir hasta haber una diferencia de 80 grados min entre inicial y actual

//Primero se va a detectar con comunicacion serial si hay colores o no, en caso de que no se ignora y se pasa al siguiente 

if (!listo){

    for(i=0;i<10;i++)
    {
        ReceiveSerial();
        Sector=Numeros%10;
        auxSector+=Sector;
      
          if(Sector==0)
        {
            Serial.println("No hay Color");
        }
          else
        {
            Color=(Numeros/10);
            auxColor+=1; //Para sacar promedio de cuantas veces se detecto color
            //Serial.println("Color: "+NOMBRE_COLORES[Color]+" En Sector "+String(Sector));
        }
    }

    //Aqui Sacamos un promedio de 10 lecturas, solo para comprobar antes de moverse si ve colores, si es asi...
    //podemos activar ModoColo como true, caso contrario leeremos mas adelante para comprobar y desactivar 
    //el modo mas adelante para evitar falsos positivos y consumir recursos

    auxColor=auxColor/10;

    //Mayor a %50 lo damos por bueno
    if(auxColor>0.5){
    ModoColor=true;
    Vel=150;
    }


  setLongRangeMode();
  ENFRENTE=(sensor2.readRangeContinuousMillimeters()/10.0);
  for(i=0; i<=50; i++)
  {
    ENFRENTE=(sensor2.readRangeContinuousMillimeters()/10.0);
    if(ENFRENTE<=300) break;
  }

  if(ENFRENTE>300.0)
  {
    setNormalMode();
    for(i=0; i<=50; i++)
  {
    ENFRENTE=(sensor2.readRangeContinuousMillimeters()/10.0);
    if(ENFRENTE<=300.0) break;
  }
  }

  if(ENFRENTE<=300.0)
  {
    inicial=ENFRENTE;
  }
  else
  {
    inicial=100.0;
  }
  listo=true;
}

setNormalMode();

ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits

while(!listo2){

    Distancias();
    ColorAnt=0;

    if(DERECHA>=18 && IZQUIERDA>=18) //Cuidar que no estemos muy pegados 
    {
    ReceiveSerialFull();
    ColorAnt=Color;
    if(Sector>0)
    {
      ReceiveSerialFull();
      if(Sector>0 && ColorAnt==Color)
      {
        ModoColor=true;
        Vel=155;
        ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits
        ActoColor();
      }
      else
      {
        Serial.println("Falsa Alarma");
      }
    }
    previousTime=millis();
    }
    else if(ENFRENTE<=DLim && IZQUIERDA>=90)
    {
    pos=10+(ENFRENTE*mult);
    casos=1;
    SetServo();

      delay(1);

      Tvuelta=timePrev=millis();
      while(abs(AnguloZ)<75.0)
      {
        anguloIMUYa();
        Serial.println(AnguloZ);
        if(millis()-Tvuelta>3000)
        break;
      }

      Serial.println("Ya salio");
      AnguloZ=0.0;
      
      pos=90;
      SetServo();
      delay(1);
      Distancias();

      while(IZQUIERDA>=90.0)
      {
        Distancias();
        dif= (DERECHA-35);  //para mantener pegado a un lado en lo que hay otro de referencia
        pos=PD_Centro (dif);
        ReceiveSerialFull();
        if(Sector>0 && (Color>0 && Color<4))
        {
          ReceiveSerialFull();
          if(Sector>0 && (Color>0 && Color<4))
          break;
        }
        Serial.println("Derecho");
      }
        
      previousTime=millis();
      break;
    }
      else if(ENFRENTE<=DLim && DERECHA>=90)
    {
    pos=170-(ENFRENTE*mult);
    casos=2;
    SetServo();
    delay(1);

      Tvuelta=timePrev=millis();
      
      while(abs(AnguloZ)<75.0)
      {
        anguloIMUYa();
        Serial.println(AnguloZ);
        if(millis()-Tvuelta>3000)
        break;
      }
      Serial.println("Ya salio");
      AnguloZ=0.0;

      pos=90;
      SetServo();
      while(DERECHA>=100.0)
      {
        Distancias();
        dif= (35-IZQUIERDA);  //para mantener pegado a un lado en lo que hay otro de referencia
        pos=PD_Centro (dif);

        ReceiveSerialFull(); //Volvemos a checar para evitar falsos positivos
        if(Sector>0 && (Color>0 && Color<4))
        {
          ReceiveSerialFull();
          if(Sector>0 && (Color>0 && Color<4))
          break;
        }
        Serial.println("Derecho");
      }
  
      previousTime=millis();
      break;
    }
    else if(DERECHA<100.0 && IZQUIERDA<100.0)
    {
    dif= (DERECHA-IZQUIERDA);
    pos=PD_Centro (dif);
    if(abs(pos)>4)
    {
    pos=90+pos;
    }
    else
    {
        pos=90;
    }
    }
    else
    pos=90;
    

    SetServo();
    delay(5);
}

listo2=true;

if(casos==1 && ModoColor==true)
casos=3;

if(casos==2 && ModoColor==true)
casos=4;
//Si se acaba la memoria, tal vez hacer los casos e irlos llamando porn turnos en el primer while, luego quedarse en uno solo

    switch(casos) {
      case 1:
      Serial.println("Sentido Antihorario No Color");
      while(Vuelta<3){

          if(Esq==4){
          Vuelta++;
          Esq=0;
          }
          if(Vuelta>=3)
          break;

        Distancias();

        if(ENFRENTE<=DLim && IZQUIERDA>=90)
    {
    pos=10+(ENFRENTE*mult);
    Esq++;
    SetServo();
      delay(1);

      Tvuelta=timePrev=millis();
      
      while(abs(AnguloZ)<75.0)
      {
        anguloIMUYa();
        Serial.println(AnguloZ);
        if(millis()-Tvuelta>3000)
        break;
      }
      Serial.println("Ya salio");
      AnguloZ=0.0;
    
      pos=90;
      SetServo();

      while(IZQUIERDA>=100.0){
        Distancias();
        dif= (DERECHA-35);  //para mantener pegado a un lado en lo que hay otro de referencia
        pos=PD_Centro (dif);
        Serial.println("Derecho");
      }

      previousTime=millis();
      //break;
    }
    else if(DERECHA<100.0 && IZQUIERDA<100.0)
    {
    dif= (DERECHA-IZQUIERDA);
    pos=PD_Centro (dif);

    if(abs(pos)>4)
    pos=90+pos;
    else
    pos=90;
    }
    else
    pos=90;

    SetServo();
    delay(5);

        if(Esq==4){
          Vuelta++;
          Esq=0;
        }
      }

      break;

      case 2:
      Serial.println("Sentido Horario No Color");
            while(Vuelta<3){

          if(Esq==4){
          Vuelta++;
          Esq=0;
          }
          if(Vuelta>=3)
          break;
            
        Distancias();

        if(ENFRENTE<=DLim && DERECHA>=90)
    {
    pos=170-(ENFRENTE*mult);
    Esq++;
    SetServo();
      delay(1);

      Tvuelta=timePrev=millis();
      
      while(abs(AnguloZ)<75.0)
      {
        anguloIMUYa();
        Serial.println(AnguloZ);
        if(millis()-Tvuelta>3000)
        break;
      }
      Serial.println("Ya salio");
      AnguloZ=0.0;
    
      pos=90;
      SetServo();

      while(DERECHA>=100.0){
        Distancias();
        dif= (35-IZQUIERDA);  //para mantener pegado a un lado en lo que hay otro de referencia
        pos=PD_Centro (dif);
        Serial.println("Derecho");
      }

      previousTime=millis();
      //break;
    }
    else if(DERECHA<100.0 && IZQUIERDA<100.0)
    {
    dif= (DERECHA-IZQUIERDA);
    pos=PD_Centro (dif);

    if(abs(pos)>4)
    pos=90+pos;
    else
    pos=90;
    }
    else
    pos=90;

    SetServo();
    delay(5);

        if(Esq==4){
          Vuelta++;
          Esq=0;
        }
      }

      break;

      case 3:

      Serial.println("Sentido Antihorario Color");

      //Estara Complicado
      while(Vuelta<3){

          if(Esq==4){
          Vuelta++;
          Esq=0;
          }
          if(Vuelta>=3)
          break;

          Distancias();
    ColorAnt=0;

    if(DERECHA>=18 && IZQUIERDA>=18) //Cuidar que no estemos muy pegados 
    {
    ReceiveSerialFull();
    ColorAnt=Color;
    if(Sector>0)
    {
      ReceiveSerialFull();
      if(Sector>0 && ColorAnt==Color)
      {
        ModoColor=true;
        Vel=155;
        ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits
        ActoColor();
      }
      else
      {
        Serial.println("Falsa Alarma");
      }
    }
    previousTime=millis();
    }
    else if(ENFRENTE<=DLim && IZQUIERDA>=90)
    {
    pos=10+(ENFRENTE*mult);
    Esq++;

    SetServo();

      delay(1);

      Tvuelta=timePrev=millis();
      while(abs(AnguloZ)<75.0)
      {
        anguloIMUYa();
        Serial.println(AnguloZ);
        if(millis()-Tvuelta>3000)
        break;
      }

      Serial.println("Ya salio");
      AnguloZ=0.0;
      
      pos=90;
      SetServo();
      delay(1);
      Distancias();

      while(IZQUIERDA>=90.0)
      {
        Distancias();
        dif= (DERECHA-35);  //para mantener pegado a un lado en lo que hay otro de referencia
        pos=PD_Centro (dif);
        ReceiveSerialFull();
        if(Sector>0 && (Color>0 && Color<4))
        {
          ReceiveSerialFull();
          if(Sector>0 && (Color>0 && Color<4))
          break;
        }
        Serial.println("Derecho");
      }
        
      previousTime=millis();
      //break;
    }
    else if(DERECHA<100.0 && IZQUIERDA<100.0)
    {
    dif= (DERECHA-IZQUIERDA);
    pos=PD_Centro (dif);
    if(abs(pos)>4)
    {
    pos=90+pos;
    }
    else
    {
        pos=90;
    }
    }
    else
    pos=90;
    

    SetServo();
    delay(5);
            if(Esq==4){
          Vuelta++;
          Esq=0;
        }
      }

      break;

      case 4:

      Serial.println("Sentido Horario Color");
      //Definitivamente Complicado

      while(Vuelta<3){

          if(Esq==4){
          Vuelta++;
          Esq=0;
          }
          if(Vuelta>=3)
          break;

      Distancias();
    ColorAnt=0;

    if(DERECHA>=18 && IZQUIERDA>=18) //Cuidar que no estemos muy pegados 
    {
    ReceiveSerialFull();
    ColorAnt=Color;
    if(Sector>0)
    {
      ReceiveSerialFull();
      if(Sector>0 && ColorAnt==Color)
      {
        ModoColor=true;
        Vel=155;
        ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits
        ActoColor();
      }
      else
      {
        Serial.println("Falsa Alarma");
      }
    }
    previousTime=millis();
    }
    else if(ENFRENTE<=DLim && IZQUIERDA>=90)
    {
    pos=10+(ENFRENTE*mult);
    casos=1;
    SetServo();

      delay(1);

      Tvuelta=timePrev=millis();
      while(abs(AnguloZ)<75.0)
      {
        anguloIMUYa();
        Serial.println(AnguloZ);
        if(millis()-Tvuelta>3000)
        break;
      }

      Serial.println("Ya salio");
      AnguloZ=0.0;
      
      pos=90;
      SetServo();
      delay(1);
      Distancias();

      while(IZQUIERDA>=90.0)
      {
        Distancias();
        dif= (DERECHA-35);  //para mantener pegado a un lado en lo que hay otro de referencia
        pos=PD_Centro (dif);
        ReceiveSerialFull();
        if(Sector>0 && (Color>0 && Color<4))
        {
          ReceiveSerialFull();
          if(Sector>0 && (Color>0 && Color<4))
          break;
        }
        Serial.println("Derecho");
      }
        
      previousTime=millis();
      break;
    }
      else if(ENFRENTE<=DLim && DERECHA>=90)
    {
    pos=170-(ENFRENTE*mult);
    casos=2;
    SetServo();
    delay(1);

      Tvuelta=timePrev=millis();
      
      while(abs(AnguloZ)<75.0)
      {
        anguloIMUYa();
        Serial.println(AnguloZ);
        if(millis()-Tvuelta>3000)
        break;
      }
      Serial.println("Ya salio");
      AnguloZ=0.0;

      pos=90;
      SetServo();
      while(DERECHA>=100.0)
      {
        Distancias();
        dif= (35-IZQUIERDA);  //para mantener pegado a un lado en lo que hay otro de referencia
        pos=PD_Centro (dif);

        ReceiveSerialFull(); //Volvemos a checar para evitar falsos positivos
        if(Sector>0 && (Color>0 && Color<4))
        {
          ReceiveSerialFull();
          if(Sector>0 && (Color>0 && Color<4))
          break;
        }
        Serial.println("Derecho");
      }
  
      previousTime=millis();
      //break;
    }
    else if(DERECHA<100.0 && IZQUIERDA<100.0)
    {
    dif= (DERECHA-IZQUIERDA);
    pos=PD_Centro (dif);
    if(abs(pos)>4)
    {
    pos=90+pos;
    }
    else
    {
        pos=90;
    }
    }
    else
    pos=90;
    
    SetServo();
    delay(5);

          if(Esq==4){
          Vuelta++;
          Esq=0;
        }

}

      break;

      default:

      Serial.println("Espero esto no suceda");

      break;
    }

    if(casos <=2)
    {
    Distancias();
    servo.write(90);
    Tvuelta=millis();
    while(ENFRENTE<inicial)
    {
      Distancias();
      if(millis()-Tvuelta>=2000)
      break;
    }
    }

    if(casos==3) //Antihorario color
    { 
    Vel=140;
    ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits
    Tvuelta=millis();

    while(Color!=4 && Sector<=3)
    {
      Distancias();
      if(DERECHA<=90)
      dif=(DERECHA-IZQUIERDA);
      else
      dif=(35-IZQUIERDA);

      pos=PD_Centro (dif);
      SetServo();

      ReceiveSerialFull();

      if(millis()-Tvuelta>=10000)
      break;;
    }
    pos=150;
    SetServo();
    
    Tvuelta=millis();
    while(ENFRENTE>12.0)
    {
      Distancias();
      if(millis()-Tvuelta>=2000)
      break;
    }
    }

    if(casos==4) //Antihorario color
    { 
    Vel=140;
    ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits
    Tvuelta=millis();


    while(Color!=4 && Sector>=3)
    {
      Distancias();
      if(IZQUIERDA<=90)
      dif=(DERECHA-IZQUIERDA);
      else
      dif=(DERECHA-35);
    
      pos=PD_Centro (dif);
      SetServo();

      ReceiveSerialFull();

      if(millis()-Tvuelta>=10000)
      break;
    }
    pos=30;
    SetServo();
    Tvuelta=millis();
    while(ENFRENTE>12.0)
    {
      Distancias();
      if(millis()-Tvuelta>=2000)
      break;
    }
    }

    //FIN DEL PROGRAMA

    Vel=0;
    ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits

    while(1)
    {
      Serial.println("Esto es todo amigos");
      delay(500);
    }

    //Si llegan a haber problemas para parkearse, poner otra condicion para buscar magenta en la vuelta 2 Esq 4

}

int PD_Centro (int input) {
 
    currentTime = millis();                          // obtener el tiempo actual
    elapsedTime = (currentTime - previousTime)/1000.0;     // calcular el tiempo transcurrido
        
    error = input;                               // determinar el error entre la consigna y la medición
    rateError = (error - lastError) / elapsedTime;       // calcular la derivada del error
 
    int output = kp*error + kd*rateError;     // calcular la salida del PID

    if(output>50)
    {
      output=50;
    }
    if(output<-50)
    {
      output=-50;
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
    DERECHA=(sensor1.readRangeContinuousMillimeters()/10.0)-1.2;
    delay(1);
    ENFRENTE=(sensor2.readRangeContinuousMillimeters()/10.0)-6.5;
    delay(1);
    IZQUIERDA=(sensor3.readRangeContinuousMillimeters()/10.0)-4.3;
    delay(1);

    Serial.print("Distancia Sensor 1: ");
    Serial.print(DERECHA);

    Serial.print("   Distancia Sensor 2: ");
    Serial.print(ENFRENTE);

    Serial.print("   Distancia Sensor 3: ");
    Serial.println(IZQUIERDA);
}

void ReceiveSerial(){

    unsigned long timeout=millis();
    mensaje="";
    while(mensaje.length() == 0) {
    Serial.println("Esperando...");
    if(millis()-timeout>=500)
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
    if(pos>135)
    pos=135;

    if(pos<45)
    pos=45;

    servo.write(pos);
    Serial.print("Servo a: ");
    Serial.println (pos);
    delay(5);
}


void ReceiveSerialFull(){

  unsigned long timeout=millis();
  mensaje="";
    while(mensaje.length() == 0) 
    {
    Serial.println("Esperando...");
    if(millis()-timeout>=500)
    break;
    delay(1);
    if (Serial2.available()) {
        mensaje = Serial2.readStringUntil('\n');
        mensaje.trim();
        Numeros = mensaje.toInt();

        Sector=Numeros%10;
      
          if(Sector==0)
        {
            Serial.println("No hay Color");
        }
          else
        {
            Color=(Numeros/10);
        }

    }
    else
    {
      Color=0;
      Sector=0;
    }
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
