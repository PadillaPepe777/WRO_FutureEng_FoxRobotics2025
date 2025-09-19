#include <Arduino.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>

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

int Numeros=0, Sector=0, Color=0;

//String NOMBRE_COLORES[4] = {"Nada","Rojo", "Verde", "Magenta"};

//IMU
int16_t Gyr_rawZ; //Read first Raw Data
float EgZ; //Data different from 0, avoids nan
float GradoRaw;
float AnguloVuelta=77.0;
float AnguloZ=0.0;

//Evitar estrellarse
float kp=0.95;
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
int Vel=80;
int VelVuelta=70;
  
//Para serial
String mensaje;

//Filtro distancias
float ProD, ProE, ProI, Pro_US;
float a=0.9;
float DERECHA, ENFRENTE, IZQUIERDA, US_ENFRENTE, ENFRENTE_V;
unsigned long UT;
//bool PROM=false;

float LimiteSensor=250.0;

float dif; //entre izq y derecha

//Auxiliares
int i;
float Acumulador=0.0;
float Acumulador2=0.0;
float Acumulador3=0.0;
int ColorAnt, SectorAnt;
float auxColor, auxSector;
bool ModoColor=false;
int ContColor=0;
int casos=0;

//Limite+multiplicador enfrente angulo servo
float DEnfrente=75.0, DPeligro=11.0, DVuelta=80.0, mult=0.85; //este ultimo es para angulo servo de vuelta
//

//valores iniciales + contador vueltas
int Esq=1;
int Vueltas=0;
float InicioE=0.0;
float InicioD=0.0;
float InicioI=0.0;

unsigned long displayTimer;

String NOMBRE_COLORES[4] = {"N","R", "V", "M"};

void setup() {
    
    Serial.begin(115200);
    delay(100);
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
    sensor3.setAddress(0x32);  // Cambiar dirección del sensor 3
    if (!sensor3.init()) {
        Serial.println("❌ Error al iniciar Sensor 3");
        delay(5000);
    } else {
        Serial.println("✅ Sensor 3 iniciado correctamente");
    }

    DERECHA=ENFRENTE=IZQUIERDA=US_ENFRENTE=0.0;
    ProD=ProE=ProI=Pro_US=DERECHA;

    // Iniciar medición en ambos sensores
    sensor1.startContinuous();
    sensor2.startContinuous();
    sensor3.startContinuous();

    Serial.println("Todo bien aqui");

delay(200);

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

  //servo.setPeriodHertz(50);
  //servo.attach(18, 500, 2400);

 // ledcWrite(PWM, 200);  // valor entre 0-255 para 8 bits
 Serial2.begin(115200, SERIAL_8N1, RX_PIN2, TX_PIN2);

  pos=90;
  SetServo();
  delay(80);

  ledcWrite(PWM, 0);
  AnguloZ=0.0;
/*
  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  */
}

void loop() {

  while(Ready==false){
  Ready=digitalRead(Boton);
  delay(40);
  }

  for (i=0;i<10;i++)
  {
    Distancias();
    Acumulador=Acumulador+US_ENFRENTE;
    Acumulador2=Acumulador2+DERECHA;
    Acumulador3=Acumulador3+IZQUIERDA;
    delay(5);
  }

    InicioE=Acumulador/10.0; //Distancia Frontal de inicio
    if(InicioE<50.0 || InicioE>250.0)
    InicioE=150.0; //por si salimos de los margenes

    InicioD=Acumulador2/10.0; //Distancia Lateral de inicio
    if(InicioD<15.0 || InicioE>85.0)
    InicioD=30.0; //Por si nos salimos de los margenes

    InicioI=Acumulador3/10.0;
    if(InicioD<15.0 || InicioE>85.0)
    InicioD=30.0; //Por si nos salimos de los margenes

  ledcWrite(PWM, Vel);  // valor entre 0-255 para 8 bits
  //displayTimer=millis();
  timePrev=millis();
  //display.clearDisplay();


  /*Prioridades
  Oportunidad de Dar Vuelta a algun lado
  Laterales >Peligro entonces
  Escanear por colores, si hay color es acto color, si no hay presentes se hace PD
  Si Laterales<peligro, directamente hace PD
  Aqui incluir que Izq>Dvuelta, tomar de referencia Der, y viceversa
  Caso contrario la diferencia se saca de ambos
  */

  while(Ready2==false)
  {
    Distancias();

    if(ENFRENTE_V<=DEnfrente && (ProD>=DVuelta || ProI>=DVuelta))
    {
      ledcWrite(PWM, 0);
      if(ProI>=DVuelta) //Sentido AntiHorario
      {
        pos=160;
        casos=1; //Antihorario
        SetServo();
        delay(5);

        AnguloZ=0.0;
        
        ledcWrite(PWM, VelVuelta);
        delay(1);
        Tvuelta=timePrev=millis();
        anguloIMUYa();
          while(abs(AnguloZ)<AnguloVuelta)
          {
            anguloIMUYa();
            //Serial.println(AnguloZ);
            if(millis()-Tvuelta>2200)
            break;
          }

          pos=90;
          SetServo();
          Distancias();
          //Serial.println("Ya salio");
          AnguloZ=0.0;
          delay(1);
          Distancias();
          
      
          //Tvuelta=millis();

          while(ProI>=DVuelta) {
            Distancias();
            dif= (ProD-30.0);  //para mantener pegado a un lado en lo que hay otro de referencia
            pos=PD_Centro (dif);
            SetServo();

              ReceiveSerialFull();
              if(Sector>0)
              {
                ContColor++;
                //ModoColor=true;
                
                if(Color<3)
                { //Checar que no sea magenta, aun no se ocupa
                  ledcWrite(PWM, VelVuelta);  // valor entre 0-255 para 8 bits
                  ActoColor();
                  TiempoMuerto(250); //250 ms de tiempo muerto
                }
                ledcWrite(PWM,Vel);
                delay(1);
                break;
              }
          } 
        
          //previousTime=millis();
          Ready2=true;
          //Serial.println("Derecho");
          pos=90;
          SetServo();
      }
      else //Sentido horario ProD>=Dvuelta
      {
        pos=20;
        casos=2; //horario
        SetServo();
        delay(1);

        AnguloZ=0.0;
        ledcWrite(PWM, VelVuelta);
        delay(1);
        Tvuelta=timePrev=millis();
        anguloIMUYa();
          while(abs(AnguloZ)<AnguloVuelta)
          {
            anguloIMUYa();
            //Serial.println(AnguloZ);
            if(millis()-Tvuelta>2200)
            break;
          }
          pos=90;
          SetServo();
          Distancias();
          Serial.println("Ya salio");
          AnguloZ=0.0;
          delay(1);
          Distancias();
          
          //Tvuelta=millis();

          while(ProD>=DVuelta) {
            Distancias();
            dif= (30.0-ProI);  //para mantener pegado a un lado en lo que hay otro de referencia
            pos=PD_Centro (dif);
            SetServo();

              ReceiveSerialFull();
              if(Sector>0)
              {
                //ModoColor=true;
                ContColor++;
                //ModoColor=true;
                if(Color<3)
                { //Checar que no sea magenta, aun no se ocupa
                  ledcWrite(PWM, VelVuelta);  // valor entre 0-255 para 8 bits
                  ActoColor();
                  TiempoMuerto(250); //250 ms de tiempo muerto
                }
                ledcWrite(PWM,Vel);
                delay(1);
                break;
              }

          } 
          //previousTime=millis();
          Ready2=true;
          Serial.println("Derecho");
          pos=90;
          SetServo();
        }
        ledcWrite(PWM, Vel);
        
      }//Fin primer If
      else if(ProD>DPeligro && ProI>DPeligro)
      {
          ReceiveSerialFull();
          if(Sector>0)
          {               
            ContColor++;
            if(Color<3)
            { //Checar que no sea magenta, aun no se ocupa
              ledcWrite(PWM, VelVuelta);  // valor entre 0-255 para 8 bits
              ActoColor();
              TiempoMuerto(250); //250 ms de tiempo muerto
            }
            else
            {
              Distancias();
              dif= (ProD-ProI);
              pos=PD_Centro (dif);
              SetServo();
            }
            ledcWrite(PWM, Vel);
            delay(1);

          }
          else
          {
            dif= (ProD-ProI);
            pos=PD_Centro (dif);
            SetServo();
          }
        
        Distancias();
      } //Fin If para colores Y distancia peligro
      else //Aqui haremos PD porque esta en distancia peligrosa y no esta libre de dar vuelta aun, pero deberiamos checar estar dentro de los parametros en ambos lados
      {
        if(ProI<DVuelta && ProD<DVuelta)
          dif=(ProD-ProI);
        else if(ProI>=DVuelta)
          dif=(ProD-30.0);
        else
          dif=(30.0-ProI);

        pos=PD_Centro (dif);
        SetServo();
        Distancias();
      }//FIN ultimo else

      TiempoMuerto(6); //6 ms de delay para actualizar distancias y dar tiempo de hacer cosas a lo demas
    }//Fin Ready2

    //ledcWrite(PWM, 0);
    Distancias();
    if(ContColor>=10)
    {
      ModoColor=true;
      casos=casos+2;
      Serial2.println("START");
      delay(1);
    }
    else
    {
      Serial2.println("STOP");
      delay(1);
    }

      Distancias();
      dif= (ProD-ProI);
      pos=PD_Centro (dif);
      SetServo();
      ReceiveSerialFull();

      if(ModoColor==true && Color>3 && (ProD>DPeligro && ProI>DPeligro))
      {
          ledcWrite(PWM, VelVuelta);  // valor entre 0-255 para 8 bits
          ActoColor();
          TiempoMuerto(250); //250 ms de tiempo muerto
          ledcWrite(PWM, Vel);
      }

    switch (casos)
    {
      case 1: //Antihorario no color
      while(Vueltas<3)
      {
      Distancias();

    if(ENFRENTE_V<=DEnfrente && ProI>=DVuelta)
    {
        ledcWrite(PWM, 0);
        pos=160;
        //Antihorario
        SetServo();
        delay(5);
        Esq++;
        if(Esq>=4)
        {
          Esq=0;
          Vueltas=Vueltas+1;
        }

        AnguloZ=0.0;
        Tvuelta=timePrev=millis();
        ledcWrite(PWM, VelVuelta);
          while(abs(AnguloZ)<AnguloVuelta)
          {
            anguloIMUYa();
            Serial.println(AnguloZ);
            if(millis()-Tvuelta>2150)
            break;
          }

          pos=90;
          SetServo();
          Distancias();
          Serial.println("Ya salio");
          AnguloZ=0.0;
          delay(1);
          Distancias();
          
      
          //Tvuelta=millis();

          while(ProI>=DVuelta) {
            Distancias();
            if(Vueltas>=3)
            break;

            dif= (ProD-35.0);  //para mantener pegado a un lado en lo que hay otro de referencia
            pos=PD_Centro (dif);
            SetServo();
            } 
        
          //previousTime=millis();
          //Ready2=true;
          Serial.println("Derecho");
          pos=90;
          SetServo();
          ledcWrite(PWM, Vel);
      }
      else //Aqui haremos PD porque esta en distancia peligrosa y no esta libre de dar vuelta aun, pero deberiamos checar estar dentro de los parametros en ambos lados
      {
        if(ProI<DVuelta && ProD<DVuelta)
          dif=(ProD-ProI);
        else if(ProI>=DVuelta)
          dif=(ProD-30.0);
        else
          dif=(30.0-ProI);

        pos=PD_Centro (dif);
        SetServo();
        Distancias();
      }//FIN ultimo else

      TiempoMuerto(5); //6 ms de delay para actualizar distancias y dar tiempo de hacer cosas a lo demas
    }//end while 1
    break;

    case 2: //HORARIO NO COLOR
    while(Vueltas<3)
    {
      Distancias();

      if(ENFRENTE_V<=DEnfrente && ProD>=DVuelta)
      {
        ledcWrite(PWM, 0);
        pos=20;
        SetServo();
        delay(1);
        Esq++;
        if(Esq>=4)
        {
          Esq=0;
          Vueltas=Vueltas+1;
        }

        AnguloZ=0.0;
        Tvuelta=timePrev=millis();
        ledcWrite(PWM, VelVuelta);
          while(abs(AnguloZ)<AnguloVuelta)
          {
            anguloIMUYa();
            Serial.println(AnguloZ);
            if(millis()-Tvuelta>2150)
            break;
          }
          pos=90;
          SetServo();
          Distancias();
          //Serial.println("Ya salio");
          AnguloZ=0.0;
          delay(1);
          Distancias();
          

          while(ProD>=DVuelta) {

            Distancias();
            if(Vueltas>=3)
            break;
            dif= (30.0-ProI);  //para mantener pegado a un lado en lo que hay otro de referencia
            pos=PD_Centro (dif);
            SetServo();
            } 
          //previousTime=millis();
          Ready2=true;
          Serial.println("Derecho");
          pos=90;
          SetServo();
          ledcWrite(PWM, Vel);
        }
        else
        {
          if(ProI<DVuelta && ProD<DVuelta)
          dif=(ProD-ProI);
          else if(ProI>=DVuelta)
          dif=(ProD-30.0);
          else
          dif=(30.0-ProI);

          pos=PD_Centro (dif);
          SetServo();
          Distancias();
        }//FIN ultimo else

      TiempoMuerto(5); //6 ms de delay para actualizar distancias y dar tiempo de hacer cosas a lo demas
    }//end while 2
    break;

    case 3:
    DEnfrente=DEnfrente+10.0;
    while(Vueltas<3) //Antihorario Color
    {
      Distancias();
      if(ENFRENTE_V<=DEnfrente && ProI>=DVuelta)
      {
        ledcWrite(PWM, 0);
        pos=160;
        //Antihorario
        SetServo();
        delay(5);
        Esq++;
        if(Esq>=4)
        {
          Esq=0;
          Vueltas=Vueltas+1;
        }

        AnguloZ=0.0;
        Tvuelta=timePrev=millis();
        ledcWrite(PWM, VelVuelta);
          while(abs(AnguloZ)<AnguloVuelta)
          {
            anguloIMUYa();
            Serial.println(AnguloZ);
            if(millis()-Tvuelta>2150)
            break;
          }

          pos=90;
          SetServo();
          Distancias();
          Serial.println("Ya salio");
          AnguloZ=0.0;
          delay(1);
          Distancias();
          
      
          //Tvuelta=millis();

            while(ProI>=DVuelta) {
            Distancias();
            if(Vueltas>=3)
            break;
            dif= (ProD-42.0);  //para mantener pegado a un lado en lo que hay otro de referencia
            pos=PD_Centro (dif);
            SetServo();

            ReceiveSerialFull();
              if(Sector>0)
              {
                if(Color<3)
                { //Checar que no sea magenta, aun no se ocupa
                  ledcWrite(PWM, VelVuelta);  // valor entre 0-255 para 8 bits
                  ActoColor();
                  TiempoMuerto(250); //250 ms de tiempo muerto
                }
                ledcWrite(PWM,Vel);
                delay(1);
                break;
              }
          } 
          //previousTime=millis();
          //Serial.println("Derecho");
          pos=90;
          SetServo();
      }
      else if(ProD>DPeligro && ProI>DPeligro)
      {
          ReceiveSerialFull();
          if(Sector>0)
          {               
            ContColor++;
            if(Color<3)
            { //Checar que no sea magenta, aun no se ocupa
              ledcWrite(PWM, VelVuelta);  // valor entre 0-255 para 8 bits
              ActoColor();
              TiempoMuerto(250); //250 ms de tiempo muerto
            }
            else
            {
              Distancias();
              dif= (ProD-ProI);
              pos=PD_Centro (dif);
              SetServo();
            }
            ledcWrite(PWM, Vel);
            delay(1);

          }
          else
          {
            dif= (ProD-ProI);
            pos=PD_Centro (dif);
            SetServo();
          }
        
        Distancias();
      } //Fin If para colores Y distancia peligro
      else //Aqui haremos PD porque esta en distancia peligrosa y no esta libre de dar vuelta aun, pero deberiamos checar estar dentro de los parametros en ambos lados
      {
        if(ProI<DVuelta && ProD<DVuelta)
          dif=(ProD-ProI);
        else if(ProI>=DVuelta)
          dif=(ProD-30.0);
        else
          dif=(30.0-ProI);

        pos=PD_Centro (dif);
        SetServo();
        Distancias();
      }//FIN ultimo else


    }

    break;

    case 4: //Horario Color
    DEnfrente=DEnfrente+10.0;

    while(Vueltas<3)
    {

    }
    break;

    default:
    Serial.println("Esto no debería suceder");
    ledcWrite(PWM, 0);
    delay(2000);
    break;

    }//End switch case

    if(casos==1) //Antihorario color
    {
      while(ENFRENTE_V>(InicioE+5.0))
      {
          Distancias();
          dif=(ProD-InicioD);
          pos=PD_Centro (dif);
          SetServo();
          Distancias();
      }
      ledcWrite(PWM,0);
      pos=90;
      SetServo();

      while(1)
      {
        ledcWrite(PWM,0);
        Serial.println("Ya es todo");
        delay(1000);
      }
    }
    else if(casos==2) //horario Color
    {
        while(ENFRENTE_V>(InicioE+5.0))
      {
          Distancias();
          dif=(InicioI-ProI);
          pos=PD_Centro (dif);
          SetServo();
          Distancias();
      }
      ledcWrite(PWM,0);
      pos=90;
      SetServo();
      
      while(1)
      {
        ledcWrite(PWM,0);
        Serial.println("Ya es todo");
        delay(1000);
      }

    }
    else if(casos==3)//Antihorario Color
    {
      Distancias();
      pos = 90;
      SetServo();
      int contador = 0;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      ledcWrite(PWM, 70);
      delay(1);
      while (contador != 2) {
      Distancias();

      if (DERECHA<= 18.0) {
      contador++;
      if (contador==2)
      {ledcWrite(PWM, 0);
      break;}
      pos=90;
      SetServo();
      delay(200);
    }
    else
    {
      dif= (ProD-30.0);
      pos=PD_Centro (dif);
      SetServo();
    }
    //Por seguridad
    if(ENFRENTE_V<=30.0 && ProI>=DVuelta)
    {
      ledcWrite(PWM, 0);
      break;
    }

  }
  ledcWrite(PWM, 0);
  delay(500);
  unsigned long tRev = millis();
  pos = 150;
  SetServo();
  while (millis() - tRev < 400) {
  Distancias();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM, 70);
  }
  ledcWrite(PWM, 0);
  delay(500);
  unsigned long tRe1 = millis();
  pos = 30;
  SetServo();
  while (millis() - tRe1 < 600) {
    Distancias();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM, 70);
    }
  ledcWrite(PWM, 0);
  delay(500);
  unsigned long tRe2 = millis();
  pos = 150;
  SetServo();
  while (millis() - tRe2 < 600)
  {
    Distancias();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM, 70);
    }
  ledcWrite(PWM, 0);
  delay(500);
  unsigned long tRe3 = millis();
  pos = 30;
  SetServo();
  while (millis() - tRe3 < 279.36)
  {
    Distancias();
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(PWM, 70);
    }
  ledcWrite(PWM, 0);
  delay(500);
  pos = 90;
  SetServo();
  ledcWrite(PWM, 0);

  while(1)
  {
    Serial.println("Esto es todo amigos");
    delay(2000);
  }
    }
    else //casos==4 horario color
    {
    Distancias();
  pos = 90;
  SetServo();
  Distancias();
  dif= (30.0-ProI);
  pos=PD_Centro (dif);
  SetServo();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM, 70);

  int contador = 0;
  while (contador != 2) {
  Distancias();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM, 70);
      if (IZQUIERDA<= 18.0) {
      contador++;
      if (contador==2)
      {
        ledcWrite(PWM, 0);
        break;
      }
      pos=90;
      SetServo();
      delay(200);
    }
    else
    {
      dif= (30.0-ProI);
      pos=PD_Centro (dif);
      SetServo();
    }

    if(ENFRENTE_V<=30.0 && ProD>=DVuelta)
    {
      ledcWrite(PWM, 0);
      break;
    }
  }
  ledcWrite(PWM, 0);
  delay(500);
  unsigned long tRev = millis();
  pos = 30;
  SetServo();
  while (millis() - tRev < 400) {
  Distancias();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM, 70);
  }
  ledcWrite(PWM, 0);
  delay(500);
  unsigned long tRe1 = millis();
  pos = 150;
  SetServo();
  while (millis() - tRe1 < 600) {
    Distancias();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM, 70);
    }
  ledcWrite(PWM, 0);
  delay(500);
  unsigned long tRe2 = millis();
  pos = 30;
  SetServo();
  while (millis() - tRe2 < 600)
  {
    Distancias();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM, 70);
    }
  ledcWrite(PWM, 0);
  delay(500);
  unsigned long tRe3 = millis();
  pos = 150;
  SetServo();
  while (millis() - tRe3 < 279.36)
  {
    Distancias();
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(PWM, 70);
    }
  ledcWrite(PWM, 0);
  delay(500);
  pos = 90;
  SetServo();
  ledcWrite(PWM, 0);
  while(1)
  {
    Serial.println("Esto es todo amigos");
    delay(2000);
  }
  }
  

}//Fin Void LOOP

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

   if(abs(GradoRaw*elapsedTime2)>=0.02)
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

void Distancias(){
  //Implementamos filtros Pasabajos

    DERECHA=(sensor1.readRangeContinuousMillimeters()/10.0)-5.7;
    ENFRENTE=(sensor2.readRangeContinuousMillimeters()/10.0)-1.5; //estaba en 3.15, pero lo ponemos como -1.5 para que este igual al ultrasonico, que esta 2cm enfrente
    IZQUIERDA=(sensor3.readRangeContinuousMillimeters()/10.0)-7.8;

    digitalWrite(Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigger, LOW);
    UT=pulseIn(Echo, HIGH, 20000);
    US_ENFRENTE=UT/58.2;

    if(DERECHA>LimiteSensor)
    DERECHA=LimiteSensor;

    if(IZQUIERDA>LimiteSensor)
    IZQUIERDA=LimiteSensor;

    if(US_ENFRENTE>LimiteSensor || US_ENFRENTE==0.0)
    US_ENFRENTE=LimiteSensor;

    if(ENFRENTE>LimiteSensor || ENFRENTE<10.0)
      ENFRENTE=LimiteSensor;
      //VL_Bueno=false;

    ProD=(a*DERECHA)+((1-a)*ProD);
    ProE=(a*ENFRENTE)+((1-a)*ProE);
    ProI=(a*IZQUIERDA)+((1-a)*ProI);
    Pro_US=(a*US_ENFRENTE)+((1-a)*Pro_US);

    if(ProE<220.0)
    ENFRENTE_V=(Pro_US+ProE)/2.0;
    else
    ENFRENTE_V=Pro_US;

    delayMicroseconds(100);
}


void SetServo(){
    if(pos>155)
    pos=155;

    if(pos<25)
    pos=25;

    servo.write(pos);
    //Serial.print("Servo a: ");
    //Serial.println (pos);
    delay(1);
}

void ReceiveSerialFull(){
  
  Color=0;
  Sector=0;
  Numeros=0;
  mensaje="";
  unsigned long timeout=millis();
  
    while (Serial2.available()) {
    mensaje = Serial2.readStringUntil('\n');  // siempre sobrescribe con la más reciente
    }

    if (mensaje.length()>0) {
        
        mensaje.trim();
        Numeros = mensaje.toInt();

        Sector=Numeros%10;
      
          if(Sector==0)
          Color=0;
          else
        {
            Color=(Numeros/10);
            if(Color>3)
            Color=0;
        }
    }
    else
      Sector=Color=0;

    delay(1);
}

void ActoColor(){

  switch(Color){
    case 1: //Rojo esquivar por derecha
    if(Sector==1)
    pos=90;
    if(Sector==2)
    pos=75;
    if(Sector==3)
    pos=60;
    if(Sector==4)
    pos=50;
    if(Sector==5)
    pos=40;

    break;

    case 2: //Verde esquivar por izquierda
    if(Sector==1)
    pos=140;
    if(Sector==2)
    pos=130;
    if(Sector==3)
    pos=120;
    if(Sector==4)
    pos=105;
    if(Sector==5)
    pos=90;

    break;

    case 3: //magenta para aparcarse
    Serial.println("Ya deberia escribir esto");
    break;

    default:
    Serial.println("Esto no debería pasar");
    digitalWrite(STBY, LOW);
    break;
  }
  SetServo();
}

int PD_Centro (float input) {
 
    error = -input;                               // determinar el error entre la consigna y la medición
 
    int output = kp*error;

    if(output>55)
      output=55;

    if(output<-55)
      output=-55;

    output=output+90;
 
    return output;
}

void TiempoMuerto (int tiempo) {
  unsigned long timerM = millis();
  while(millis()-timerM<tiempo)
  {
    Distancias();
    if(ProD<=DPeligro && ProI<=DPeligro)
    break;
  }
}