#define TX_PIN2 14  // Pin TX personalizado
#define RX_PIN2 15  // No se usa en el transmisor

 int Numeros, Sector=0, Color;

String NOMBRE_COLORES[3] = {"Rojo", "Verde", "Magenta"};

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX_PIN2, TX_PIN2);
}

void loop() {
    if (Serial2.available()) {
        String mensaje = Serial2.readStringUntil('\n');
        mensaje.trim();
        Numeros = mensaje.toInt();
        Sector=Numeros%10;

    }

        if(Sector==0)
        {
            Serial.println("No hay Color");
        }
        else
        {
            Color=(Numeros/10)-1;
            Serial.println("Color: "+NOMBRE_COLORES[Color]+" En Sector "+String(Sector));
        }

        delay(100);
    
}