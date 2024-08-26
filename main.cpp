//Universidad del Valle de Guatemala
//BE3023 Digital II
//Maria Reneé Ponce - 22601
//Proyecto 1
//Micro: ESP32 DEV Kit 1.0
//Sensor de temperatura que prende leds y mueve un servo según la temperatura

#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include <driver/ledc.h>

#define pintempe 34 //pin del sensor
#define rojo  21 //pin del led rojo
#define verde 22 //pin del led verde
#define amarillo 17 //pin del led amarillo 
#define servo 13 //pin del servo

//definir canal, frecuencia y resolucion para PWM
#define canalrojo  8 //numeros son los canales internos del ESPN
#define canalverde 9
#define canalamarillo 10
#define canalservo 0

#define freqPWM 50
#define resPWM 10


float sensortempo = 0;
float sensor = 0; //float para decimales

volatile int centenas = 0; //volatile para ahorrar memoria
volatile int decenas = 0;
volatile int unidades = 0; //int para enteros 
volatile int digitoencendido = 0;  // Para alternar entre los dos dígitos

const int segmentos[] = {27,12,33,25,26,19,14};  // Pines para los segmentos a-g
const int dp = 32;  // Pin para el punto decimal
const int digitos[] = {18, 5, 23};  // Pines para los dígitos comunes


String color;

//Paso 1: Instanciar configuración timer
hw_timer_t *timer0 = NULL;

// set up the 'counter' feed
AdafruitIO_Feed *tempcanal = io.feed("Temperatura");
AdafruitIO_Feed *semaforocanal = io.feed("Servo");

//interrupciones del boton
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//estructura del boton
struct Button {
  const uint8_t PIN;
  bool pressed;
  bool laststate;
};

Button button = {16, false, false};

//seguimos con las interrupciones del boton
void IRAM_ATTR BOTON_ISR(){
  portENTER_CRITICAL_ISR(&mux);
  button.pressed = true;
  portEXIT_CRITICAL_ISR(&mux);
}

void funciontemperatura(void); //renap de la funcion de temperatura
void funcionsemaforotemperatura(void); //renap de las leds
void funcionconversiondisplay(void); //renap de conversion del display 
void printdisplay(int numero); //funcion para prender display
void initTimer0(void);

void IRAM_ATTR Timer0_ISR (void){


  // Multiplexado de los tres dígitos
  if (digitoencendido == 0) {
    digitalWrite(digitos[1], LOW);  // Apaga el segundo dígito
    digitalWrite(digitos[2], LOW);  // Apaga el tercer dígito
    printdisplay(centenas);
    digitalWrite(digitos[0], HIGH);  // Enciende el primer dígito
    digitoencendido = 1;
  } else if (digitoencendido == 1) {
    digitalWrite(digitos[0], LOW);  // Apaga el primer dígito
    digitalWrite(digitos[2], LOW);  // Apaga el tercer dígito
    printdisplay(decenas);
    digitalWrite(dp, LOW);  // Enciende el punto decimal
    digitalWrite(digitos[1], HIGH);  // Enciende el segundo dígito
    digitoencendido = 2;
  } else if (digitoencendido == 2) {
    digitalWrite(digitos[0], LOW);  // Apaga el primer dígito
    digitalWrite(digitos[1], LOW);  // Apaga el segundo dígito
    digitalWrite(dp, HIGH);  // Apaga el punto decimal
    printdisplay(unidades);
    digitalWrite(digitos[2], HIGH);  // Enciende el tercer dígito
    digitoencendido = 0;
  }

}

void initrojo(void);
void initverde(void);
void initamarillo(void);
void initservo(void);
void enviar(float enviar1, String enviar2);

void setup() {

initrojo();
initverde();
initamarillo();
initservo();

  //velocidad del puerto serial
  Serial.begin(115200);

  initTimer0();

   for (int i = 0; i < 7; i++) {  // for para recorrer dos arreglos que configuran los pines del display como salidas
    pinMode(segmentos[i], OUTPUT);
  }
  pinMode(dp, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(digitos[i], OUTPUT);
  }

  pinMode(button.PIN, INPUT_PULLUP);
  attachInterrupt(button.PIN, BOTON_ISR, RISING);

  // wait for serial monitor to open
  while(! Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

}

void loop() {

  io.run();

  if(button.pressed != button.laststate){
    if(button.pressed){
    funciontemperatura();
    funcionsemaforotemperatura();
    funcionconversiondisplay();
    Serial.printf("centenas: %u \n", centenas); //verificar valores sin mapea
    Serial.printf("decenas: %u \n", decenas); //verificar valores sin mapea
    Serial.printf("unidades: %u \n", unidades); //verificar valores sin mapea
    enviar(sensor, color);
    button.pressed = false;
    }
  }

button.laststate = button.pressed; //regreso el estado del boton al estado anterior 



delay(50);
}

//void funciontemperatura usa una variable global para guardar el valor del sensor 
void funciontemperatura(void){
sensortempo = analogRead(pintempe); //leer el sensor

float voltaje = sensortempo * (3.3 / 4095.0);
  sensor = (voltaje * 100)-30;
Serial.printf("valor sin mapear: %f \n", sensortempo); //verificar valores sin mapear
Serial.printf("valor mapeado: %f \n", sensor); //verificar valores mapeados 
}

void initrojo(void){
  ledcSetup(canalrojo, freqPWM, resPWM); //configuro el canal, freq y resolucion
  ledcAttachPin(rojo, canalrojo); //asigno el pin al canal
  ledcWrite(canalrojo, 0); //le doy un valor para que empiece apagada
}

void initverde(void){
  ledcSetup(canalverde, freqPWM, resPWM); //configuro el canal, freq y resolucion
  ledcAttachPin(verde, canalverde); //asigno el pin al canal
  ledcWrite(canalverde, 0); //le doy un valor para que empiece apagada
}

void initamarillo(void){
  ledcSetup(canalamarillo, freqPWM, resPWM); //configuro el canal, freq y resolucion
  ledcAttachPin(amarillo, canalamarillo); //asigno el pin al canal
  ledcWrite(canalamarillo, 0); //le doy un valor para que empiece apagada
}

void initservo(void){
  ledcSetup(canalservo, freqPWM, resPWM); //configuro el canal, freq y resolucion
  ledcAttachPin(servo, canalservo); //asigno el pin al canal
  ledcWrite(canalservo, 128); //le doy un valor para que empiece en 0
}

void funcionsemaforotemperatura(void){
  if(sensor <=37.0){
    ledcWrite(canalverde, 1023);
    ledcWrite(canalrojo, 0);
    ledcWrite(canalamarillo, 0);
    ledcWrite(canalservo, 128);
    color = "#00ff00";
  }

  if(sensor >37.0 & sensor <= 37.5){
    ledcWrite(canalverde, 0);
    ledcWrite(canalrojo, 0);
    ledcWrite(canalamarillo, 1023);
    ledcWrite(canalservo, 77);
    color = "#ffff00";
  }

  if(sensor >37.5){
    ledcWrite(canalverde, 0);
    ledcWrite(canalrojo, 1023);
    ledcWrite(canalamarillo, 0);
    ledcWrite(canalservo, 26);
    color = "#ff0000";
  }
}

void funcionconversiondisplay(void){
  int temp = sensor*10;
  centenas = temp/100;
  temp = temp-(centenas*100);
  decenas = temp/10;
  temp = temp-(decenas*10);
  unidades = temp;
}

void printdisplay(int numero) {
  const int numeros[10][7] = {
    {0, 0, 0, 0, 0, 0, 1},  // 0
    {1, 0, 0, 1, 1, 1, 1},  // 1
    {0, 0, 1, 0, 0, 1, 0},  // 2
    {0, 0, 0, 0, 1, 1, 0},  // 3
    {1, 0, 0, 1, 1, 0, 0},  // 4
    {0, 1, 0, 0, 1, 0, 0},  // 5
    {0, 1, 0, 0, 0, 0, 0},  // 6
    {0, 0, 0, 1, 1, 1, 1},  // 7
    {0, 0, 0, 0, 0, 0, 0},  // 8
    {0, 0, 0, 0, 1, 0, 0}   // 9
  };

  for (int i = 0; i < 7; i++) {
    digitalWrite(segmentos[i], numeros[numero][i]);
  }
}

//funcion para inicializar el timer 0
void initTimer0(void){
  //Paso 2: Seleccionar #Timer, Prescaler, Flag
  timer0 = timerBegin(0,80,true);
  //Paso 3: Definir función de ISR(HANDLER)
  timerAttachInterrupt(timer0,&Timer0_ISR, true);
  //Paso 4: Establecer alarma (TimerTicks) y si queremos reload
  timerAlarmWrite(timer0,1000, true);
  // Paso 5: Habilitar la alarma
  timerAlarmEnable(timer0);
}


// función para enviar el valor de temperatura que es decimal (float) y el color del semáforo (string) porque lo mandamos en hex
void enviar(float enviar1, String enviar2){
Serial.print("sending -> ");
Serial.println(enviar1);
tempcanal ->save(enviar1);
Serial.print("sending -> ");
Serial.println(enviar2);
semaforocanal ->save(enviar2);
delay(3000);
}
