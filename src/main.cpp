#include <Arduino.h>
#include <SPI.h>
#include "Timer.h"
#include <Adafruit_NeoPixel.h> // Afin de gérer notre LED RGB
#include "WIFIConnector_MKR1000.h"
#include "MQTTConnector.h"


#define PIN_LED 4     // Control signal, connect to DI of the LED
#define NUM_LED 1     // Number of LEDs in a strip

Adafruit_NeoPixel RGB_Strip = Adafruit_NeoPixel(NUM_LED, PIN_LED, NEO_GRB + NEO_KHZ800); // Variable LED RGB

// Déclarations des pins des boutons.
const int BUTON_PIN1 = 6;
const int BUTON_PIN2 = 7;
const int BUTON_PIN3 = 8;

// Notre variable timer afin de gérer millis()
Timer Pause;
unsigned long  delayPause = 3000;

// Variables pour l'état de chaque bouton
int Brightness = 128; // Variable pour la luminosité de la LED.
int StateB1 = 0;
int StateB2 = 0;
int StateB3 = 0;

// Déclarationd des fonctions.
void SetLightRGB();
void colorWipe(uint32_t c);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Configuration des broches.
  pinMode(BUTON_PIN1, INPUT);
  pinMode(BUTON_PIN2, INPUT);
  pinMode(BUTON_PIN3, INPUT);
  // Configuration de notre LED RGB.
  RGB_Strip.begin();
  RGB_Strip.show();
  RGB_Strip.setBrightness(Brightness);    // Set brightness, 0-255 (darkest - brightest)
  // Setup wifi and Mqtt
  wifiConnect();
  MQTTConnect();
  Pause.startTimer(delayPause);
}

void loop() {
  // put your main code here, to run repeatedly:
  StateB1 = digitalRead(BUTON_PIN1);
  StateB2 = digitalRead(BUTON_PIN2);
  StateB3 = digitalRead(BUTON_PIN3);
  // Actions à faire à chaque delay : envoyer sur TB. 
  if(Pause.isTimerReady()){
    appendPayload("StateB1", StateB1);
    appendPayload("StateB2", StateB2);
    appendPayload("StateB3", StateB3);
    sendPayload();
    Serial.print("L'état des boutons : ");
    Serial.print(StateB1);
    Serial.print(" ");
    Serial.print(StateB2);
    Serial.print(" ");
    Serial.print(StateB3);
    Serial.print("\n\n");
    Pause.startTimer(delayPause);
  }
    SetLightRGB();
    ClientMQTT.loop();
}

void SetLightRGB(){
  int R = 0, G = 0, B = 0;
  if(DataRecu == "001"){
    R = 255;

  }
  if(DataRecu == "010")
    G = 255;
  if(DataRecu == "00")
    B = 255;
  Brightness = Speed; // "Speed" provient de MQTTConnector, variable global qui recoit l'entier provenant du RPC
                      // sur le set du brightness.
  RGB_Strip.setBrightness(Brightness);    // Set brightness, 0-255 (darkest - brightest)
  colorWipe(RGB_Strip.Color(R, G, B));
}

void colorWipe(uint32_t c) {
  for (uint16_t i = 0; i < RGB_Strip.numPixels(); i++) {
    RGB_Strip.setPixelColor(i, c);
    RGB_Strip.show();
  }
}