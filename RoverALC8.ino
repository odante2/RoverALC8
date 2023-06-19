// Projeto Rover ALC8 idealizado por ARR
// O mecanismo irá se mover quando o radio é controlado
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h> // 
//#ifdef __AVR__
//#include <avr/power.h>
#define PIN 4 // pino para o led endereçavel
#define NUMPIXELS 30
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GBR + NEO_KHZ800);
#define DELAYVAL 500
////////////
// DEFINIÇÕES
#define endereco  0x27 // Endereços comuns: 0x27, 0x3F
#define colunas   16
#define linhas    2
/////////

// INSTANCIANDO OBJETOS
LiquidCrystal_I2C lcd(endereco, colunas, linhas);
//////
#include <VarSpeedServo.h> 
VarSpeedServo servo1; // Motor tampa direita
VarSpeedServo servo2; // Motor tampa esquerda
#define pinServo1  7 // Servo motor Tampa Direita Drone
#define pinServo2  6 // Servo motor Tampa Direita Drone
//////////// Variaveis receptor radio controle
int canal1 = 53;
int canal2 = 51;
int canal3 = 49;
int canal4 = 47;
int canal5 = 45;
int canal6 = 43;
unsigned long valorX;
unsigned long valorY;
unsigned long valorZ;
unsigned long valorA;
unsigned long valorB;
unsigned long valorC;
////////////
///////////////Variaveis Fita Led
int vermelho = 0;
int verde = 0;
int azul = 255;
///////////////////////
///////////Variaveis para tampa drone
int fecha = 1;
void setup()
{
pinMode (2, OUTPUT); // Habilita tensão dos LEDs vermelhos traseira
pinMode (3, OUTPUT); // Habilita tensão para os servo motores
pinMode (41, OUTPUT); // Habilita tensão para o alarme 12V
////////////
pinMode (8, OUTPUT); // Pino controle motor esquerda
pinMode (9, OUTPUT); // Pino controle motor esquerda (apresentando problema)
pinMode (10, OUTPUT); // Pino controle motor direita
pinMode (11, OUTPUT); // Pino controle motor direita
  
lcd.init(); // INICIA A COMUNICAÇÃO COM O DISPLAY
  lcd.backlight(); // LIGA A ILUMINAÇÃO DO DISPLAY
  lcd.clear(); // LIMPA O DISPLAY

  lcd.print("---ARR----");
  lcd.clear();
  pinMode(canal1, INPUT); //Canal 1
  pinMode(canal2, INPUT);
  pinMode(canal3, INPUT);
  pinMode(canal4, INPUT);
  pinMode(canal5, INPUT);
  pinMode(canal6, INPUT);
  Serial.begin(9600);
  servo1.attach(pinServo1); // Servo motor tampa direita
  servo2.attach(pinServo2); // Servo motor tampa esquerda

  pixels.begin();
}

void loop()
{
  // analogWrite (9,LOW); // sentido para frente motor esquerdo do rover
//  analogWrite (8,LOW); // sentido para tras motor esquerdo do rover (apresentando problema)
 // analogWrite (11,LOW); // sentido para frente motor direito do rover
 // analogWrite (10,LOW); // sentido para tras motor direito do rover

  
  //pixel.clear();
pixels.setPixelColor (0, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (1, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (2, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (3, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (4, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (5, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (6, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (7, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (8, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (9, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (10, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (11, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (12, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (13, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (14, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (15, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (16, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (17, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (18, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (19, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (20, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (21, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (22, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (23, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (24, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (25, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (26, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (27, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (28, pixels.Color(azul,verde,vermelho));
pixels.setPixelColor (29, pixels.Color(azul,verde,vermelho));
  pixels.show();
digitalWrite (3, LOW); /// desligando os servos para programar APAGAR LINHA
digitalWrite (2, HIGH); // ligando as luzes de aviso
  ///////////////////////////////////////////////////////// Fechamento da tampa drone

if ((valorB > 1950 ) && (valorC <990) && (fecha == 0)){// Condição para fechamento da tampa do drone.
fecha = 1;//Essa variavel garante que os servo motores permanecam desligados quando chegar no final do laço
digitalWrite (3, HIGH); // Habilita tensão para os servo motores
digitalWrite (41, HIGH); // Habilita tensão alarme 12V
servo1.slowmove(170,260); // Fecha tampa direita
servo2.slowmove(5,260); // Fecha tampa esquerda 
delay (10000);
digitalWrite (3, LOW); // Desabilita tensão para os servo motores
digitalWrite (41, LOW); // Desabilita tensão alarme 12V
}
///////// Abertura da tampa drone
if ((valorB < 989 ) && (valorC > 1940) && (fecha == 1)){// Condição para fechamento da tampa do drone.
servo1.slowmove(170,260); // Fecha tampa direita (coloquei essa linha para tentar minimizar o efeito abertura fantasma)
servo2.slowmove(5,260); // Fecha tampa esquerda (coloquei essa linha para tentar minimizar o efeito abertura fantasma)
fecha = 0; // Essa variavel garante que os servo motores permanecam desligados quando chegar no final do laço
digitalWrite (3, HIGH); // Habilita tensão para os servo motores
digitalWrite (41, HIGH); // Habilita tensão alarme 12V
servo1.slowmove(170,260); // Fecha tampa direita (coloquei essa linha para tentar minimizar o efeito abertura fantasma)
servo2.slowmove(5,260); // Fecha tampa esquerda (coloquei essa linha para tentar minimizar o efeito abertura fantasma)
delay (2000);
servo1.slowmove(85,260); // Abre tampa direita 85 abre 170 fecha
servo2.slowmove(85,260); // Abre tampa esquerda 85 abre 5 fecha
delay(10000);
digitalWrite (3, LOW); // Desabilita tensão para os servo motores
digitalWrite (41, LOW); // Desabilita tensão alarme 12V
}
/////////////////////////////////////////////////////////////////////////////// Ciclo tampa drone


//lcd.clear();
lcd.print("Automacao");
  lcd.setCursor(0, 1); // POSICIONA O CURSOR NA PRIMEIRA COLUNA DA LINHA 2
  lcd.print("     ARR");
  
  // Debug dos valores recebidos pelo Radio
  valorX = pulseIn(canal1, HIGH);
 // Serial.print("Valor lido X: ");
 // Serial.print(valorX);
 // Serial.println("us");
 valorY = pulseIn(canal2, HIGH);
  Serial.print("  Valor lido Y: ");
  Serial.print(valorY);
  Serial.println("us");
  valorZ = pulseIn(canal3, HIGH);
 // vermelho=map(valorZ, 1971, 972, 0, 255);
  //azul=map(valorZ, 1971, 972, 150, 255);
  //verde=map(valorZ, 1971, 972, 255, 0);
 // Serial.print("  Valor lido Z: ");
 //Serial.print(valorZ);
 // Serial.println("us");
  valorA = pulseIn(canal4, HIGH);
  //Serial.print("  Valor lido A: ");
 //Serial.print(valorA);
  //Serial.println("us");
  valorB = pulseIn(canal5, HIGH);
 //Serial.print("  Valor lido B: ");
 //Serial.print(valorB);
 // Serial.println("us");
  valorC = pulseIn(canal6, HIGH);
 // Serial.print("  Valor lido C: ");
 //Serial.print(valorC);
 // Serial.println("us");


if (valorX >= 1701){ // analogico do controle direita para direita
  analogWrite (9,255); // sentido para frente motor esquerdo do rover
  analogWrite (8,LOW); // sentido para tras motor esquerdo do rover (apresentando problema)
  analogWrite (11,LOW); // sentido para frente motor direito do rover
  analogWrite (10,255); // sentido para tras motor direito do rover
  
}
if ((valorY <= 1700)&&(valorX <= 1700)){ //analogico do controle direita no meio
  analogWrite (9,LOW); // sentido para frente motor esquerdo do rover
  analogWrite (8,LOW); // sentido para tras motor esquerdo do rover (apresentando problema)
  analogWrite (11,LOW); // sentido para frente motor direito do rover
  analogWrite (10,LOW); // sentido para tras motor direito do rover
}
if (valorY >= 1701){ // analogico do controle direita para frente
  analogWrite (9,255); // sentido para frente motor esquerdo do rover
  analogWrite (8,LOW); // sentido para tras motor esquerdo do rover (apresentando problema)
  analogWrite (11,255); // sentido para frente motor direito do rover
  analogWrite (10,LOW); // sentido para tras motor direito do rover
  
}

  
 // delay(10000);
 // digitalWrite (7, LOW);
  delay(10);
}


///int base = analogRead(0); 
 ////  base=map(base, 155, 852, 0, 180);
 ////  servo1.write(base); 
