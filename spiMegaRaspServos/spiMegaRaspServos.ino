#include <SPI.h>
#include "SPI_anything.h"
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


#define PIN_SERVO_0 13
#define PIN_SERVO_1 3
#define PIN_SERVO_2 7
#define PIN_SERVO_3 6
#define PIN_SERVO_4 15
#define PIN_SERVO_5 4
#define PIN_SERVO_6 12
#define PIN_SERVO_7 2
#define PIN_SERVO_8 5
#define PIN_SERVO_9 18
#define PIN_SERVO_10 0
#define PIN_SERVO_11 19

#define MIN_RANGE_SERVO_0 700 //calcanhar_eixo_X
#define MIN_RANGE_SERVO_1 700 //calcanhar_eixo_Y
#define MIN_RANGE_SERVO_2 700 //joelho
#define MIN_RANGE_SERVO_3 700 //coxa
#define MIN_RANGE_SERVO_4 700 //pelves
#define MIN_RANGE_SERVO_5 700 //calcanhar_eixo_X
#define MIN_RANGE_SERVO_6 700 //calcanhar_eixo_Y
#define MIN_RANGE_SERVO_7 700 //joelho
#define MIN_RANGE_SERVO_8 700 //coxa
#define MIN_RANGE_SERVO_9 700 //pelves
#define MIN_RANGE_SERVO_10 700 //tragetory esq
#define MIN_RANGE_SERVO_11 700 //tragetory dir


#define MAX_RANGE_SERVO_0 2400
#define MAX_RANGE_SERVO_1 2400
#define MAX_RANGE_SERVO_2 2400
#define MAX_RANGE_SERVO_3 2400
#define MAX_RANGE_SERVO_4 2400
#define MAX_RANGE_SERVO_5 2400
#define MAX_RANGE_SERVO_6 2400
#define MAX_RANGE_SERVO_7 2400
#define MAX_RANGE_SERVO_8 2400
#define MAX_RANGE_SERVO_9 2400
#define MAX_RANGE_SERVO_10 2400
#define MAX_RANGE_SERVO_11 2400

int motors[] = {90+20, 90-1, 90, 90-5, 90+5,          90+10, 90+7, 90, 90+5, 90-15,       90+7, 90+49}; //posição motores
int qi[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     //posição inicial calibração copia

Servo servo_0;
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;
Servo servo_7;
Servo servo_8;
Servo servo_9;
Servo servo_10;
Servo servo_11;

int tempoDelayServo = 0;

typedef struct tanto
{
//  char cth= '#';
  byte pos[12];
}bloco;

char faz[16];
bool state = false;

ISR (SPI_STC_vect){
  //delayMicroseconds(100);
  state = SPI_readAnything(faz, 8);
}// end of interrupt routine SPI_STC_vect

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // now turn on interrupts
  SPCR |= _BV(SPIE);

  initServos();
  
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  //initServos();
  iddleState();
  
  leitura();
  if(state){
    state = false;
//    for(int i = 0; i < 16; i++){
//      Serial.print(int(faz[i]));
//      Serial.print(" ");
//    }
    Serial.print((int)faz[4]);
    Serial.print("  ");
    Serial.println((int)faz[12]);
    //Serial.println("Recebido e nos conformes!!");
    //walkState();
    
    
  }
  
  

}

void initServos() {

  qi[0] = motors[0];
  qi[1] = motors[1];
  qi[2] = motors[2];
  qi[3] = motors[3];
  qi[4] = motors[4];
  qi[5] = motors[5];
  qi[6] = motors[6];
  qi[7] = motors[7];
  qi[8] = motors[8];
  qi[9] = motors[9];
  qi[10] = motors[10];
  qi[11] = motors[11];

  servo_0.attach(PIN_SERVO_0, MIN_RANGE_SERVO_0, MAX_RANGE_SERVO_0);
  servo_1.attach(PIN_SERVO_1, MIN_RANGE_SERVO_1, MAX_RANGE_SERVO_1);
  servo_2.attach(PIN_SERVO_2, MIN_RANGE_SERVO_2, MAX_RANGE_SERVO_2);
  servo_3.attach(PIN_SERVO_3, MIN_RANGE_SERVO_3, MAX_RANGE_SERVO_3);
  servo_4.attach(PIN_SERVO_4, MIN_RANGE_SERVO_4, MAX_RANGE_SERVO_4);
  servo_5.attach(PIN_SERVO_5, MIN_RANGE_SERVO_5, MAX_RANGE_SERVO_5);
  servo_6.attach(PIN_SERVO_6, MIN_RANGE_SERVO_6, MAX_RANGE_SERVO_6);
  servo_7.attach(PIN_SERVO_7, MIN_RANGE_SERVO_7, MAX_RANGE_SERVO_7);
  servo_8.attach(PIN_SERVO_8, MIN_RANGE_SERVO_8, MAX_RANGE_SERVO_8);
  servo_9.attach(PIN_SERVO_9, MIN_RANGE_SERVO_9, MAX_RANGE_SERVO_9);
  servo_10.attach(PIN_SERVO_10, MIN_RANGE_SERVO_10, MAX_RANGE_SERVO_10);
  servo_11.attach(PIN_SERVO_11, MIN_RANGE_SERVO_11, MAX_RANGE_SERVO_11);
  writeServos(tempoDelayServo);
}

void walkState() {
  motors[0] = qi[0] + faz[0];
  motors[1] = qi[1] + faz[1];
  motors[2] = qi[2] + faz[2];
  motors[3] = qi[3] + faz[3];
  motors[4] = qi[4] + faz[4];
  motors[5] = qi[5] + faz[8]*-1;
  motors[6] = qi[6] + faz[9]*-1;
  motors[7] = qi[7] + faz[10]*-1;
  motors[8] = qi[8] + faz[11]*-1;
  motors[9] = qi[9] + faz[12]*-1;

//  motors[0] = qi[0];
//  motors[1] = qi[1] + faz[1];
//  motors[2] = qi[2];
//  
//  motors[3] = qi[3];
//  motors[4] = qi[4];
//  
//  motors[5] = qi[5];
//  motors[6] = qi[6] + faz[9]*-1;
//  motors[7] = qi[7];
//  
//  motors[8] = qi[8];
//  motors[9] = qi[9];
  
  writeServos(tempoDelayServo);
}

void iddleState() {
  motors[0] = qi[0] + 0;
  motors[1] = qi[1] + 16;
  motors[2] = qi[2] + 36;
  motors[3] = qi[3] + (-20);
  motors[4] = qi[4] + 0;
  motors[5] = qi[5] + 0;
  motors[6] = qi[6] + (-16);
  motors[7] = qi[7] + (-36);
  motors[8] = qi[8] + 20;
  motors[9] = qi[9] + 0;
 
  writeServos(tempoDelayServo);
}

void leitura() {
  while(!Serial2.available());
    int s = Serial2.read();
    if(s==255){
          for(int i=0;i<16;i++){
               while(!Serial2.available());
               faz[i] = Serial2.read() - 90;
          }
          while(!Serial2.available());
          s = Serial2.read();
          if(s == 254){
            state = true;
          }
    }
}

void writeServos(int espera) {
  servo_0.writeMicroseconds(map(motors[0], 0, 180, MIN_RANGE_SERVO_0, MAX_RANGE_SERVO_0));
  servo_1.writeMicroseconds(map(motors[1], 0, 180, MIN_RANGE_SERVO_1, MAX_RANGE_SERVO_1));
  servo_2.writeMicroseconds(map(motors[2], 0, 180, MIN_RANGE_SERVO_2, MAX_RANGE_SERVO_2));
  servo_3.writeMicroseconds(map(motors[3], 0, 180, MIN_RANGE_SERVO_3, MAX_RANGE_SERVO_3));
  servo_4.writeMicroseconds(map(motors[4], 0, 180, MIN_RANGE_SERVO_4, MAX_RANGE_SERVO_4));
  servo_5.writeMicroseconds(map(motors[5], 0, 180, MIN_RANGE_SERVO_5, MAX_RANGE_SERVO_5));
  servo_6.writeMicroseconds(map(motors[6], 0, 180, MIN_RANGE_SERVO_6, MAX_RANGE_SERVO_6));
  servo_7.writeMicroseconds(map(motors[7], 0, 180, MIN_RANGE_SERVO_7, MAX_RANGE_SERVO_7));
  servo_8.writeMicroseconds(map(motors[8], 0, 180, MIN_RANGE_SERVO_8, MAX_RANGE_SERVO_8));
  servo_9.writeMicroseconds(map(motors[9], 0, 180, MIN_RANGE_SERVO_9, MAX_RANGE_SERVO_9));
  servo_10.writeMicroseconds(map(motors[10], 0, 180, MIN_RANGE_SERVO_10, MAX_RANGE_SERVO_10));
  servo_11.writeMicroseconds(map(motors[11], 0, 180, MIN_RANGE_SERVO_11, MAX_RANGE_SERVO_11));
  
  delay(espera);
}
