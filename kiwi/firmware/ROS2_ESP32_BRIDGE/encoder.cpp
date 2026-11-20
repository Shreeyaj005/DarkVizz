#include "encoder.h"

volatile long encoder0Pos = 0;    // encoder 1  (wheel_1 / LEFT slot)
volatile long encoder1Pos = 0;    // encoder 2  (wheel_2 / RIGHT slot)
volatile long encoder2Pos = 0;    // encoder 3  (wheel_3 / BACK slot)

/* Quadrature decoding, identical for all three encoders.
   Edge on A: +1 when A != B, otherwise -1.
   Edge on B: +1 when A == B, otherwise -1.
   Per-encoder direction is flipped with ENCx_SIGN in config.h. */
static inline long stepA(int a, int b) { return (a != b) ? 1 : -1; }
static inline long stepB(int a, int b) { return (a == b) ? 1 : -1; }

// ************** encoder 1 *********************
void doEncoderA(){
  encoder0Pos = encoder0Pos + ENC0_SIGN * stepA(digitalRead(encoder0PinA), digitalRead(encoder0PinB));
}
void doEncoderB(){
  encoder0Pos = encoder0Pos + ENC0_SIGN * stepB(digitalRead(encoder0PinA), digitalRead(encoder0PinB));
}

// ************** encoder 2 *********************
void doEncoderC(){
  encoder1Pos = encoder1Pos + ENC1_SIGN * stepA(digitalRead(encoder1PinA), digitalRead(encoder1PinB));
}
void doEncoderD(){
  encoder1Pos = encoder1Pos + ENC1_SIGN * stepB(digitalRead(encoder1PinA), digitalRead(encoder1PinB));
}

// ************** encoder 3 *********************
void doEncoderE(){
  encoder2Pos = encoder2Pos + ENC2_SIGN * stepA(digitalRead(encoder2PinA), digitalRead(encoder2PinB));
}
void doEncoderF(){
  encoder2Pos = encoder2Pos + ENC2_SIGN * stepB(digitalRead(encoder2PinA), digitalRead(encoder2PinB));
}

void init_encoders(){
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  attachInterrupt(encoder0PinA, doEncoderA, CHANGE);
  attachInterrupt(encoder0PinB, doEncoderB, CHANGE);

  attachInterrupt(encoder1PinA, doEncoderC, CHANGE);
  attachInterrupt(encoder1PinB, doEncoderD, CHANGE);

  attachInterrupt(encoder2PinA, doEncoderE, CHANGE);
  attachInterrupt(encoder2PinB, doEncoderF, CHANGE);
}

long readEncoder(int i) {
  if (i == LEFT) return encoder0Pos;
  else if (i == RIGHT) return encoder1Pos;
  else return encoder2Pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT)       encoder0Pos = 0L;
  else if (i == RIGHT) encoder1Pos = 0L;
  else                 encoder2Pos = 0L;
}

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
  resetEncoder(BACK);
}
