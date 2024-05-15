#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Wheels.h"
#include "TimerOne.h"

// defining arrows

uint8_t arrowRight[8] = {0b01000, 0b01100, 0b00110, 0b11111, 0b11111, 0b00110, 0b01100, 0b01000};


#define SET_MOVEMENT(side,f,b) digitalWrite(side[0], f);\
                               digitalWrite(side[1], b)

Wheels::Wheels(){ }


void Wheels::attachRight(int pF, int pB, int pS){
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsRight[0] = pF;
    this->pinsRight[1] = pB;
    this->pinsRight[2] = pS;
}


void Wheels::attachLeft(int pF, int pB, int pS){
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsLeft[0] = pF;
    this->pinsLeft[1] = pB;
    this->pinsLeft[2] = pS;
}


void Wheels::setSpeedRight(uint8_t s){
    analogWrite(this->pinsRight[2], s);
}


void Wheels::setSpeedLeft(uint8_t s){
    analogWrite(this->pinsLeft[2], s);
}


void Wheels::setSpeed(uint8_t s){
    setSpeedLeft(s);
    setSpeedRight(s);
}


void Wheels::attach(int pRF, int pRB, int pRS, int pLF, int pLB, int pLS){
    this->attachRight(pRF, pRB, pRS);
    this->attachLeft(pLF, pLB, pLS);
}


void Wheels::forwardLeft(){
    SET_MOVEMENT(pinsLeft, HIGH, LOW);
}


void Wheels::forwardRight(){
    SET_MOVEMENT(pinsRight, HIGH, LOW);
}


void Wheels::backLeft(){
    SET_MOVEMENT(pinsLeft, LOW, HIGH);
}


void Wheels::backRight(){
    SET_MOVEMENT(pinsRight, LOW, HIGH);
}


void Wheels::forward(){
    this->forwardLeft();
    this->forwardRight();
}


void Wheels::back(){
    this->backLeft();
    this->backRight();
}


void Wheels::stopLeft(){
    SET_MOVEMENT(pinsLeft, LOW, LOW);
}


void Wheels::stopRight(){
    SET_MOVEMENT(pinsRight, LOW, LOW);
}


void Wheels::stop(){
    this->stopLeft();
    this->stopRight();
}


byte LCDAddress = 0x28;

void Wheels::goForward(int cm){
    LiquidCrystal_I2C lcd(LCDAddress, 16, 2);

    lcd.setCursor(0, 0);

    forward();
    for(int i=0; i<cm; i++){
        // printing 
        // TODO: CHECK???
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(cm - i - 1);

        // printing distance left
        lcd.setCursor(2, 0);
        lcd.print("cm");

        // printing speed
        lcd.setCursor(0, 1);
        lcd.print(255);
        lcd.setCursor(12, 1);
        lcd.print(255);

        // applying the movement

        delay(125);
    }
    stop();

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(0);
    lcd.setCursor(12, 1);
    lcd.print(0);

    // // end beeping
    // TimerEnd();
}


void Wheels::goBack(int cm){
    LiquidCrystal_I2C lcd(LCDAddress, 16, 2);
    // // start beeping
    // TimerStart();
    
    lcd.setCursor(0, 0);

    back();
    for(int i=0; i<cm; i++){
        // printing 
        // TODO: CHECK???
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(cm - i - 1);

        // printing distance left
        lcd.setCursor(2, 0);
        lcd.print("cm");

        // printing speed
        lcd.setCursor(0, 1);
        lcd.print(-255);
        lcd.setCursor(12, 1);
        lcd.print(-255);

        // applying the movement

        delay(125);
    }
    stop();

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(0);
    lcd.setCursor(12, 1);
    lcd.print(0);
}


