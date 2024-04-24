#include "Wheels.h"
#include "TimerOne.h"
#include "IRremote.hpp"
#include "PinChangeInterrupt.h"

// #define BLINK_PIN 
// #define MAX_IDLE_TIME 10000;

Wheels w;
volatile char cmd;


// obsługa pilota
// kody pilota:
// 1: 69  2: 70  3: 71   4: 68   5: 64   6: 67   7: 7  8: 21   9: 9  0: 22   *: 25   #: 13   up: 24  left: 8   right: 90   down: 82  ok: 28 
constexpr uint8_t RECV_PIN{2};
enum State{OFF, ON} state;

const uint16_t s1 = 0x3F;  // Taste 5


void setup(){
    w.attach(12, 4, 3, 8, 7, 5);
    pinMode(3, OUTPUT); // L power
    pinMode(12, OUTPUT); // LT
    pinMode(4, OUTPUT); // LP
    pinMode(7, OUTPUT);  // PP
    pinMode(8, OUTPUT);  // PT    
    pinMode(5, OUTPUT);  // P power


  

    // set speed
    analogWrite(3, 255);
    analogWrite(5, 255);

    // setup serial
    Serial.begin(9600);
    Serial.println("Forward: WAD");
    Serial.println("Back: ZXC");
    Serial.println("Stop: S");

    Serial.print("Initiated pins.");

    // setup timer1
    // pinMode(LED_BUILTIN, OUTPUT);
    // Timer1.initialize(1); // Fire An Interrupt Every 10ms
    // // Timer1.attachInterrupt(blinkInterrupt);
    // // Timer1.attachInterrupt(printInterrupt);
    // // Timer1.attachInterrupt(signalInterrupt);

    // setup

    // setup radio receiver
    IrReceiver.begin(RECV_PIN);
    Serial.print(F("Ready to receive IR signals at pin "));
    Serial.println(RECV_PIN);
    // enter_password();
}

int time_current = 0;
int time_prev_signal = 0;
int time_diff = 0;
int signal = 0;
int max_idle_time = 10000;

void loop(){
    time_current = millis();
    time_diff = time_current - time_prev_signal;
    signal = irReceive();
    // perform_action(signal);
    if(time_diff > max_idle_time){
        w.stop();
    }

    switch(signal){
        case 24:
            if(time_diff < max_idle_time){
                w.forward();
                Serial.print("Signal: ");
                Serial.print(signal);
                Serial.print(", going forward\n");
            }
            break;
        case 82:
            // w.goBack(5);
            if(time_diff < max_idle_time){
                w.back();
                Serial.print("Signal: ");
                Serial.print(signal);
                Serial.print(", going forward\n");  
            }
            break;
    }

    // handling serial input
    if(Serial.available()){
        cmd = Serial.read();
        switch(cmd){
            case 'w': w.forward(); break;
            case 'x': w.back(); break;
            case 'a': w.forwardLeft(); break;
            case 'd': w.forwardRight(); break;
            case 'z': w.backLeft(); break;
            case 'c': w.backRight(); break;
            case 's': w.stop(); break;
            case '1': w.setSpeedLeft(75); break;
            case '2': w.setSpeedLeft(200); break;
            case '9': w.setSpeedRight(75); break;
            case '0': w.setSpeedRight(200); break;
            case '5': w.setSpeed(100); break;
            case 'f': w.goForward(5); break;
            case 'b': w.goBack(5); break;
        }
    }
}


// void signalInterrupt(void){
//     uint16_t signal = irReceive();
//     perform_action(signal);
//     Serial.print(signal);
//     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
// }


void blinkInterrupt(void){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Serial.print("Interrupted\n");
}


void printInterrupt(void){
    Serial.print("interrupt\n");
}


// Receives signal and prints it to console
uint16_t irReceive(){
    uint16_t received{0};
  
    if (IrReceiver.decode()){
        IrReceiver.printIRResultShort(&Serial);
        if(IrReceiver.decodedIRData.protocol == UNKNOWN){
            // We have an unknown protocol here, print more info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
        }
        if(IrReceiver.decodedIRData.protocol == NEC){
            received = IrReceiver.decodedIRData.command;
            Serial.print("Command: 0x");
            Serial.println(received, HEX);
        }
        IrReceiver.resume();
    }

    if(received != 0){
        Serial.print("Received signal: ");
        Serial.println(received);
    }
    return received;
}


// Waits until user provides correct signal
int enter_password(){
    Serial.print("Waiting for a password\n");
    int password = 111;
    int entered = 000;
    int signal = 0;
    while(entered != 69){
        entered = 000;
        delay(100);
        signal = irReceive();
        entered = signal;
    }
    Serial.println("Entered a proper password");
}


// Performs action depending on signal
void perform_action(uint16_t signal){
    switch(signal){   
        case 24:
            w.goForward(5);
            Serial.print("Signal: ");
            Serial.print(signal);
            Serial.print(", going forward\n");
            break;
        case 82:
            w.goBack(5);
            Serial.print("Signal: ");
            Serial.print(signal);
            Serial.print(", going forward\n");
            break;
  }
}

