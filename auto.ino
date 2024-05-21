#include "Wheels.h"
#include "TimerOne.h"
#include "IRremote.hpp"
#include "PinChangeInterrupt.h"
#include <Servo.h>

// #define BLINK_PIN 
// #define MAX_IDLE_TIME 10000;

// podłączanie lc:
// SDA - A4
// SCL - A5

// podłączanie RC:
// S - signal
// "-" - ground
// ostatni - +

// podłączanie sonaru
// czerwony     zasil
// brązowy      masa
// żółty        signal

// piny dla sonaru (HC-SR04)
#define TRIG A4     // do zmiany na 2
#define ECHO A5     // do zmiany na 3

// pin kontroli serwo (musi być PWM)
#define SERVO 11

Servo serwo;

Wheels w;
volatile char cmd;

// kody pilota:
// 1: 69  2: 70  3: 71   4: 68   5: 64   6: 67   7: 7  8: 21   9: 9  0: 22   *: 25   #: 13   up: 24  left: 8   right: 90   down: 82  ok: 28 
constexpr uint8_t RECV_PIN{2};
enum State{OFF, ON} state;

const uint16_t s1 = 0x28;  // Taste 5

void setup(){

    w.attach(12, 4, 3, 8, 7, 5);
    pinMode(3, OUTPUT); // L power
    pinMode(12, OUTPUT); // LT
    pinMode(4, OUTPUT); // LP
    pinMode(7, OUTPUT);  // PP
    pinMode(8, OUTPUT);  // PT    
    pinMode(5, OUTPUT);  // P power

    pinMode(TRIG, OUTPUT);    // TRIG startuje sonar
    pinMode(ECHO, INPUT);     // ECHO odbiera powracający impuls

    // set speed
    analogWrite(3, 255);
    analogWrite(5, 255);

    // setup serial
    Serial.begin(9600);
    serwo.attach(SERVO);

    // Serial.print("Initiated pins.");

    // w.goForward(5);

    // setup radio receiver
    IrReceiver.begin(RECV_PIN);
    // Serial.print(F("Ready to receive IR signals at pin "));
    // Serial.println(RECV_PIN);
}

int time_current = 0;
int time_prev_signal = 0;
int time_diff = 0;
int signal = 0;
int max_idle_time = 200;
int is_auto = 0;


void loop(){
    time_current = millis();
    time_diff = time_current - time_prev_signal;
    signal = irReceive();

    if(is_auto == 0){
            // check if signal is non-zero
        if(signal != 0){
            time_prev_signal = millis();
        }

        // if there is a time without receiving a signal, stop
        if(time_diff > max_idle_time){
            w.stop();
        }

        switch(signal){
            case 24:
                if(time_diff < max_idle_time){
                    // if there is no obstacle in front
                    if(!check_obstacle_front()){
                        w.forward();
                        Serial.print("Signal: ");
                        Serial.print(signal);
                        Serial.print(", going forward\n");
                    }
                    else{
                        w.stop();
                    }
                    
                }
                break;
            case 82:
                if(time_diff < max_idle_time){
                    w.back();
                    Serial.print("Signal: ");
                    Serial.print(signal);
                    Serial.print(", going back\n");  
                }
                break;
            case 8:
                if(time_diff < max_idle_time){
                    w.forwardRight();
                    Serial.print("Signal: ");
                    Serial.print(signal);
                    Serial.print(", going left\n");  
                }
                break;        
            case 90:
                if(time_diff < max_idle_time){
                    w.forwardLeft();
                    Serial.print("Signal: ");
                    Serial.print(signal);
                    Serial.print(", going right\n");  
                }
                break;
            case 69:
                is_auto = 1;
                Serial.println("changing state to autonomous");
                break;          
        }   // end switch
    }   // end state 0
    else{
        // go back to manual if we receive signal "ok"
        if(signal == 70){
            Serial.println("changing state to manual");
            w.stop();
            is_auto = 0;
        }

        int auto_state = 0;

        // LOGIC:
        // 1. Try moving forward
        // 2. Move forward as far as you can
        // 3. Check possible right and left paths and pick a turn
        // 4. Goto 1

        // STATES:
        // 0    standing
        // 1    moving forward
        // 2    looking
        // 3    moving front right
        // 4    moving front left
        switch(auto_state){
            case 0: // preparing to go forward
                // if there's no obstacle, go to state 1
                if(!check_obstacle_front()){
                    w.forward();
                    auto_state = 1;
                }
                else{
                    w.stop();
                    auto_state = 2;
                }
                break;
            case 1: // going forward
                if(check_obstacle_front()){
                    w.stop();
                    auto_state = 2;
                }
                break;
            case 2: // handling wall infront
                w.goBack(20);
                int left_distance = 0;
                int right_distance = 0;
                left_distance = get_distance_left();
                right_distance = get_distance_right();
                if(left_distance < 30 && right_distance < 30){
                    // w.goBack(20);
                    w.stop();
                    break;
                }
                if(left_distance > right_distance){
                    w.forwardRight();
                    delay(125);
                    w.stop();
                }
                else{
                    w.forwardLeft();
                    delay(125);
                    w.stop();
                }
                auto_state = 0;
                break;
        }
    }
}


// Receives signal and prints it to console
uint16_t irReceive(){
    uint16_t received{0};
  
    if(IrReceiver.decode()){
        // IrReceiver.printIRResultShort(&Serial);
        if(IrReceiver.decodedIRData.protocol == UNKNOWN){
            // We have an unknown protocol here, print more info
            // IrReceiver.printIRResultRawFormatted(&Serial, true);
        }
        if(IrReceiver.decodedIRData.protocol == NEC){
            received = IrReceiver.decodedIRData.command;
            Serial.print("Hex command: 0x");
            Serial.println(received, HEX);
        }
        IrReceiver.resume();
    }

    if(received != 0){
        Serial.print("Decimal command: ");
        Serial.println(received);
    }
    return received;
}


int lookAndTellDistance(byte angle) {
    unsigned long tot;      // czas powrotu (time-of-travel)
    unsigned int distance;

    Serial.print("Patrzę w kącie ");
    Serial.print(angle);
    serwo.write(angle);
    
    /* uruchamia sonar (puls 10 ms na `TRIGGER')
    * oczekuje na powrotny sygnał i aktualizuje
    */
    digitalWrite(TRIG, HIGH);
    delay(10);
    digitalWrite(TRIG, LOW);
    tot = pulseIn(ECHO, HIGH);

    /* prędkość dźwięku = 340m/s => 1 cm w 29 mikrosekund
    * droga tam i z powrotem, zatem:
    */
    distance = tot/58;

    Serial.print(": widzę coś w odległości ");
    Serial.println(distance);
    return distance;
}


int check_obstacle_front(){
    int distance = lookAndTellDistance(90);
    // delay(500);
    if(distance <= 30){
        return 1;
    }
    else{
        return 0;
    }
}


int check_obstacle_left(){
    int distance = lookAndTellDistance(180);
    delay(500);
    if(distance <= 30){
        return 1;
    }
    else{
        return 0;
    }
}


int check_obstacle_right(){
    int distance = lookAndTellDistance(0);
    delay(500);
    serwo.write(90);
    delay(500);
    if(distance <= 30){
        return 1;
    }
    else{
        return 0;
    }
}


// CHECK
int get_distance_left(){
    int distance = lookAndTellDistance(45);
    delay(500);
    serwo.write(90);
    delay(500);
    return distance;
}


// CHECK
int get_distance_right(){
    int distance = lookAndTellDistance(135);
    delay(500);
    serwo.write(90);
    delay(500);
    return distance;
}
