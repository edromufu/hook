#include <Arduino.h>

#define VRX1 36
#define VRY1 39
#define SW1 34

#define VRX2 33
#define VRY2 25
#define SW2 23

#define LED 2

#define MAXINPUT 4095

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / ((in_max - in_min) * 1.0) + out_min;
}

void printIfChanged(const char *flag, int previous, int current) {
    if (previous != current) {
        Serial.print(flag);
        Serial.print("=");
        Serial.println(current);
    }
}

class Joystick {
private:
    int currentX, previousX, currentY, previousY, currentButton, previousButton;
    int xPort, yPort, bPort;
    const char *name;

public:
    Joystick(int valXport, int valYport, int valBport, const char *valName);
    void start();
    void read();
    void print();
};

Joystick::Joystick(int valXport, int valYport, int valBport, const char *valName) {
    xPort = valXport;
    yPort = valYport;
    bPort = valBport;
    name = valName;
}

void Joystick::start() {
    pinMode(xPort, INPUT);
    pinMode(yPort, INPUT);
    pinMode(bPort, INPUT_PULLUP);
    Serial.println("STARTED");
}

void Joystick::read() {
    previousX = currentX;
    previousY = currentY;
    previousButton = currentButton;

    currentX = mapf(analogRead(xPort), 0, MAXINPUT, -5, 5);
    currentY = mapf(analogRead(yPort), 0, MAXINPUT, -5, 5);
    currentButton = digitalRead(bPort);

    printIfChanged(name, previousX, currentX);
    printIfChanged(name, previousY, currentY);
    printIfChanged(name, previousButton, currentButton);
}

void Joystick::print() {
    Serial.print(name);
    Serial.print(" X: ");
    Serial.print(currentX);
    Serial.print(" Y: ");
    Serial.print(currentY);
    Serial.print(" Button: ");
    Serial.print(currentButton);
    Serial.println();
}

Joystick left(VRX1, VRY1, SW1, "Left");
Joystick right(VRX2, VRY2, SW2, "Right");

bool printLeft = true;

void setup() {
    Serial.begin(115200);
    left.start();
    right.start();
    pinMode(LED, OUTPUT);
}

void loop() {
    if (printLeft) {
        left.read();
        //left.print();
    } else {
        right.read();
        //right.print();
    }
    
    printLeft = !printLeft;
    
    delay(100);
}
