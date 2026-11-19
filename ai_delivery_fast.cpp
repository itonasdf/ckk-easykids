#include <PTBOTAtomVX.h>
#include <Adafruit_NeoPixel.h>
#include "huskylens.hpp"

#define RED 0
#define GREEN 1
#define YELLOW 2
#define BLUE 3

int fl[3] = { 1, 150, 850 };
int fr[3] = { 3, 200, 880 };
int rl[3] = { 7, 200, 880 };
int rr[3] = { 5, 100, 500 };

Huskylens husky(&Wire);
Adafruit_NeoPixel strip(12, 4, NEO_GRB + NEO_KHZ800);

double heading = 0.0;
int apriltag = 0;
int current = 0;
int iteration = 0;
int aprilhash[4][4] = { // HOSPITAL, SHOP, CHARGING, SCHOOL
    { RED, YELLOW, GREEN, BLUE },
    { YELLOW, GREEN, BLUE, RED },
    { BLUE, RED, YELLOW, GREEN },
    { GREEN, BLUE, RED, YELLOW }
};

int colorhash[4][3] = {
    { 255, 0, 0 }, // RED
    { 0, 255, 0 }, // GREEN
    { 255, 255, 0 }, // YELLOW
    { 0, 0, 255 }, // BLUE
};

int depth[2] = {
    400, 200
};

double wraps(double val) {
    while (val > 180) val -= 360;
    while (val < -180) val += 360;
    return val;
}

void turn(double relative) {
    double kp = 1.5; double kd = 3.0;

    double target = heading + relative;
    double prev_error = 0.0;
    if (target > 180) target -= 360;
    if (target < -180) target += 360; 
    heading = target;

    while (1) {
        double error = target - angleRead(YAW);
        error = wraps(error);

        double derivative = error - prev_error;
        double result = (error * kp) + (derivative * kd);
        prev_error = error;

        if (fabs(error) <= 0.5) {
            motorStop();
            break;
        } else if (fabs(error) <= 30.0) {
            double power = copysign(12, error);
            motorWrite(power, power, -power, -power);
        } else {
            motorWrite(result, result, -result, -result);
        }
    }
}

void straightUntilAny(int power, int* sen1, int* sen2) {
    double kp = 1.8; double kd = 10.0;

    double prev_error = 0.0;

    while (1) {
        int sensor1 = map(adcRead(sen1[0]), sen1[1], sen1[2], 0, 100);
        int sensor2 = map(adcRead(sen2[0]), sen2[1], sen2[2], 0, 100);
        double error = heading - angleRead(YAW);
        error = wraps(error);

        double derivative = error - prev_error;
        double result = (error * kp) + (derivative * kd);
        prev_error = error;

        double left = power + result;
        double right = power - result;
        motorWrite(left, left, right, right);

        if (sensor1 >= 80 || sensor2 >= 80) {
            motorStop();
            break;
        }
    }
}

void straightUntil(int power, int* sen) {
    double kp = 1.8; double kd = 10.0;

    double prev_error = 0.0;

    while (1) {
        int sensor = map(adcRead(sen[0]), sen[1], sen[2], 0, 100);
        double error = heading - angleRead(YAW);
        error = wraps(error);

        double derivative = error - prev_error;
        double result = (error * kp) + (derivative * kd);
        prev_error = error;

        double left = power + result;
        double right = power - result;
        motorWrite(left, left, right, right);

        if (sensor >= 80) {
            motorStop();
            break;
        }
    }
}

void straight(int power, int timer) {
    double kp = 1.8; double kd = 10.0;

    unsigned long start = millis();
    double prev_error = 0.0;

    while (1) {
        double error = heading - angleRead(YAW);
        error = wraps(error);

        double derivative = error - prev_error;
        double result = (error * kp) + (derivative * kd);
        prev_error = error;

        double left = power + result;
        double right = power - result;
        motorWrite(left, left, right, right);

        if ((millis() - start) >= timer) {
            motorStop();
            break;
        }
    }
}

void setb(int power) {
    while (1) {
        int left_back = adcRead(rl[0]);
        int right_back = adcRead(rr[0]);

        if (left_back >= 650 && right_back >= 345) {
            motorStop();
            break;
        }
        if (right_back >= 345) {
            motorWrite(3, power);
            motorWrite(4, power);
        } else {
            motorWrite(3, -power);
            motorWrite(4, -power);
        }
        if (left_back >= 650) {
            motorWrite(1, power);
            motorWrite(2, power);
        } else {
            motorWrite(1, -power);
            motorWrite(2, -power);
        }
    }
    delay(100);
    heading = angleRead(YAW);
}

void setf(int power) {
    while (1) {
        int left_front = adcRead(fl[0]);
        int right_front = adcRead(fr[0]);

        if (left_front >= 535 && right_front >= 600) {
            motorStop();
            break;
        }
        if (left_front >= 535) {
            motorWrite(1, -power);
            motorWrite(2, -power);
        } else {
            motorWrite(1, power);
            motorWrite(2, power);
        }
        if (right_front >= 600) {
            motorWrite(3, -power);
            motorWrite(4, -power);
        } else {
            motorWrite(3, power);
            motorWrite(4, power);
        }
    }
    delay(100);
    heading = angleRead(YAW);
}

void wait() {
    motorStop();
    while (!switchRead(A) && !switchRead(B));
    delay(500);
}

int retrieveAprilTag() {
    delay(500);
    while (1) {
        std::vector<Block> blocks = husky.retrieve();

        if (!blocks.empty()) {
            lcdClear();
            lcdPrintText(0,0, String(blocks[0].id), 0xffff);
            lcdPrintText(20,0, String(blocks[0].x), 0xffff);
            lcdPrintText(50,0, String(blocks[0].y), 0xffff);

            return blocks[0].id;
        }

        delay(100);
    }
}

void blinkColor(int color) {
    servoWrite(6, 120); // 95 CLOSE // 120 OPEN
    servoWrite(4, 40); // 40 DOWN // 150 UP

    int now = millis();
    int i = 0;
    while (millis() - now <= 1000) {
        if (i) {
            for (int j = 0; j < 12; j++) {
                strip.setPixelColor(j, strip.Color(colorhash[color][0], colorhash[color][1], colorhash[color][2]));
            }
            i = 0;
        } else {
            strip.clear();
            i = 1;
        }
        
        strip.show();
        delay(250);
    }

    strip.clear();
    strip.show();

    straight(20, 200);
    servoWrite(6, 95);

    delay(250);

    servoWrite(4, 165);
    straight(-20, 300);
}

void gotoAprilTag() {
    setb(10);
    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);

    setf(10);
    straight(20, 210);

    turn(88);
    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 225);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);

    setf(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 150);
    straight(50, 100);
    straightUntil(20, fr);

    straight(20, 210);
    turn(89);

    straight(20, 100);
    straight(50, 100);
    straight(75, 200);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 50);
    turn(89);
}

void gotoCollectZone() {
    turn(88);

    straight(20, 100);
    straight(50, 100);
    straight(75, 175);
    straight(50, 100);
    straight(20, 100);

    setf(10);
    straight(20, 210);

    turn(-90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 225);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fl);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10); // at collect zone
}

void gotoHospital() {
    straight(-20, 100);
    straight(-50, 100);
    straight(-75, 25);
    straight(-50, 100);
    straight(-20, 100);
    setb(10);

    straight(-20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 225);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fr);

    straight(20, 210);
    turn(-91);

    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 210);
    turn(88);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);

    setf(10);
    straight(20, 210);

    turn(-91);
    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 250);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(88);

    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 210);
    turn(-90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fl);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(-91);
    
    straight(20, 400);
}

void returnHospital() {
    setb(10);

    straight(-20, 210);
    turn(-90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fr);

    straight(20, 210);
    turn(90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);

    setf(10);
    straight(20, 210);
    turn(-91);

    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(20);

    straight(20, 210);
    turn(89);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 225);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(88);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 225);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fl);

    straight(20, 210);
    turn(88);

    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 210);
    turn(-90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fl);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);

    setf(10); // at collect zone
}

void gotoShop() {
    straight(-20, 100);
    straight(-50, 100);
    straight(-75, 25);
    straight(-50, 100);
    straight(-20, 100);
    setb(10);

    straight(-20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 225);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fr);

    straight(20, 210);
    turn(-91);

    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 210);
    turn(89);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);

    setf(10);
    straight(20, 210);

    turn(-91);
    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 200);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(-90);

    straight(20, 400);
}

void returnShop() {
    straightUntilAny(-20, rl, rr);

    straight(-20, 210);
    turn(-90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 125);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(88);
    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 225);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fl);

    straight(20, 210);
    turn(88);

    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 210);
    turn(-90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fl);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);

    setf(10); // at collect zone
}

void gotoCharging() {
    straight(-20, 100);
    straight(-50, 100);
    straight(-75, 25);
    straight(-50, 100);
    straight(-20, 100);
    setb(10);

    straight(-20, 210);
    turn(89);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 275);
    turn(90);

    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 260);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 75);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(88);

    straight(20, 400);
}

void returnCharging() {
    straightUntil(-20, rr);

    straight(-20, 210);
    turn(88);

    straight(20, 100);
    straight(50, 100);
    straight(75, 25);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 275);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 125);
    straight(20, 100);
    setb(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 185);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fr);
    
    straight(20, 210);
    turn(90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);
}

void gotoSchool() {
    straight(-20, 100);
    straight(-50, 100);
    straight(-75, 25);
    straight(-50, 100);
    straight(-20, 100);
    setb(10);

    straight(-20, 210);
    turn(89);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 275);
    turn(90);

    straight(-20, 100);
    straight(-50, 90);
    straight(-20, 100);
    setf(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 260);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(88);

    straight(20, 100);
    straight(50, 100);
    straight(75, 75);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(88);

    straight(20, 400);
}

void returnSchool() {
    setb(10);

    straight(-20, 210);
    turn(88);

    straight(20, 100);
    straight(50, 100);
    straight(75, 75);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(20, 100);
    setb(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 185);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fr);
    
    straight(20, 210);
    turn(90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);
}

void gotoParkinglot() {
    setb(10);

    straight(-20, 210);
    turn(88);

    straight(20, 100);
    straight(50, 100);
    straight(75, 75);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(90, 200);
    straight(75, 100);
    straight(50, 100);
    straight(20, 100);
    setf(10);

    straight(20, 210);
    turn(-91);

    straight(20, 100);
    straight(50, 125);
    straight(20, 100);
    setb(10);

    straight(20, 100);
    straight(50, 100);
    straight(75, 100);
    straight(50, 100);
    straightUntil(20, fr);
    
    straight(20, 210);
    turn(90);

    straight(20, 100);
    straight(50, 100);
    straight(75, 200);
    straight(50, 100);
    straight(20, 100);
    setf(10);
}

void drop(int depth) {
    servoWrite(4, 40); // 40 DOWN // 150 UP
    delay(300);
    servoWrite(6, 120); // 95 CLOSE // 120 OPEN

    straight(20, depth);
    straight(-20, depth);

    servoWrite(4, 175); // 40 DOWN // 150 UP
    servoWrite(6, 95); // 95 CLOSE // 120 OPEN
}

void setup() {
    Serial.begin(115200);
    initialize();
    //calibrateIMU();
    lcdClear();

    lcdPrintText(0,0, "connecting huskylens", 0xffff);
    husky.begin();
    while (!husky.connected()) {
        delay(200);
    }
    lcdClear();
    lcdPrintText(0,0, "connected", 0xffff);
    husky.algorithm(ALGORITHM_TAG_RECOGNITION);

    servoWrite(6, 95); // 95 CLOSE // 120 OPEN
    servoWrite(4, 165); // 40 DOWN // 150 UP

    strip.begin();
    strip.setBrightness(75);
    strip.clear();
    strip.show();
}

void loop() {
    wait();
    setAngleOffset();
    heading = 0.0;

    gotoAprilTag();

    apriltag = retrieveAprilTag();

    gotoCollectZone();

    for (int i = 0; i <= 1; i++) {
        
        blinkColor(aprilhash[apriltag][iteration++]);

        gotoHospital();
        drop(depth[i]);
        returnHospital();

        blinkColor(aprilhash[apriltag][iteration++]);

        gotoShop();
        drop(depth[i]);
        returnShop();

        blinkColor(aprilhash[apriltag][iteration++]);

        gotoCharging();
        drop(depth[i]);
        returnCharging();

        blinkColor(aprilhash[apriltag][iteration]);

        gotoSchool();
        drop(depth[i]);
        if (!i) returnSchool();
        else gotoParkinglot();

        iteration = 0;
    }
}
