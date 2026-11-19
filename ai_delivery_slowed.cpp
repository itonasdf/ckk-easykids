#include <PTBOTAtomVX.h>

#define RED 0
#define GREEN 1
#define YELLOW 2
#define BLUE 3

int fl[3] = { 1, 150, 850 };
int fr[3] = { 3, 200, 890 };
int rl[3] = { 7, 200, 890 };
int rr[3] = { 5, 100, 500 };

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

double wraps(double val) {
    while (val > 180) val -= 360;
    while (val < -180) val += 360;
    return val;
}

void turn(double relative) {
    double kp = 1.5; double kd = 2.5;

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
            double power = copysign(10, error);
            motorWrite(power, power, -power, -power);
        } else {
            motorWrite(result, result, -result, -result);
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
        int left_back = map(adcRead(rl[0]), rl[1], rl[2], 0, 100);
        int right_back = map(adcRead(rr[0]), rr[1], rr[2], 0, 100);

        if (left_back >= 80 && right_back >= 80) {
            motorStop();
            break;
        }
        if (right_back >= 80) {
            motorWrite(3, power);
            motorWrite(4, power);
        } else {
            motorWrite(3, -power);
            motorWrite(4, -power);
        }
        if (left_back >= 80) {
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
        int left_front = map(adcRead(fl[0]), fl[1], fl[2], 0, 100);
        int right_front = map(adcRead(fr[0]), fr[1], fr[2], 0, 100);

        if (left_front >= 80 && right_front >= 80) {
            motorStop();
            break;
        }
        if (left_front >= 80) {
            motorWrite(1, -power);
            motorWrite(2, -power);
        } else {
            motorWrite(1, power);
            motorWrite(2, power);
        }
        if (right_front >= 80) {
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
    delay(1000);
    return 0;
}

void blinkColor(int color) {
    delay(1000);
}




void gotoAprilTag() {
    setb(20);
    straight(20, 100);
    straight(50, 400);
    straight(20, 100);

    setf(20);
    straight(20, 250);
    turn(89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    setf(20); // at 1st intersection

    straight(20, 100);
    straight(50, 400);
    straightUntil(20, fr);
    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 450);
    straight(20, 100);
    setf(20);
}

void gotoCollectZone() {
    straight(-20, 100);
    straight(-50, 350);
    straight(-20, 100);
    setb(20);

    straight(-20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20); // at 1st intersection

    straight(20, 100);
    straight(50, 400);
    straight(20, 100);
    straightUntil(20, fl);
    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 350);
    straight(20, 100);
    setf(20); // at collect zone
}

void gotoHospital() {
    straight(-20, 100);
    straight(-50, 300);
    straight(-20, 100);
    setb(20);

    straight(-20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    straightUntil(20, fr);

    straight(20, 200);
    turn(-89);

    straight(-20, 100);
    straight(-50, 100);
    straight(-20, 100);
    setf(20);

    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 875);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);
    straight(20, 100);
    straight(50, 100);
    straight(20, 100);
    setb(20);

    straight(20, 100);
    straight(50, 800);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);
    straight(20, 100);
    straight(50, 100);
    straight(20, 100);
    setb(20);

    straight(-20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    straightUntil(20, fl);
    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 350);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);
    
    straight(20, 500);
}

void returnHospital() {
    setb(20);

    straight(-20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 350);
    straight(20, 100);
    straightUntil(20, fr);

    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);

    setf(20);
    straight(20, 200);
    turn(-89);

    straight(-20, 100);
    straight(-50, 100);
    straight(-20, 100);
    setf(20);

    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    straightUntil(20, fl);

    straight(20, 200);
    turn(89);

    straight(-20, 100);
    straight(-50, 100);
    straight(-20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20); // at 1st intersection

    straight(20, 100);
    straight(50, 400);
    straight(20, 100);
    straightUntil(20, fl);
    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 350);
    straight(20, 100);
    setf(20); // at collect zone
}

void gotoShop() {
    straight(-20, 100);
    straight(-50, 300);
    straight(-20, 100);
    setb(20);

    straight(-20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    straightUntil(20, fr);

    straight(20, 200);
    turn(-89);

    straight(-20, 100);
    straight(-50, 100);
    straight(-20, 100);
    setf(20);

    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 875);
    straight(20, 100);
    setf(20);

    straight(20, 200);

    turn(-89);
    straight(20, 100);
    straight(50, 100);
    straight(20, 100);
    setb(20);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);

    straight(20, 500);
}

void returnShop() {
    straightUntil(-20, rl);

    straight(-20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 350);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    straightUntil(20, fl);

    straight(20, 200);
    turn(89);

    straight(-20, 100);
    straight(-50, 100);
    straight(-20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20); // at 1st intersection

    straight(20, 100);
    straight(50, 400);
    straight(20, 100);
    straightUntil(20, fl);
    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 350);
    straight(20, 100);
    setf(20); // at collect zone
}

void gotoCharging() {
    straight(-20, 100);
    straight(-50, 300);
    straight(-20, 100);
    setb(20);

    straight(-20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    setf(20);

    straight(20, 250);
    turn(89);

    straight(20, 100);
    straight(50, 100);
    straight(20, 100);
    setb(20);

    straight(20, 100);
    straight(50, 750);
    straight(20, 100);
    setf(20);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(89);

    straight(20, 500);
}

void returnCharging() {
    straightUntil(-20, rr);

    straight(-20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 200);
    straight(20, 100);
    setf(20);

    straight(20, 100);
    straight(50, 950);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 150);
    straight(20, 100);
    setb(20);

    straight(20, 100);
    straight(50, 800);
    straight(20, 100);
    straightUntil(20, fr);
    
    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20);
}

void gotoSchool() {
    straight(-20, 100);
    straight(-50, 300);
    straight(-20, 100);
    setb(20);

    straight(-20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    setf(20);

    straight(20, 250);
    turn(89);

    straight(20, 100);
    straight(50, 100);
    straight(20, 100);
    setb(20);

    straight(20, 100);
    straight(50, 750);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(89);

    straight(20, 500);
}

void returnSchool() {
    setb(20);

    straight(-20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-91);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 100);
    straight(20, 100);
    setb(20);

    straight(20, 100);
    straight(50, 800);
    straight(20, 100);
    straightUntil(20, fr);
    
    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20);
}

void gotoParkinglot() {
    setb(20);

    straight(-20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-91);

    straight(20, 100);
    straight(50, 850);
    straight(20, 100);
    setf(20);

    straight(20, 200);
    turn(-89);

    straight(20, 100);
    straight(50, 100);
    straight(20, 100);
    setb(20);

    straight(20, 100);
    straight(50, 300);
    straight(20, 100);
    straightUntil(20, fr);
    
    straight(20, 200);
    turn(89);

    straight(20, 100);
    straight(50, 400);
    straight(20, 100);
    setf(20);
}

void drop() {
    delay(1000);
}

void setup() {
    Serial.begin(115200);
    initialize();
    //calibrateIMU();
    lcdClear();
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
        drop();
        returnHospital();

        blinkColor(aprilhash[apriltag][iteration++]);

        gotoShop();
        drop();
        returnShop();

        blinkColor(aprilhash[apriltag][iteration++]);

        gotoCharging();
        drop();
        returnCharging();

        blinkColor(aprilhash[apriltag][iteration]);

        gotoSchool();
        drop();
        if (!i) returnSchool();
        else gotoParkinglot();

        iteration = 0;
    }
}
