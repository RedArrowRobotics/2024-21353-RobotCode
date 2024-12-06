package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class L3D {
    LED ledR;
    LED ledG;
    int loops = 0;

    void initialize(HardwareMap hwm) {
        ledR = hwm.get(LED.class, Constants.LED_R);
        ledG = hwm.get(LED.class, Constants.LED_G);
    }

    void red() {
        ledR.on();
        ledG.off();
    }

    void green() {
        ledR.off();
        ledG.on();
    }

    void off() {
        ledR.off();
        ledG.off();
    }


    void pattern() {
        loops = loops + 1;
        if (loops < 1) {
            red();
        }
        if (loops >= 1 && loops < 2) {
            off();
        }
        if (loops >= 2 && loops < 3) {
            red();
        }
        if (loops >= 3 && loops < 4) {
            off();
        }
        if (loops >= 4 && loops < 5) {
            red();
        }
        if (loops >= 5 && loops < 6) {
            off();
        }
        if (loops >= 7 && loops < 8) {
            green();
        }
        if (loops >= 8 && loops < 9) {
            off();
        }
        if (loops >= 9 && loops < 10) {
            green();
        }
        if (loops >= 10 && loops < 11) {
            off();
        }
        if (loops >= 11 && loops < 12) {
            green();
        }
        if (loops >= 12 && loops < 13) {
            off();
        }
        if (loops >= 13) {
            loops = 0;
        }
    }
}