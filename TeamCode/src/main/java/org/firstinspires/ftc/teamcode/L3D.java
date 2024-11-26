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
        if (loops < 250) {
            red();
        }
        if (loops >= 250 && loops < 500) {
            off();
        }
        if (loops >= 500 && loops < 750) {
            red();
        }
        if (loops >= 750 && loops < 1000) {
            off();
        }
        if (loops >= 1000 && loops < 1250) {
            red();
        }
        if (loops >= 1250 && loops < 1500) {
            off();
        }
        if (loops >= 1500 && loops < 1750) {
            green();
        }
        if (loops >= 1750 && loops < 2000) {
            off();
        }
        if (loops >= 2000 && loops < 2250) {
            green();
        }
        if (loops >= 2250 && loops < 2500) {
            off();
        }
        if (loops >= 1000 && loops < 1250) {
            green();
        }
        if (loops >= 1250 && loops < 1500) {
            off();
        }
        if (loops >= 1500) {
            loops = 0;
        }
    }
}