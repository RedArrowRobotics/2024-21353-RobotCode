package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class L3D {
    LED ledR;
    LED ledG;
    

    void initialize(HardwareMap hwm){
        ledR = hwm.get(LED.class, Constants.LED_R);
        ledG = hwm.get(LED.class, Constants.LED_G);
    }
    void red(){
        ledR.on();
        ledG.off();
    }
    void green(){
        ledR.off();
        ledG.on();
    }
    void off(){
        ledR.off();
        ledG.off();
    }

    void blinkR(){
        red();
        off();
        red();
        off();
        red();
        off();
        red();
        off();
    }
    void blinkG(){
        green();
        off();
        green();
        off();
        green();
        off();
        green();
        off();
    }
    void alternate(){
        blinkR();
        blinkG();
    }
}