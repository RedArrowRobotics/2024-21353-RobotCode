package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Train {

    DcMotor trainSlide;

    void initialize(HardwareMap hwm){
        trainSlide = hwm.get(DcMotor.class, Constants.TRAIN_SLIDE);
        trainSlide.setDirection(DcMotor.Direction.FORWARD);
        trainSlide.setTargetPosition(0);
        trainSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trainSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void extend(){
        trainSlide.setPower(.5);
        trainSlide.setTargetPosition(100);
    }
    void retract(){
        trainSlide.setPower(.5);
        trainSlide.setTargetPosition(0);
    }
}