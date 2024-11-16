package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Train", group="Linear OpMode")
public class Train extends LinearOpMode {

    Servo trainSlide;

    void initialize(HardwareMap hwm){
        trainSlide = hwm.get(Servo.class, Constants.TRAIN_SLIDE);
        trainSlide.setDirection(Servo.Direction.FORWARD);
    }
    void extend(){
        //trainSlide.setPosition(trainSlide.getPosition()+0.4);
        trainSlide.setPosition(.8);
    }
    void retract(){
        //trainSlide.setPosition(trainSlide.getPosition()-0.4);
        trainSlide.setPosition(.45);
    }

    public void runOpMode() {
        initialize(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.b) {
                extend();
                telemetry.addData("Drawer Power", "%4.2f", trainSlide.getPosition());
            }
            if (gamepad2.a) {
                retract();
                telemetry.addData("Drawer Power", "%4.2f", trainSlide.getPosition());
            }
            telemetry.addData("Drawer Power", "%4.2f", trainSlide.getPosition());
            telemetry.update();
        }
    }
}