package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="Train", group="Linear OpMode")
public class Train extends LinearOpMode {

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

    public void runOpMode() {
        initialize(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.b) {
                extend();
                telemetry.addData("Drawer Power", "%4.2f", trainSlide.getCurrentPosition());
            }
            if (gamepad2.a) {
                retract();
                telemetry.addData("Drawer Power", "%4.2f", trainSlide.getCurrentPosition());
            }
            telemetry.addData("Drawer Power", "%4.2f", trainSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}