package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SampleBucket", group="Linear OpMode")
public class SampleBucket extends LinearOpMode {
    Servo bucket;

    void initialize(HardwareMap hwm){
         bucket = hwm.get(Servo.class, Constants.SAMPLE_BUCKET);
         bucket.setDirection(Servo.Direction.REVERSE);
    }
    void dump(){
        bucket.setPosition(135);
    }
    void reset(){
        bucket.setPosition(90);
    }

    public void runOpMode() {
        initialize(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.y) {
                dump();
                telemetry.addData("train power", "%4.2f", bucket.getPosition());
            }
            if (gamepad2.x) {
                reset();
                telemetry.addData("train power", "%4.2f", bucket.getPosition());
            }
            telemetry.addData("train power", "%4.2f", bucket.getPosition());
            telemetry.update();
        }
    }
}
