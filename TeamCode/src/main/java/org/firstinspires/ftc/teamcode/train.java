package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="train", group="Linear OpMode")
public class train extends LinearOpMode {
    private Servo trainSlide = null;
    public void runOpMode() {
        trainSlide = hardwareMap.get(Servo.class, Constants.TRAIN_SLIDE);
        waitForStart();
        trainSlide.setDirection(Servo.Direction.FORWARD);
        while (opModeIsActive()) {
            if (gamepad2.b) {
                trainSlide.setPosition(0);
            }
            if (gamepad2.a) {
                trainSlide.setPosition(135);
            }
            telemetry.addData("train power", "%4.2f", trainSlide.getPosition());
        }
    }
}
