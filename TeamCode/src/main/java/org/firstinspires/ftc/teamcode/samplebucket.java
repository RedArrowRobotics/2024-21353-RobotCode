package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="samplebucket", group="Linear OpMode")
public class samplebucket extends LinearOpMode {
    private Servo bucket = null;

    public void runOpMode() {
        bucket = hardwareMap.get(Servo.class, Constants.SAMPLE_BUCKET);
        waitForStart();
        bucket.setDirection(Servo.Direction.REVERSE);
        while (opModeIsActive()) {
            if (gamepad2.y) {
                bucket.setPosition(135);
            }
            if (gamepad2.x) {
                bucket.setPosition(90);
            }
            telemetry.addData("train power", "%4.2f", bucket.getPosition());
        }
    }
}
