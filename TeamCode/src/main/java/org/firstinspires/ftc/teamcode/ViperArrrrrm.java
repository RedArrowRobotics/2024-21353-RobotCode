package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ViperArrrrrm", group="Linear OpMode")
public class ViperArrrrrm extends LinearOpMode {
    private DcMotor viperArm = null;

    public ViperArrrrrm() {
    }
    double adjustControllerSensitivity(double input){
        return input * Math.abs(input);
    }

    @Override
    public void runOpMode() {
        viperArm = hardwareMap.get(DcMotor.class, Constants.VIPER_ARM);
        waitForStart();
        viperArm.setDirection(DcMotor.Direction.FORWARD);
        while (opModeIsActive()) {
            double power = adjustControllerSensitivity(-gamepad2.left_stick_y);
            double max = 1;
            max = Math.max(max, Math.abs(power));
            if (power > max) {
                power = max;
            }
        viperArm.setPower(power);
        telemetry.addData("arm power", "%4.2f", power);
        }
    }
}
