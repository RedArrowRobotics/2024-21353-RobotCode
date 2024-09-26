package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class train {
    private DcMotor trainSlide = null;

    public train() {
        trainSlide = hardwareMap.get(DcMotor .class, Constants.VIPER_ARM);
    }
    double adjustControllerSensitivity(double input){
        return input * Math.abs(input);
    }
    public void runOpMode() {
        trainSlide.setDirection(DcMotor.Direction.FORWARD);
        double power = adjustControllerSensitivity(-gamepad2.left_stick_y);
        double max = 1;
        max = Math.max(max, Math.abs(power));
        if (power > 1) {
            power = max;
            trainSlide.setPower(power);
            telemetry.addData("train power", "%4.2f", power);
        }
    }
}
