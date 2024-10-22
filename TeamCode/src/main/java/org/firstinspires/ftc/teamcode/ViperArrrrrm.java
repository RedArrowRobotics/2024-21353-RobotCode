package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ViperArrrrrm", group="Linear OpMode")
public class ViperArrrrrm extends LinearOpMode {
    private DcMotor viperArm = null;

    double adjustControllerSensitivity(double input){
        return input * Math.abs(input);
    }

    Servo trainSlide;

    void initialize(HardwareMap hwm){
        viperArm = hwm.get(DcMotor.class, Constants.VIPER_ARM);
        viperArm.setDirection(DcMotor.Direction.FORWARD);
    }
    void operateArm(double armPower){
        double max = 1;
        max = Math.max(max, Math.abs(armPower));
        if (armPower > max) {
            armPower = max;
        }
        viperArm.setPower(armPower);
        telemetry.addData("arm power", "%4.2f", armPower);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double armPower = adjustControllerSensitivity(-gamepad2.left_stick_y);

            operateArm(armPower);
        }
    }
}
