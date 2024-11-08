package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="ViperArrrrrm", group="Linear OpMode")
public class ViperArrrrrm {
    private DcMotor viperArm = null;
    private TouchSensor touchSensor;

    double adjustControllerSensitivity(double input){
        return input * Math.abs(input);
    }

    void initialize(HardwareMap hwm){
        viperArm = hwm.get(DcMotor.class, Constants.VIPER_ARM);
        viperArm.setDirection(DcMotor.Direction.REVERSE);
        viperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        touchSensor = hwm.get(TouchSensor.class, Constants.TOUCH_SENSOR);
    }
    void operateArm(Telemetry telemetry, double armPower){
        double max = 1;
        max = Math.max(max, Math.abs(armPower));
        if (armPower > max) {
            armPower = max;
        }

        if (touchSensor.isPressed() && armPower < 0) {
            armPower = 0;
            telemetry.addData("Deb", "Pressed");
        } else {
            telemetry.addData("Deb", "Not Pressed");
        }
        viperArm.setPower(armPower);
        telemetry.addData("arm power", "%4.2f", armPower);
        telemetry.update();
    }
}