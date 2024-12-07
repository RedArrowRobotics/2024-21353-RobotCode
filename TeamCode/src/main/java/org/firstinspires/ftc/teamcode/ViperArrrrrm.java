package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ViperArrrrrm {
    private DcMotor viperArm = null;
    private TouchSensor touchSensor;

    void initialize(HardwareMap hwm){
        viperArm = hwm.get(DcMotor.class, Constants.VIPER_ARM);
        viperArm.setDirection(DcMotor.Direction.REVERSE);
        viperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperArm.setTargetPosition(0);
        viperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        touchSensor = hwm.get(TouchSensor.class, Constants.TOUCH_SENSOR);
    }

    void home(){
        viperArm.setPower(1);
        viperArm.setTargetPosition(0);
    }
    void highBucket(){
        viperArm.setPower(1);
        viperArm.setTargetPosition(3100);
    }
    void lowBucket(){
        viperArm.setPower(1);
        viperArm.setTargetPosition(1800);
    }
    void bar(){
        viperArm.setPower(1);
        viperArm.setTargetPosition(1200); //not calibrated
    }

    boolean isMoving2(){
        return viperArm.isBusy();
    }

    void operateArm(Telemetry telemetry, double armPower){

        if (viperArm.getTargetPosition() <= 0) {
            if (touchSensor.getValue() == 1) {
                viperArm.setTargetPosition(0);
                viperArm.setPower(0);
                viperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                viperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperArm.setPower(1);
            } else if (touchSensor.getValue() == 0 && viperArm.getCurrentPosition() <= 0) {
                viperArm.setTargetPosition(viperArm.getCurrentPosition() - 100);
                viperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                viperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperArm.setPower(1);
            }
        }

        telemetry.addData("button push", "%4.2f", touchSensor.getValue());
        telemetry.addData("arm power", "%4.2f", armPower);
        telemetry.addData("arm position (Ticks)", viperArm.getCurrentPosition());
        telemetry.addData("arm destination (Ticks)", viperArm.getTargetPosition());
    }
}