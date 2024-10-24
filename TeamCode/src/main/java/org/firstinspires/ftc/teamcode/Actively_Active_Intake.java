package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="BensonIntake", group="Linear OpMode")
public class Actively_Active_Intake extends LinearOpMode {

    Servo rotate;
    CRServo spin;

    void initialize(HardwareMap hwm){
        rotate = hwm.get(Servo.class, Constants.BENSON_INTAKE_ROTATE);
        rotate.setDirection(Servo.Direction.FORWARD);
        spin = hwm.get(CRServo.class, Constants.BENSON_INTAKE_SPIN);
        spin.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    void rotateOut(Telemetry telemetry){
        rotate.setPosition(135);
        telemetry.addData("Intake Position", "%4.2f", rotate.getPosition());
    }
    void rotateIn(Telemetry telemetry){
        rotate.setPosition(90);
        telemetry.addData("Intake Position", "%4.2f", rotate.getPosition());
    }
    void startSpin(Telemetry telemetry){
        spin.setPower(1);
        telemetry.addData("Intake Power", "%4.2f", spin.getPower());
    }
    void stopSpin(Telemetry telemetry){
        spin.setPower(0);
        telemetry.addData("Intake Power", "%4.2f", spin.getPower());
    }
    public void runOpMode() {
        initialize(hardwareMap);
        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                rotateOut(telemetry);
                startSpin(telemetry);
            }
            if (gamepad2.right_bumper) {
                rotateIn(telemetry);
                stopSpin(telemetry);
            }
            telemetry.addData("Intake Position", "%4.2f", rotate.getPosition());
            telemetry.addData("Intake Power", "%4.2f", spin.getPower());
            telemetry.update();
        }
    }
}