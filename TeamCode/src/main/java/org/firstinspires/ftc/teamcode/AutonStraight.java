package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.BL;
import static org.firstinspires.ftc.teamcode.Constants.BR;
import static org.firstinspires.ftc.teamcode.Constants.FL;
import static org.firstinspires.ftc.teamcode.Constants.FR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auton Straight", group = "Robot")
public class AutonStraight {
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;

    void forward(int ticks) {
        int initialPosition = leftFrontDrive.getCurrentPosition();
        if (ticks > 0) {
            while (leftFrontDrive.getCurrentPosition() < initialPosition + ticks) {
                leftBackDrive.setPower(1);
                leftFrontDrive.setPower(1);
                rightBackDrive.setPower(1);
                rightFrontDrive.setPower(1);
            }
        }
    }

    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, FL);
        leftBackDrive = hardwareMap.get(DcMotor.class, BL);
        rightFrontDrive = hardwareMap.get(DcMotor.class, FR);
        rightBackDrive = hardwareMap.get(DcMotor.class, BR);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        forward(2150); //4 revolutions
    }
}
