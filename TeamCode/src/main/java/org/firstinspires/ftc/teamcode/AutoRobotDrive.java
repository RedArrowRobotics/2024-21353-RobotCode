/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.BL;
import static org.firstinspires.ftc.teamcode.Constants.BR;
import static org.firstinspires.ftc.teamcode.Constants.FL;
import static org.firstinspires.ftc.teamcode.Constants.FR;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "Full Autonomous", group = "Robot")
//@Disabled
public class AutoRobotDrive extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    
    private ViperArrrrrm viperArm = new ViperArrrrrm();
    private SampleBucket bucket = new SampleBucket();

    private IMU imu = null;

    private ElapsedTime runtime = new ElapsedTime();

    private L3D led = new L3D();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double MINIMUM_TURN_SPEED = 0.1;
    private void imuTurn(double angle) {
        imu.resetYaw();
        while (Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) < Math.abs(angle)) {
            double power = (-(TURN_SPEED-MINIMUM_TURN_SPEED)/angle*(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))+TURN_SPEED);
            if(angle > 0) {
                leftBackDrive.setPower(power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(-power);
                rightFrontDrive.setPower(-power);
            } else {
                leftBackDrive.setPower(-power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(power);
                rightFrontDrive.setPower(power);
            }
            telemetry.addData("Degrees", "%.3f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void imuDepends(double angle) {
        imu.resetYaw();
        while (opModeIsActive() && Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) < Math.abs(angle)) {
            double currentImu = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double power = 0;
            if(Math.abs(currentImu) < Math.abs(angle)-45) {
                power = TURN_SPEED;
            } else {
                power = (-0.4/45*(Math.abs(currentImu)-(Math.abs(angle)-45))+0.5);
            }
            telemetry.addData("Power", power);
            telemetry.addData("Degrees", angle);

            if(angle > 0) {
                leftBackDrive.setPower(power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(-power);
                rightFrontDrive.setPower(-power);
            } else {
                leftBackDrive.setPower(-power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(power);
                rightFrontDrive.setPower(power);
            }
            telemetry.addData("Degrees", "%.3f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    void pause(double seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) { }
    }
    double speed = FORWARD_SPEED;
    void forward(double speed, int ticks) {
        int initialPosition = leftFrontDrive.getCurrentPosition();
        if (ticks > 0) {
            while (leftFrontDrive.getCurrentPosition() < initialPosition + ticks) {
                leftBackDrive.setPower(speed);
                leftFrontDrive.setPower(speed);
                rightBackDrive.setPower(speed);
                rightFrontDrive.setPower(speed);
            }
        }else{
            while (leftFrontDrive.getCurrentPosition() > initialPosition + ticks) {
                leftBackDrive.setPower(speed);
                leftFrontDrive.setPower(speed);
                rightBackDrive.setPower(speed);
                rightFrontDrive.setPower(speed);
            }
        }
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
    void strafeRight(double speed, int ticks) {
        int initialPosition = leftFrontDrive.getCurrentPosition();
        if (ticks > 0) {
            while (leftFrontDrive.getCurrentPosition() < initialPosition + ticks) {
                leftBackDrive.setPower(-speed);
                leftFrontDrive.setPower(speed);
                rightBackDrive.setPower(speed);
                rightFrontDrive.setPower(-speed);
            }
        }else{
            while (leftFrontDrive.getCurrentPosition() > initialPosition + ticks) {
                leftBackDrive.setPower(-speed);
                leftFrontDrive.setPower(speed);
                rightBackDrive.setPower(speed);
                rightFrontDrive.setPower(-speed);
            }
        }
    }

    @Override
    public void runOpMode() {

            // Initialize the drive system variables.
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

            viperArm.initialize(hardwareMap);
            bucket.initialize(hardwareMap);
            led.initialize(hardwareMap);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(orientationOnRobot));


            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Send telemetry message to signify robot waiting;
            imu.resetYaw();
            runtime.reset();
            telemetry.addData("Status", "Ready to run");    //
            telemetry.addData("Degrees", "%.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            //537.6 ticks per revolution
            bucket.reset();
            //forward(-FORWARD_SPEED, -537);
            int initialPosition = leftFrontDrive.getCurrentPosition();
            while (leftBackDrive.getCurrentPosition() > initialPosition - 269) { //0.5 revolutions
            leftBackDrive.setPower(-0.6);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(-0.6);
            }
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            viperArm.highBucket();
            while (opModeIsActive() && viperArm.isMoving2()) { }
            bucket.dump();
            pause(1);
            bucket.reset();
            viperArm.home();
            while (opModeIsActive() && viperArm.isMoving2()) { }
            initialPosition = leftFrontDrive.getCurrentPosition();
            while (leftBackDrive.getCurrentPosition() < initialPosition + 5370) { //5 revolutions
                leftBackDrive.setPower(FORWARD_SPEED);
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(FORWARD_SPEED);
            }

            // Step 4:  Stop
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);

            led.pattern();

            telemetry.addData("Path", "Complete");
            telemetry.update();
    }
}