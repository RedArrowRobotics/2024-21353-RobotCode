/* Copyright (c) 2021 FIRST. All rights reserved.
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
import static org.firstinspires.ftc.teamcode.Constants.VELOCITY_SCALE_FACTOR;
import static org.firstinspires.ftc.teamcode.Constants.TOUCH_SENSOR;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Full Teleop", group="Linear OpMode")
public class FullTeleOP extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private IMU imu = null;
    private SampleBucket bucket = null;
    private Train trainSlide = null;
    private ViperArrrrrm viperArm = null;
    private Actively_Active_Intake intake = null;
    private SlewRateLimiter slewStraight = new SlewRateLimiter(0.1);
    private SlewRateLimiter slewStrafe = new SlewRateLimiter(0.1);
    private SlewRateLimiter slewRotate = new SlewRateLimiter(0.1);

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    double adjustControllerSensitivity(double input){
        if(input > 0.01) {
            input = input - 0.01;
        }else if(input < -0.01) {
            input = input + 0.01;
        }
        double speed = input * Math.abs(input);
        return speed;
    }

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Starting Initialization");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, FL);
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, BL);
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, FR);
        rightBackDrive = hardwareMap.get(DcMotorEx.class, BR);

        telemetry.addData("Status", "Finished getting drive motors..");
        telemetry.update();

        bucket = new SampleBucket();
        viperArm = new ViperArrrrrm();
        trainSlide = new Train();
        intake = new Actively_Active_Intake();

        telemetry.addData("Status", "Finished Creating Robot Components..");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        bucket.initialize(hardwareMap);
        viperArm.initialize(hardwareMap);
        trainSlide.initialize(hardwareMap);
        intake.initialize(hardwareMap);

        telemetry.addData("Status", "Finished Initializing Robot Components....");
        telemetry.update();

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double maxPower = .65;
            double axial   = adjustControllerSensitivity(-gamepad1.left_stick_y) * maxPower;  // Note: pushing stick forward gives negative value
            double lateral =  adjustControllerSensitivity(gamepad1.left_stick_x) * maxPower * 1.5;
            double yaw     =  adjustControllerSensitivity(-gamepad1.right_stick_x) * maxPower;

            axial = slewStraight.limit(axial);
            lateral = slewStrafe.limit(lateral);
            yaw = slewRotate.limit(yaw);

            telemetry.addData("axialPower", axial);
            telemetry.addData("lateralPower", lateral);
            telemetry.addData("yawPower", yaw);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral - yaw;
            double rightFrontPower = axial - lateral + yaw;
            double leftBackPower   = axial - lateral - yaw;
            double rightBackPower  = axial + lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setVelocity(leftFrontPower * VELOCITY_SCALE_FACTOR);
            rightFrontDrive.setVelocity(rightFrontPower * VELOCITY_SCALE_FACTOR);
            leftBackDrive.setVelocity(leftBackPower * VELOCITY_SCALE_FACTOR);
            rightBackDrive.setVelocity(rightBackPower * VELOCITY_SCALE_FACTOR);
            //make the power slope more shallow so its easier to go slower
            //strafing is wonky because the weight distribution is all on the back wheels

            //Bucket
            if (gamepad2.y) {
                bucket.dump();
            } else if (!gamepad2.x) {
                bucket.reset();
            }
            telemetry.addData("Bucket Servo", "%4.2f", bucket.getPosition());

            //Viper Arm
            double armPower = adjustControllerSensitivity(-gamepad2.left_stick_y);
            viperArm.operateArm(telemetry, armPower);

            if (gamepad2.a) {
                viperArm.home();
                telemetry.addData("Deb", "Home");
            } else if (gamepad2.b) {
                viperArm.highBucket();
                telemetry.addData("Deb", "High Bucket");
            } if (gamepad2.x) {
                viperArm.lowBucket();
                telemetry.addData("Deb", "Low Bucket");
            }

            //Benson and Train
            if (gamepad2.left_bumper) {
                trainSlide.extend();
                intake.rotateOut(telemetry);
                intake.startSpin(telemetry);
            }
            if (gamepad2.right_bumper) {
                intake.stopSpin(telemetry);
                intake.rotateIn(telemetry);
                trainSlide.retract();
            }
            if (gamepad2.dpad_right) {
                intake.reverseSpin(telemetry);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}

