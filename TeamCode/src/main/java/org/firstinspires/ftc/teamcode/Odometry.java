package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Odometry {
    GoBildaPinpointDriver odo;

    void initialize(HardwareMap hwm) {
        odo = hwm.get(GoBildaPinpointDriver.class, Constants.ODO);
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        // TODO: Need to measure these based on our robot. See page 2 and 3 of https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf
        odo.setOffsets(-84.0, -168.0);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public void logInitializationData(Telemetry telemetry) {
        telemetry.addData("Odometry Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
    }

    /**
     * @return the raw value of the X (forward) encoder in ticks
     */
    public int getEncoderX() {
        return odo.getEncoderX();
    }

    /**
     * @return the raw value of the Y (strafe) encoder in ticks
     */
    public int getEncoderY() {
        return odo.getEncoderY();
    }

    /**
     * @return the estimated X (forward) position of the robot in mm
     */
    public double getPosX() {
        return odo.getPosX();
    }

    /**
     * @return the estimated Y (Strafe) position of the robot in mm
     */
    public double getPosY() {
        return odo.getPosY();
    }

    /**
     * @return the estimated H (heading) position of the robot in Radians
     */
    public double getHeading() {
        return odo.getHeading();
    }

    /**
     * @return the estimated X (forward) velocity of the robot in mm/sec
     */
    public double getVelX() {
        return odo.getVelX();
    }

    /**
     * @return the estimated Y (strafe) velocity of the robot in mm/sec
     */
    public double getVelY() {
        return odo.getVelY();
    }

    /**
     * @return the estimated H (heading) velocity of the robot in radians/sec
     */
    public double getHeadingVelocity() {
        return odo.getHeadingVelocity();
    }

    /**
     * @return a Pose2D containing the estimated position of the robot
     */
    public Pose2D getPosition() {
        return odo.getPosition();
    }

    /**
     * @return a Pose2D containing the estimated velocity of the robot, velocity is unit per second
     */
    public Pose2D getVelocity() {
        return odo.getVelocity();
    }
}
