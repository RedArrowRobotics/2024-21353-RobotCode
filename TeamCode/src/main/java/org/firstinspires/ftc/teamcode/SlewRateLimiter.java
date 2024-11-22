package org.firstinspires.ftc.teamcode;

public class SlewRateLimiter {
    private double slewRate;
    private double lastOutput;

    public SlewRateLimiter(double slewRate) {
        this.slewRate = slewRate;
        this.lastOutput = 0.0;
    }

    public double limit(double input) {
        if (input > lastOutput + slewRate) {
            lastOutput =  lastOutput + slewRate;
        } else if (input < lastOutput - slewRate) {
            lastOutput = lastOutput - slewRate;
        } else {
            lastOutput = input;
        }
        return lastOutput;
    }

}
