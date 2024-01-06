package org.firstinspires.ftc.teamcode.boltbusterz;

import java.util.Arrays;

public class MecanumDrive {
    public double throttle;
    public MecanumDrive(double throttle){
        this.throttle = throttle;
    }
    public double[] CalculatePower(double rightStrafe, double leftStrafe, double leftTank, double rightTank){
        double[] inputs = {throttle * rightStrafe, throttle * leftStrafe, throttle * leftTank, throttle * rightTank, 1};
        double normalizer = Arrays.stream(inputs).max().getAsDouble();
        double frPower = (rightTank - rightStrafe + leftStrafe) * throttle / normalizer;
        double brPower = (rightTank + rightStrafe - leftStrafe) * throttle / normalizer;
        double blPower = (leftTank + leftStrafe - rightStrafe) * throttle / normalizer;
        double flPower = (leftTank - leftStrafe + rightStrafe) * throttle / normalizer;
        return new double[]{frPower, brPower, blPower, flPower};
    }
}
