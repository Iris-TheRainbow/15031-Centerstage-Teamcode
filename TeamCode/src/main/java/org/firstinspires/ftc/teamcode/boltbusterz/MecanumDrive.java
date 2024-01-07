package org.firstinspires.ftc.teamcode.boltbusterz;

import java.util.Arrays;

@SuppressWarnings("unused")
public class MecanumDrive {
    public double throttle;
    public MecanumDrive(double throttle){ this.throttle = throttle; }
    public double[] calculateTankPower(double rightStrafe, double leftStrafe, double leftTank, double rightTank){
        double[] inputs = {throttle * rightStrafe, throttle * leftStrafe, throttle * leftTank, throttle * rightTank, 1};
        double normalizer = Arrays.stream(inputs).max().getAsDouble();
        double frPower = (rightTank - rightStrafe + leftStrafe) * throttle / normalizer;
        double brPower = (rightTank + rightStrafe - leftStrafe) * throttle / normalizer;
        double blPower = (leftTank + leftStrafe - rightStrafe) * throttle / normalizer;
        double flPower = (leftTank - leftStrafe + rightStrafe) * throttle / normalizer;
        return new double[] {frPower, brPower, blPower, flPower};
    }
    public  double[] calculateOneStickPower(double x, double y, double turn, double offset){
        double direction = Math.atan2(y, x) - Math.toRadians(offset);
        double sin = Math.sin(direction - Math.PI/4);
        double cos = Math.sin(direction - Math.PI/4);
        double power = Math.sqrt(x*x + y*y);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        double frPower = power * sin/max + turn;
        double brPower = power * cos/max + turn;
        double blPower = power * sin/max + turn;
        double flPower = power * cos/max + turn;
        if ((power + Math.abs(turn)) > 1){
            frPower /= power + Math.abs(turn);
            brPower /= power + Math.abs(turn);
            blPower /= power + Math.abs(turn);
            flPower /= power + Math.abs(turn);
        }
        return new double[] {frPower, brPower, blPower, flPower};
    }
}
