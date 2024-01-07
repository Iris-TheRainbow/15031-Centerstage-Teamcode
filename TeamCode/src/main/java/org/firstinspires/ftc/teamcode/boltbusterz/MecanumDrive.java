package org.firstinspires.ftc.teamcode.boltbusterz;

import com.arcrobotics.ftclib.controller.PIDController;

import java.util.Arrays;

@SuppressWarnings("unused")
public class MecanumDrive {
    public double throttle;
    public PIDController headingController;
    public double p, i, d;
    public MecanumDrive(double throttle){
        this.throttle = throttle;
        headingController = new PIDController(p, i, d);
    }
    double frPower, brPower, blPower, flPower;
    double sin, cos, power, max, direction, targetHeading, turn;
    public double[] calculateTankPower(double rightStrafe, double leftStrafe, double leftTank, double rightTank){
        double[] inputs = {throttle * rightStrafe, throttle * leftStrafe, throttle * leftTank, throttle * rightTank, 1};
        double normalizer = Arrays.stream(inputs).max().getAsDouble();
        frPower = (rightTank - rightStrafe + leftStrafe) * throttle / normalizer;
        brPower = (rightTank + rightStrafe - leftStrafe) * throttle / normalizer;
        blPower = (leftTank + leftStrafe - rightStrafe) * throttle / normalizer;
        flPower = (leftTank - leftStrafe + rightStrafe) * throttle / normalizer;
        return new double[] {frPower, brPower, blPower, flPower};
    }
    public  double[] calculateOneStickPower(double x, double y, double turn, double offset){
        direction = Math.atan2(y, x) - Math.toRadians(offset);
        sin = Math.sin(direction - Math.PI/4);
        cos = Math.sin(direction - Math.PI/4);
        power = Math.sqrt(x*x + y*y);
        max = Math.max(Math.abs(sin), Math.abs(cos));
        frPower = power * sin/max - turn;
        brPower = power * cos/max - turn;
        blPower = power * sin/max + turn;
        flPower = power * cos/max + turn;
        if ((power + Math.abs(turn)) > 1){
            frPower /= power + Math.abs(turn);
            brPower /= power + Math.abs(turn);
            blPower /= power + Math.abs(turn);
            flPower /= power + Math.abs(turn);
        }
        return new double[] {frPower, brPower, blPower, flPower};
    }

    public double[] calculateOneStickPidHeadingPower(double driveX, double drvieY, double headingX, double headingY, double currentHeading){
        direction = Math.atan2(drvieY, driveX) - Math.toRadians(currentHeading);
        sin = Math.sin(direction - Math.PI/4);
        cos = Math.sin(direction - Math.PI/4);
        power = Math.sqrt(driveX*driveX + drvieY*drvieY);
        max = Math.max(Math.abs(sin), Math.abs(cos));
        targetHeading = currentHeading - Math.toDegrees(Math.atan2(headingY, headingX));
        if (targetHeading > 180) {
            targetHeading = (-360) + targetHeading;
        } else if (targetHeading < -180) {
            targetHeading= 360 + targetHeading;
        }
        turn = headingController.calculate(currentHeading, targetHeading);
        frPower = power * sin/max - turn;
        brPower = power * cos/max - turn;
        blPower = power * sin/max + turn;
        flPower = power * cos/max + turn;
        if ((power + Math.abs(turn)) > 1){
            frPower /= power + Math.abs(turn);
            brPower /= power + Math.abs(turn);
            blPower /= power + Math.abs(turn);
            flPower /= power + Math.abs(turn);
        }
        return new double[] {frPower, brPower, blPower, flPower};
    }
}
