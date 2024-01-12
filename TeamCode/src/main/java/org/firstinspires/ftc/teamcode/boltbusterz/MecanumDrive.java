package org.firstinspires.ftc.teamcode.boltbusterz;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

@SuppressWarnings("unused")
@Config
public class MecanumDrive {
    public double throttle;
    public PIDController headingController;
    public static double p = 0, i = 0, d = 0;
    public MecanumDrive(double throttle){
        this.throttle = throttle;
        headingController = new PIDController(p, i, d);
    }
    double frPower, brPower, blPower, flPower;
    double sin, cos, power, max, direction, denominator, normalizer, rotX, rotY, angle, heading, radians, translatedHeading, turn;
    public double[] calculateTankPower(double rightStrafe, double leftStrafe, double leftTank, double rightTank){
        double[] inputs = {throttle * rightStrafe, throttle * leftStrafe, throttle * leftTank, throttle * rightTank, 1};
        normalizer = Arrays.stream(inputs).max().getAsDouble();
        frPower = (rightTank - rightStrafe + leftStrafe) * throttle / normalizer;
        brPower = (rightTank + rightStrafe - leftStrafe) * throttle / normalizer;
        blPower = (leftTank + leftStrafe - rightStrafe) * throttle / normalizer;
        flPower = (leftTank - leftStrafe + rightStrafe) * throttle / normalizer;
        return new double[] {frPower, brPower, blPower, flPower};
    }
    public  double[] calculateOneStickPower(double driveX, double driveY, double turn, double currentHeading){
        rotX = driveX * Math.cos(-currentHeading) - driveY * Math.sin(-currentHeading);
        rotY = driveX * Math.sin(-currentHeading) + driveY * Math.cos(-currentHeading);
        rotX = rotX * 1.1;
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        flPower = (rotY + rotX + turn) / denominator;
        blPower = (rotY - rotX + turn) / denominator;
        frPower = (rotY - rotX - turn) / denominator;
        brPower = (rotY + rotX - turn) / denominator;
        return new double[] {frPower, brPower, blPower, flPower};
    }

    public double[] calculateTwoStickPower(double driveX, double driveY, double headingX, double headingY, double currentHeading){
        headingController.setPID(p,i,d);
        rotX = driveX * Math.cos(-currentHeading) - driveY * Math.sin(-currentHeading);
        rotY = driveX * Math.sin(-currentHeading) + driveY * Math.cos(-currentHeading);
        rotX = rotX * 1.1;
        angle = Math.atan2(headingY, headingX);
        heading = currentHeading + .5*Math.PI;
        if (heading < 0){
            heading += 2* Math.PI;
        }
        radians = angle - heading;
        if (Math.sqrt((headingX*headingX + headingY*headingY)) > .5){
            while (radians > Math.PI) {
                radians -= 2 * Math.PI;
            }
            while (radians < -Math.PI) {
                radians += 2 * Math.PI;
            }
            turn = headingController.calculate(heading,radians);
        }
        else{
            turn = 0;
        }
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        flPower = (rotY + rotX + turn) / denominator;
        blPower = (rotY - rotX + turn) / denominator;
        frPower = (rotY - rotX - turn) / denominator;
        brPower = (rotY + rotX - turn) / denominator;
        return new double[] {frPower, brPower, blPower, flPower};
    }
}
