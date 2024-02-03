package org.firstinspires.ftc.teamcode.boltbusterz;

import static org.firstinspires.ftc.teamcode.boltbusterz.opmode.TeleOpCS.visionTarget;

import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;

//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

@SuppressWarnings("unused")
@Config
public class mecDrive {
    public double throttle;
    public PIDController visionControler;
    public static double p = 0, i = 0, d = 0;
    //private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    public mecDrive(double throttle){
        this.throttle = throttle;
        visionControler = new PIDController(p, i, d);
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
    public  double[] calculateOneStickPower(double x, double y, double rx, double botHeading){
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY + rotX + rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;
        return new double[] {frPower, brPower, blPower, flPower};
    }

    public double[] calculateTwoStickPower(double driveX, double driveY, double headingX, double headingY, double currentHeading, double actual, boolean allow){
        rotX = driveX * Math.cos(-currentHeading) - driveY * Math.sin(-currentHeading);
        rotY = driveX * Math.sin(-currentHeading) + driveY * Math.cos(-currentHeading);
        rotX = rotX * 1.1;
        //headingController.setTargetPosition(Math.atan2(headingY, headingX));
        //if (Math.sqrt(headingX*headingX + headingY*headingY) > .5){ turn = headingController.update(currentHeading); }
        //else{turn = 0;}
        if (allow) {
            turn = visionControler.calculate(visionTarget, actual);
        }
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        flPower = (rotY + rotX + turn) / denominator;
        blPower = (rotY - rotX + turn) / denominator;
        frPower = (rotY - rotX - turn) / denominator;
        brPower = (rotY + rotX - turn) / denominator;
        return new double[] {frPower, brPower, blPower, flPower};
    }
}
