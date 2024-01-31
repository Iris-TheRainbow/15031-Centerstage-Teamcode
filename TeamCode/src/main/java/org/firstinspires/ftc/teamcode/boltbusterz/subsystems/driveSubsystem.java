package org.firstinspires.ftc.teamcode.boltbusterz.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.boltbusterz.PixelDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
@Config
public class driveSubsystem extends SubsystemBase {
    public double rotX, rotY, denominator, lfPower, lbPower, rfPower, rbPower, driveX, driveY, turn;
    public PIDController visionControler;
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public IMU imu;
    public PixelDetectionProcessor pixelDetection;
    public VisionPortal visionPortal;
    public static double p = 0, i = 0, d = 0;
    public static int visionTarget;
    public driveSubsystem(final HardwareMap hMap) {
        visionControler = new PIDController(p, i, d);
        leftFront = hMap.get(DcMotor.class, "leftFront");
        leftBack = hMap.get(DcMotor.class, "leftBack");
        rightFront = hMap.get(DcMotor.class, "rightFront");
        rightBack = hMap.get(DcMotor.class, "rightRear");
        imu = hMap.get(IMU.class, "imu");
        pixelDetection = new PixelDetectionProcessor(visionTarget);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
    }

    public void OneStickDrive(double driveX, double driveY, double turn) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        visionControler.setPID(p, i, d);
        rotX = driveX * Math.cos(-currentHeading) - driveY * Math.sin(-currentHeading);
        rotY = driveX * Math.sin(-currentHeading) + driveY * Math.cos(-currentHeading);
        rotX = rotX * 1.1;
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        lfPower = (rotY + rotX + turn) / denominator;
        lbPower = (rotY - rotX + turn) / denominator;
        rfPower = (rotY - rotX - turn) / denominator;
        rbPower = (rotY + rotX - turn) / denominator;
        leftFront.setPower(lfPower);
        leftBack.setPower(lbPower);
        rightFront.setPower(rfPower);
        rightBack.setPower(rbPower);
    }


}
