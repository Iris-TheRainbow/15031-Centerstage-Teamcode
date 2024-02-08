package org.firstinspires.ftc.teamcode.boltbusterz.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.boltbusterz.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "Red auto", group = "Autonomous")
public class RedAuto extends OpMode{
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    private Servo topClaw, bottomClaw, arm;
    private DcMotor linear;
    public Action left, center, right, backdropLeft, backdropCenter, backdropRight, park, downfield;
    public MecanumDrive drive;
    public static double claw1Open = 1, claw2open = .25, claw1Closed = .50, claw2Closed = 0;
    public static double move = .15, idle = .26, score = .6;
    public void betterSleep(long millis){
        try {
            TimeUnit.MILLISECONDS.sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void yellowPixel(){
        linear.setTargetPosition(2000);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(1);
        betterSleep(3000);
        arm.setPosition(score);
        betterSleep(1000);
        topClaw.setPosition(claw2open);
        betterSleep(100);
        topClaw.setPosition(claw2Closed);
        betterSleep(100);
        arm.setPosition(move);
        betterSleep(1000);
        linear.setTargetPosition(0);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(1);
        betterSleep(3000);
    }
    public void purplePixel() {
        bottomClaw.setPosition(claw1Open);
        arm.setPosition(move);
        betterSleep(100);
        bottomClaw.setPosition(claw1Closed);
        betterSleep(100);
    }
    @Override
    public void init() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, -61.7, Math.toRadians(90)));
        Scalar lower = new Scalar(150, 90, 90); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(210, 210, 210); // the upper hsv threshold for your detection
        double minArea = 100; // the minimum area for the detection to consider for your prop
        linear = hardwareMap.get(DcMotor.class, "linear");
        topClaw = hardwareMap.get(Servo.class, "clawServo2");
        bottomClaw = hardwareMap.get(Servo.class, "clawServo");
        arm = hardwareMap.get(Servo.class, "armServo");

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 200, // the left dividing line, in this case the left third of the frame
                () -> 5000 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cam")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .build();
        left = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(29, -35),Math.toRadians(140))
                .build();
        center = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(17, -27, Math.toRadians(140)),Math.toRadians(140))
                .build();
        right = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(17, -27, Math.toRadians(140)),Math.toRadians(140))
                .build();
        downfield = drive.actionBuilder(drive.pose).turnTo(90).build();
        backdropLeft = drive.actionBuilder(drive.pose).setReversed(true).splineTo(new Vector2d(38, 42), Math.toRadians(0)).build();
        backdropCenter = drive.actionBuilder(drive.pose).setReversed(true).splineTo(new Vector2d(38, 36), Math.toRadians(0)).build();
        backdropRight = drive.actionBuilder(drive.pose).setReversed(true).splineTo(new Vector2d(38, 30), Math.toRadians(0)).build();
        park = drive.actionBuilder(drive.pose).splineTo(new Vector2d(58, 61), Math.toRadians(0)).build();


        bottomClaw.setPosition(claw1Closed);
        topClaw.setPosition(claw2Closed );
    }

    public void init_loop(){
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
    }
    public void start(){
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();
        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.RIGHT;
        }
        switch (recordedPropPosition) {
            case LEFT:
                Actions.runBlocking(left);
                purplePixel();
                Actions.runBlocking(downfield);
                //Actions.runBlocking(backdropLeft);
                //yellowPixel();
                //Actions.runBlocking(park);
                break;
            case MIDDLE:
                Actions.runBlocking(center);
                purplePixel();
                Actions.runBlocking(downfield);
                //Actions.runBlocking(backdropCenter);
                //yellowPixel();
                //Actions.runBlocking(park);
                break;
            case RIGHT:
                Actions.runBlocking(right);
                purplePixel();
                Actions.runBlocking(downfield);
                //Actions.runBlocking(backdropRight);
                //yellowPixel();
                //Actions.runBlocking(park);
                break;

        }
    }
    @Override
    public void loop(){

    }

    @Override
    public void  stop(){

    }
}
