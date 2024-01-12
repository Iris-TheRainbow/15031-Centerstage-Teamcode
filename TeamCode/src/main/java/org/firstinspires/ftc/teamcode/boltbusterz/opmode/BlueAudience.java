package org.firstinspires.ftc.teamcode.boltbusterz.opmode;

import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.move;
import static org.firstinspires.ftc.teamcode.boltbusterz.opmode.TeleOpCS.closed;
import static org.firstinspires.ftc.teamcode.boltbusterz.opmode.TeleOpCS.opened;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Audience\ngo and drop")
public class BlueAudience extends LinearOpMode {
    public Servo arm, claw;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(Servo.class, "armServo");
        claw = hardwareMap.get(Servo.class, "clawServo");
        Pose2d startPose = new Pose2d(-36, 62, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, 12))
                .lineTo(new Vector2d(48, 12))
                .build();
        waitForStart();

        if (!isStopRequested()) {
            claw.setPosition(closed);
            sleep(2000);
            arm.setPosition(move);
            sleep(200);
            drive.followTrajectorySequence(traj);
            claw.setPosition(opened);
        }
    }
}
