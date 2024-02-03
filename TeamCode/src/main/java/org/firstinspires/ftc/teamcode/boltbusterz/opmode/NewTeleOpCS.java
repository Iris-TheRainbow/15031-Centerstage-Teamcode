package org.firstinspires.ftc.teamcode.boltbusterz.opmode;

import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.claw1Closed;
import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.f;
import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.idle;
import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.move;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide;
import org.firstinspires.ftc.teamcode.boltbusterz.mecDrive;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Photon
@TeleOp(name="TeleOp NEW NEW\n Centerstage\n\n lets go lt!!")
public class NewTeleOpCS extends OpMode {
    public LinearSlide slide;
    public mecDrive Drive;
    public Gamepad oldGamepad1, oldGamepad2, newGamepad1, newGamepad2;
    public ElapsedTime timer;
    public DcMotorEx leftFront, leftBack, rightBack, rightFront, linear;
    public Servo clawTop, clawBottom, plane, arm;
    public IMU imu;
    public double heading, timeMS, armTarget, linearPower, headingX, driveY, driveX;
    public int linearPos, planeSafety, linearTargetTicks, manualSlides;
    public boolean clawToggle, planeLaunch, armTargetUpdate, manualMode;
    public double throttle = 1;
    public double[] drivePower;

    @Override
    public void init() {
        slide = new LinearSlide();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drive = new mecDrive(throttle);
        oldGamepad1 = new Gamepad();
        oldGamepad2 = new Gamepad();
        newGamepad1 = new Gamepad();
        newGamepad2 = new Gamepad();
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        linear = hardwareMap.get(DcMotorEx.class, "linear");
        clawTop = hardwareMap.get(Servo.class, "clawServo2");
        clawBottom = hardwareMap.get(Servo.class, "clawServo");
        plane = hardwareMap.get(Servo.class, "planeServo");
        arm = hardwareMap.get(Servo.class, "armServo");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        slide.allClose();
        armTargetUpdate = true;
        oldGamepad1.copy(newGamepad1);
        oldGamepad2.copy(newGamepad2);
        newGamepad1.copy(gamepad1);
        newGamepad2.copy(gamepad2);
        linearTargetTicks = 0;
        manualMode = false;
    }

    @Override
    public void loop() {
        //Copy Gamepads
        oldGamepad2.copy(newGamepad2);
        oldGamepad1.copy(newGamepad1);
        newGamepad2.copy(gamepad2);
        newGamepad1.copy(gamepad1);

        //Read hardware states
        linearPos = linear.getCurrentPosition();
        timeMS = timer.time(TimeUnit.MILLISECONDS);
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //read gamepads
        if (newGamepad2.right_bumper && !oldGamepad2.right_bumper) {
            slide.topOpen();
        }
        if (newGamepad2.left_bumper && !oldGamepad2.left_bumper) {
            slide.bottomOpen();
        }
        if (newGamepad2.y){
            slide.allClose();
        }
        headingX = gamepad1.right_stick_x;
        driveY = -gamepad1.left_stick_y;
        driveX = gamepad1.left_stick_x;
        if (gamepad1.dpad_up) {
            planeSafety = planeSafety + 1;
        } else {
            planeSafety = 0;
        }
        if (planeSafety == 50) {
            planeLaunch = true;
            armTargetUpdate = true;
        }
        if (gamepad2.a) {
            linearTargetTicks = 0;
            armTargetUpdate = true;
        }
        if (gamepad2.b == !oldGamepad2.b && !gamepad2.start) {
            linearTargetTicks = 2000;
            armTargetUpdate = true;
        }
        if (gamepad2.x) {
            linearTargetTicks = 3000;
            armTargetUpdate = true;
        }
        if (gamepad1.dpad_right) {
            manualSlides = manualSlides + 1;
        }
        else {
            manualSlides = 0;
        }
        if (manualSlides == 100) {
            manualMode = true;
        }
        if(newGamepad1.dpad_down){
            manualMode = false;
        }

        //calculate taret hardware states
        if (!manualMode) {
            slide.linearSetTicks(linearTargetTicks);
            linearPower = slide.PID(linearPos);
        }
        boolean allowLinear = slide.safety(timeMS);
        if (armTargetUpdate) {
            armTarget = slide.getArmTarget();
        }
        if (!allowLinear) {
            linearPower = f;
        }
        if (manualMode) {
            linearPower = gamepad1.right_trigger + -gamepad1.left_trigger;
        }
        drivePower = Drive.calculateOneStickPower(driveX, driveY, headingX, heading);

        //hardware writes
        if (linearPower != linear.getPower()){
            linear.setPower(linearPower);
        }
        if (drivePower[3] != leftFront.getPower()) {
            leftFront.setPower(drivePower[3]);
        }
        if (drivePower[2] != leftBack.getPower()) {
            leftBack.setPower(drivePower[2]);
        }
        if (drivePower[0] != rightFront.getPower()) {
            rightFront.setPower(drivePower[0]);
        }
        if (drivePower[1] != rightBack.getPower()) {
            rightBack.setPower(drivePower[1]);
        }

        arm.setPosition(armTarget);
        if (manualMode){
            arm.setPosition(move);
        }
        clawTop.setPosition(slide.getClaw2());
        clawBottom.setPosition(slide.getClaw1());
    }
    @Override
    public void stop(){

    }
}
