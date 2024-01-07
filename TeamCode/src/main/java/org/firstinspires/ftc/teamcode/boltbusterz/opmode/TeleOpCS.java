package org.firstinspires.ftc.teamcode.boltbusterz.opmode;

import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.f;
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
import org.firstinspires.ftc.teamcode.boltbusterz.IMUFixer;
import org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide;
import org.firstinspires.ftc.teamcode.boltbusterz.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Photon
@TeleOp(name="TeleOp\n New Version\n Centerstage")
public class TeleOpCS extends OpMode {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, linear;
    public Servo claw, plane, arm;
    public IMU imu;
    public Gamepad oldGamepad1, oldGamepad2;
    public LinearSlide slide;
    public MecanumDrive drive;
    public ElapsedTime timer;
    public List<LynxModule> allHubs;
    public IMUFixer imuFixer;
    public IMU.Parameters parameters;
    public double linearPower;
    public double heading, correctedHeading;
    public int linearTargetTicks;
    public boolean clawToggle = false;
    public boolean planeLaunch = false;
    public double throttle = 1;
    public int planeSafety = 0;
    public double launch = 1, planeIdle = 0;
    public double opened = 0, closed = 1;

    @Override
    public void init() {
        slide = new LinearSlide();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(throttle);
        oldGamepad1 = new Gamepad();
        oldGamepad2 = new Gamepad();
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        imuFixer = new IMUFixer();
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        linear = hardwareMap.get(DcMotorEx.class, "linear");
        claw = hardwareMap.get(Servo.class, "clawServo");
        plane = hardwareMap.get(Servo.class, "planeServo");
        arm = hardwareMap.get(Servo.class, "armServo");
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        linear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }
    }

    @Override
    public void init_loop() { }
    @Override
    public void start() {
        oldGamepad1.copy(gamepad1);
        oldGamepad2.copy(gamepad2);
        plane.setPosition(planeIdle);
        claw.setPosition(opened);
    }
    @Override
    public void loop(){
        //read data
        int linearPos = linear.getCurrentPosition();
        double timeMS = timer.time(TimeUnit.MILLISECONDS);
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        correctedHeading = imuFixer.fixIMU(heading, timeMS) + heading;

        //inputs
        double headingY = -gamepad1.right_stick_y;
        double headingX = gamepad1.right_stick_x;
        double driveY = -gamepad1 .left_stick_y;
        double driveX = -gamepad1.right_stick_y;
        if (gamepad2.right_bumper == !oldGamepad2.right_bumper || gamepad2.left_bumper == !oldGamepad2.left_bumper){ clawToggle = !clawToggle; }
        if (gamepad1.dpad_up){ planeSafety = planeSafety + 1; }
        else{ planeSafety = 0; }
        if (planeSafety == 15){ planeLaunch = true; }
        if (gamepad2.a){ linearTargetTicks = 0;}
        if (gamepad2.b == !oldGamepad2.b && !gamepad2.start){ linearTargetTicks = 2000; }
        if (gamepad2.x){ linearTargetTicks = 3000; }
        if (gamepad2.y){ linearTargetTicks = 4000; }

        //calculations
        slide.linearSetTicks(linearTargetTicks);
        linearPower = slide.PID(linearPos);
        boolean allowLinear = slide.safety(timeMS);
        double armTarget = slide.getArmTarget();
        if (!allowLinear) { linearPower = f; }
        double[] drivePower = drive.calculateTwoStickPower(driveX, driveY, headingX, headingY, heading);

        //actions
        linear.setPower(linearPower);
        leftFront.setPower(drivePower[0]);
        leftRear.setPower(drivePower[1]);
        rightFront.setPower(drivePower[2]);
        rightRear.setPower(drivePower[3]);
        arm.setPosition(armTarget);
        if (planeLaunch){ plane.setPosition(launch); }
        if (clawToggle){ claw.setPosition(opened); }
        else { claw.setPosition(closed); }

        //telemetry
        telemetry.addData("heading: ", heading);
        telemetry.addData("Corrected Heading: ", correctedHeading);
        telemetry.addData("linear Position: ",linearPos);
        telemetry.addData("linear Target Position: ", linearTargetTicks);
        telemetry.update();
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }
        oldGamepad1.copy(gamepad1);
        oldGamepad2.copy(gamepad1);
    }
    @Override
    public void stop(){ }
}