package org.firstinspires.ftc.teamcode.boltbusterz.opmode;

import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.f;
import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.idle;
import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.move;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@Config
@TeleOp(name="TeleOp New Version Centerstage")
public class TeleOpCS extends OpMode {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, linear;
    public Servo claw, plane, arm;
    public IMU imu;
    public Gamepad oldGamepad1, oldGamepad2, newGamepad1, newGamepad2;
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
    public int planeSafety = 0, loopCount = 0;
    public static double launch = .3 , planeIdle = 1;
    public static double opened = .1, closed = .2;
    public boolean armTargetUpdate;
    public double armTarget = idle;

    @Override
    public void init() {
        slide = new LinearSlide();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(throttle);
        oldGamepad1 = new Gamepad();
        oldGamepad2 = new Gamepad();
        newGamepad1 = new Gamepad();
        newGamepad2 = new Gamepad();
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
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); }
    }

    @Override
    public void init_loop() { }
    @Override
    public void start() {
        loopCount = 0;
        oldGamepad1.copy(newGamepad1);
        oldGamepad2.copy(newGamepad2);
        newGamepad1.copy(gamepad1);
        newGamepad2.copy(gamepad2);
        plane.setPosition(planeIdle);
        claw.setPosition(opened);
        arm.setPosition(idle);
    }
    @Override
    public void loop(){
        //read data
        oldGamepad2.copy(newGamepad2);
        oldGamepad1.copy(newGamepad1);
        newGamepad2.copy(gamepad2);
        newGamepad1.copy(gamepad1);
        loopCount = loopCount + 1;
        int linearPos = linear.getCurrentPosition();
        double timeMS = timer.time(TimeUnit.MILLISECONDS);
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (linear.getCurrentPosition() < 300 && armTargetUpdate){ armTargetUpdate = false;}
        if (linear.getCurrentPosition() > 1800){ armTargetUpdate = true;}
        if (armTarget ==  move) { clawToggle = false; }



        //inputs
        double headingY = -gamepad1.right_stick_y;
        double headingX = gamepad1.right_stick_x;
        double driveY = -gamepad1 .left_stick_y;
        double driveX = gamepad1.left_stick_x;
        if (newGamepad2.right_bumper && !oldGamepad2.right_bumper || newGamepad2.left_bumper && !oldGamepad2.left_bumper){ clawToggle = !clawToggle; }
        if (gamepad1.dpad_up){ planeSafety = planeSafety + 1; }
        else{ planeSafety = 0; }
        if (planeSafety == 50){ planeLaunch = true; armTargetUpdate = true;}
        if (gamepad2.a){ linearTargetTicks = 0; armTargetUpdate = true;}
        if (gamepad2.b == !oldGamepad2.b && !gamepad2.start){ linearTargetTicks = 2000; armTargetUpdate = true;}
        if (gamepad2.x){ linearTargetTicks = 3000; armTargetUpdate = true;}
        if (gamepad2.y){ linearTargetTicks = 3500; armTargetUpdate = true;}
        if (gamepad2.dpad_down){ armTarget = .28; }
        if (gamepad2.dpad_up) {armTarget = idle;}




        //calculations
        slide.linearSetTicks(linearTargetTicks);
        linearPower = slide.PID(linearPos);
        boolean allowLinear = slide.safety(timeMS);
        if (armTargetUpdate){
            armTarget = slide.getArmTarget();
        }
        if (!allowLinear) { linearPower = f;}
        double[] drivePower = drive.calculateOneStickPower(driveX, driveY, headingX, heading);

        //actions
        linear.setPower(linearPower);
        leftFront.setPower(drivePower[3]);
        leftRear.setPower(drivePower[2]);
        rightFront.setPower(drivePower[0]);
        rightRear.setPower(drivePower[1]);
        arm.setPosition(armTarget);
        if (planeLaunch){ plane.setPosition(launch); }
        if (clawToggle){ claw.setPosition(opened); }
        else { claw.setPosition(closed); }
        if (gamepad1.back){imu.resetYaw();}

        //telemetry
        telemetry.addData("heading: ", heading);
        telemetry.addData("Corrected Heading: ", correctedHeading);
        telemetry.addData("linear Position: ",linearPos);
        telemetry.addData("linear Target Position: ", linearTargetTicks);
        telemetry.addData("TimeMS", timeMS);
        telemetry.addData("goTime", slide.getGoTime());
        telemetry.addData("allowLinear", allowLinear);
        telemetry.addData("averageHz", (timeMS/(1000 * loopCount)));
        telemetry.update();

    }
    @Override
    public void stop(){ }
}