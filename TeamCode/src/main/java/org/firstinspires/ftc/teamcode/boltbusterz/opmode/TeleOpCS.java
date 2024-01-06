package org.firstinspires.ftc.teamcode.boltbusterz.opmode;

import static org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide.f;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.boltbusterz.LinearSlide;
import org.firstinspires.ftc.teamcode.boltbusterz.MecanumDrive;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Photon
@TeleOp(name="TeleOp\n New Version\n Centerstage")
public class TeleOpCS extends OpMode {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, linear;
    public Servo claw, plane, arm;
    public Gamepad newGamepad1, newGamepad2, oldGamepad1, oldGamepad2;
    public LinearSlide slide;
    public MecanumDrive drive;
    public static int throttle = 1;
    public boolean clawToggle = false;
    public boolean planeLaunch = false;
    public int planeSafety = 0;
    public double launch = 1, planeIdle = 0;
    public double opened = 0, closed = 1;
    public double linearPower;
    public ElapsedTime timer;

    public List<LynxModule> allHubs;
    public int linearTargetTicks;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        linear = hardwareMap.get(DcMotorEx.class, "linear");
        claw = hardwareMap.get(Servo.class, "clawServo");
        plane = hardwareMap.get(Servo.class, "planeServo");
        arm = hardwareMap.get(Servo.class, "armServo");
        linear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        slide = new LinearSlide();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(throttle);
        oldGamepad1 = new Gamepad();
        oldGamepad2 = new Gamepad();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }
    @Override
    public void init_loop() {
    }
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

        //inputs
        double rightStrafe = gamepad1.right_trigger;
        double leftStrafe = gamepad1.left_trigger;
        double leftTank = -gamepad1 .left_stick_y;
        double rightTank = -gamepad1.right_stick_y;
        if (gamepad2.right_bumper == !oldGamepad2.right_bumper || gamepad2.left_bumper == !oldGamepad2.left_bumper){
            clawToggle = !clawToggle;
        }
        if (gamepad1.dpad_up){
            planeSafety = planeSafety + 1;
        }
        else{
            planeSafety = 0;
        }
        if (planeSafety == 15){
            planeLaunch = true;
        }
        if (gamepad2.a){
            linearTargetTicks = 0;
        }
        if (newGamepad2.b == !oldGamepad2.b && !gamepad2.start){
            linearTargetTicks = 2000;
        }
        if (gamepad2.x){
            linearTargetTicks = 3000;
        }
        if (gamepad2.y){
            linearTargetTicks = 4000;
        }


        //calculations
        slide.linearSetTicks(linearTargetTicks);
        linearPower = slide.PID(linearPos);
        boolean allowLinear = slide.safety(timeMS);
        double armTarget = slide.getArmTarget();
        if (!allowLinear) {
            linearPower = f;
        }
        double[] drivePower = drive.CalculatePower(rightStrafe, leftStrafe, leftTank, rightTank);

        //actions
        linear.setPower(linearPower);
        leftFront.setPower(drivePower[0]);
        leftRear.setPower(drivePower[1]);
        rightFront.setPower(drivePower[2]);
        rightRear.setPower(drivePower[3]);
        arm.setPosition(armTarget);
        if (planeLaunch){
            plane.setPosition(launch);
        }
        if (clawToggle){
            claw.setPosition(opened);
        }
        else{
            claw.setPosition(closed);
        }

        //telemetry
        telemetry.addData("linearPos",linearPos);
        telemetry.addData("linearTarget", linearTargetTicks);
        telemetry.update();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        oldGamepad1.copy(gamepad1);
        oldGamepad2.copy(gamepad1);
    }
    @Override
    public void stop(){
    }
}