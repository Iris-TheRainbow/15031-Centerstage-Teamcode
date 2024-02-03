package org.firstinspires.ftc.teamcode.boltbusterz.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "test")
public class test extends OpMode {
    public Servo clawTop, clawBottom, arm;
    public DcMotor linear;
    public static double top = 0, bottom = 0, arm1 = 0;
    @Override
    public void init() {
       linear = hardwareMap.get(DcMotor.class, "linear");
    }

    @Override
    public void loop() {
        telemetry.addData("linearPos", linear.getCurrentPosition());
    }
}
