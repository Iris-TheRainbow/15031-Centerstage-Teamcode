package org.firstinspires.ftc.teamcode.boltbusterz.opmode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Photon
@TeleOp(name="TeleOp\n New Version\n Centerstage")
public class TeleOpCS extends OpMode {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

    }
    @Override
    public void init_loop() {

    }
    @Override
    public void start() {

    }
    @Override
    public void loop(){

    }
    @Override
    public void stop(){

    }
}
