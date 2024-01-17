package org.firstinspires.ftc.teamcode.boltbusterz.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class armSubsystem extends SubsystemBase {
    private final Servo arm;


    public armSubsystem(final HardwareMap hMap) {
        arm = hMap.get(Servo.class, "armServo");
    }

    public void update(){

    }


}