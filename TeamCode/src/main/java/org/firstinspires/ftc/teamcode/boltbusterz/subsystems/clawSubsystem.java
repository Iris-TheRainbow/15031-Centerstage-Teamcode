package org.firstinspires.ftc.teamcode.boltbusterz.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawSubsystem extends SubsystemBase {
    private final Servo claw;

    public clawSubsystem(final HardwareMap hMap) {
        claw = hMap.get(Servo.class, "clawServo");
    }

    public void open() {
        claw.setPosition(0.1);
    }

    public void close() {
        claw.setPosition(0.2);
    }
}
