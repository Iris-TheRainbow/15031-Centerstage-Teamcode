package org.firstinspires.ftc.teamcode.boltbusterz.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class planeLauncherSubsystem extends SubsystemBase {
    private final Servo plane;

    public planeLauncherSubsystem(final HardwareMap hMap) {
        plane = hMap.get(Servo.class, "planeServo");
        plane.setPosition(1);
    }

    public void launch() {
        plane.setPosition(0.3);
    }
}
