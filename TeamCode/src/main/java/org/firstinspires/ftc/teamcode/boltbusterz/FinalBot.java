package org.firstinspires.ftc.teamcode.boltbusterz;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.boltbusterz.subsystems.driveSubsystem;

public class FinalBot extends com.arcrobotics.ftclib.command.Robot {
    public enum OpModeType {TELEOP, AUTO}
    public FinalBot(OpModeType type) {
        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }
    public void initTele() {
    }

    public void initAuto() {

    }
}
