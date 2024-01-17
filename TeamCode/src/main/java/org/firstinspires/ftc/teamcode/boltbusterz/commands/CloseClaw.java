package org.firstinspires.ftc.teamcode.boltbusterz.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.boltbusterz.subsystems.clawSubsystem;

public class CloseClaw extends InstantCommand {

    private final clawSubsystem clawSubsystem;

    public CloseClaw(clawSubsystem subsystem) {
        clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {
        clawSubsystem.close();
    }
}