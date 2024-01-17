package org.firstinspires.ftc.teamcode.boltbusterz.commands;

import com.arcrobotics.ftclib.command.InstantCommand;


import org.firstinspires.ftc.teamcode.boltbusterz.subsystems.planeLauncherSubsystem;

public class launchPlane extends InstantCommand {

    private final planeLauncherSubsystem plane;
    public launchPlane(planeLauncherSubsystem subsystem) {
        plane = subsystem;
        addRequirements(plane);
    }

    @Override
    public void execute() {
        plane.launch();
    }
}
