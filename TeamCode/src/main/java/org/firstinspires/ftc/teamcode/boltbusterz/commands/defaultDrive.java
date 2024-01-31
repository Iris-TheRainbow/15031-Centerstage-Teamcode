package org.firstinspires.ftc.teamcode.boltbusterz.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.boltbusterz.subsystems.driveSubsystem;
import java.util.function.DoubleSupplier;

public class defaultDrive extends CommandBase {
    private final driveSubsystem m_drive;
    private final DoubleSupplier m_x, m_y;
    private final DoubleSupplier m_rotation;

    public defaultDrive(driveSubsystem subsystem, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier rotation) {
        m_drive = subsystem;
        m_x = driveX;
        m_y = driveY;
        m_rotation = rotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.OneStickDrive(
                m_x.getAsDouble(),
                m_y.getAsDouble(),
                m_rotation.getAsDouble()
        );
    }

}