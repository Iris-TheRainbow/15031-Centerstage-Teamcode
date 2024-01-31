package org.firstinspires.ftc.teamcode.boltbusterz.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.boltbusterz.subsystems.slideSubsystem;

import java.util.Set;

public class slideToPosition extends CommandBase {
        // The subsystem the command runs on
        private final slideSubsystem m_slideSubsystem;
        private int target;
    public enum targets {
        GROUND(0),
        MID(100),
        CLIMB(200);

        private final int ticks;
        targets(int ticks) {
            this.ticks = ticks;
        }
        public int get() {
            return ticks;
        }
    }
    public slideToPosition(slideSubsystem subsystem, int target) {
        m_slideSubsystem = subsystem;
        addRequirements(m_slideSubsystem);
    }
    @Override
    public void initialize() {
        m_slideSubsystem.setTarget(target);
    }

    @Override
    public void execute() {
        m_slideSubsystem.runToPosition();
    }

    @Override
        public boolean isFinished() {
            if (m_slideSubsystem.getPosition() > target * .95 && m_slideSubsystem.getPosition() < target * 1.05){
                return true;
            }
            return false;
        }

    }
