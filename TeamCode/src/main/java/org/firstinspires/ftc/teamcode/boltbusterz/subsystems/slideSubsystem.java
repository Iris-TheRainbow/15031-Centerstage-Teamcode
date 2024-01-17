package org.firstinspires.ftc.teamcode.boltbusterz.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class slideSubsystem extends SubsystemBase {
    private final DcMotor linear;
    private double target;
    private final PIDController controller;
    public static double p = .03, i = 0, d = .0001, f = .00;

    public slideSubsystem(final HardwareMap hMap) {
        controller = new PIDController(p, i, d);
        linear = hMap.get(DcMotor.class, "linear");
        target = linear.getCurrentPosition();
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void runToPosition(){
        controller.setPID(p,i,d);
        double power = controller.calculate(target);
        linear.setPower(power + f);
    }

    public void setPower(double power){
        linear.setPower(power);
    }


    @Override
    public void setDefaultCommand(Command defaultCommand) {

    }
}
