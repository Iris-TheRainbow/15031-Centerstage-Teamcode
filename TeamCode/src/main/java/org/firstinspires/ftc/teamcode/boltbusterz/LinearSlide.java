package org.firstinspires.ftc.teamcode.boltbusterz;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;


@Config
public class LinearSlide{
    public double SPOOL_CIRCUMFERENCE = 112; //spool circumference in MM
    public double TICK_PER_REV = 537.7;
    public double mmPerTick = (SPOOL_CIRCUMFERENCE / TICK_PER_REV);
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public int target, pos;
    public boolean safe;
    public double goTime;
    public double armTarget;
    public static double move = .6, idle = .5, score = 0;
    public LinearSlide(){
        controller = new PIDController(p, i, d);
    }
    public void linearGoToMM(int mm){
        target = (int) (mm * mmPerTick);
    }
    public int MMToTick(int mm){
        int returnTicks = (int) (mm * mmPerTick);
        return returnTicks;
    }
    public double PID(int pos){
        this.pos = pos;
        double pid = controller.calculate(pos, target);
        double power = pid + f;
        return power;
    }

    public boolean safety(double time){
        if (pos < 300 && target <= 300){
            armTarget = idle;
            goTime = time;
        }
        if (pos < 300 && target > 300){
            armTarget = move;
            goTime = time + 100;
        }
        if (pos >= 300 && pos < 2000){
            armTarget = move;
            goTime = time;
        }
        if (pos >= 2000 && pos < 2500 && target < 2000){
            armTarget = move;
            goTime = time + 300;
        }
        if (pos >= 2500 && pos < 3000 && target < 2000){
            armTarget = move;
            goTime = time + 200;
        }
        if (pos >= 3000 && pos < 3500 && target < 2000){
            armTarget = move;
            goTime = time + 100;
        }
        if (pos >= 2000 && target >= 2000){
            armTarget = score;
            goTime = time;
        }
        if (pos >= 3500){
            armTarget = score;
            goTime = time;
        }
        if (goTime <= time){
            safe = true;
        }
        else{
            safe = false;
        }
        return safe;
    }
    public double getArmTarget(){
        return armTarget;
    }

}
