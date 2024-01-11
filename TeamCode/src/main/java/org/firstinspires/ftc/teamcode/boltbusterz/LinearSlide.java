package org.firstinspires.ftc.teamcode.boltbusterz;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@SuppressWarnings("unused")
public class LinearSlide{
    public double SPOOL_CIRCUMFERENCE = 112; //spool circumference in MM
    public double TICK_PER_REV = 537.7;
    public double mmPerTick = (SPOOL_CIRCUMFERENCE / TICK_PER_REV);
    private final PIDController controller;
    public static double p = .005, i = 0, d = .0001, f = .00;
    public int target, pos;
    public double goTime;
    public double armTarget;
    public static double move = .5, idle = .6, score = 1;
    public LinearSlide(){ controller = new PIDController(p, i, d); }
    public void linearSetMM(int mm){ target = (int) (mm * mmPerTick); }
    public void linearSetTicks(int ticks){ target = ticks; }
    public int MMToTick(int mm){ return (int) (mm * mmPerTick); }
    public double PID(int pos){
        controller.setPID(p, i, d);
        this.pos = pos;
        double pid = controller.calculate(pos, target);
        return pid + f;
    }

    public boolean safety(double time){
        if (pos < 300 && target <= 300){ armTarget = idle; }
        if (pos >= 1800 && target >= 1800){ armTarget = score; }
        if (pos >= 3500){ armTarget = score; }
        if (pos >= 300 && pos < 1800){ armTarget = move; }
        if (pos < 300 && target > 300){
            armTarget = move;
        }
        if (pos >= 1800 && pos < 2500 && target < 2000){
            armTarget = move;
            goTime = time + 300;
        }
        if (pos >= 2500 && pos < 3000 && target < 2000){
            armTarget = move;
            goTime = time + 200;
        }
        if (pos >= 3000 && pos < 3500 && target < 1800){
            armTarget = move;
            goTime = time + 100;
        }
        if (goTime <= time){
            return true;
        }
        else{
            return false;
        }
    }
    public double getArmTarget(){ return armTarget; }
    public double getGoTime() { return  goTime; }

}
