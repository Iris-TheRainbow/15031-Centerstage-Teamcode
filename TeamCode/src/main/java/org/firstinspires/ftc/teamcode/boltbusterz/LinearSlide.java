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
    public static double p = .03, i = 0, d = .0001, f = .03;
    public int target, targetOld, pos;
    public double goTime;
    public double armTarget;
    public double claw1, claw2;
    public boolean downAllowed = true, upAllowed = true, oneOpen = false;
    public static double move = .15  , idle = .28, score = .58;
    public static double claw1Open = 1, claw2open = .25, claw1Closed = .5, claw2Closed = 0;
    public LinearSlide(){ controller = new PIDController(p, i, d); }
    public void bottomOpen(){
        claw1 = claw1Open;
    }
    public void topOpen(){
        claw2 = claw2open;
    }
    public void bottomClosed(){
        claw1 = claw1Closed;
    }
    public void topClosed(){
        claw2 = claw2Closed;
    }
    public void allOpen(){
        topOpen();
        bottomOpen();
    }
    public void allClose() {
        topClosed();
        bottomClosed();
    }

    public void linearSetMM(int mm){ target = (int) (mm * mmPerTick); }
    public void linearSetTicks(int ticks){ targetOld = target; target = ticks; }
    public int MMToTick(int mm){ return (int) (mm * mmPerTick); }
    public double PID(int pos){
        controller.setPID(p, i, d);
        this.pos = pos;
        double pid = controller.calculate(pos, target);
        return pid + f;
    }

    public boolean safety(double time){
        if (targetOld != target){ downAllowed = true; }
        if (pos < 300 && target <= 300){ armTarget = idle; if (oneOpen) {allOpen(); oneOpen = false;}}
        if (pos >= 1800 && target >= 1800){ armTarget = score; }
        if (pos >= 3500){ armTarget = score; }
        if (pos >= 300 && pos < 1800){ armTarget = move; }
        if (pos < 300 && target > 300){
            armTarget = move;
            if (upAllowed) {
                allClose();
                goTime = time + 500;
                upAllowed = false;
            }
        }
        if (pos >= 1800 && pos < 2500 && target < 1800){
            armTarget = move;
            if (downAllowed) {
                allClose();
                goTime = time + 500;
                downAllowed = false;
            }
        }
        if (pos >= 2500 && pos < 3000 && target < 1800){
            armTarget = move;
            if (downAllowed) {
                allClose();
                goTime = time + 350;
                downAllowed = false;
            }
        }
        if (pos >= 3000 && pos < 3500 && target < 1800){
            armTarget = move;
            if (downAllowed) {
                allClose();
                goTime = time + 200;
                downAllowed = false;
            }
        }
        if (pos <= 1800)
            downAllowed = true;
        if (pos > 1800) {
            upAllowed = true;
            oneOpen = true;
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
    public double getClaw1() { return claw1; }
    public double getClaw2() { return claw2; }


}
