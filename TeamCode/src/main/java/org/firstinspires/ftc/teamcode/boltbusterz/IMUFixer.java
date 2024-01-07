package org.firstinspires.ftc.teamcode.boltbusterz;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;

@SuppressWarnings("unused")
public class IMUFixer {
    public double corrector;
    public double oldHeading, oldTime, angVel;
    public boolean imuReset;
    public IMUFixer(){
        oldHeading = 0;
        oldTime = 0;
        corrector = 0;
        imuReset = false;
    }
    public double fixIMU(double heading, double timeMS){
        angVel = Math.abs(1000 * (heading - oldHeading) / (timeMS - oldTime));
        if (angVel > Math.toDegrees(MAX_ANG_VEL)){
            imuReset = true;
            corrector = oldHeading + corrector;
            if (corrector > 180) {
                corrector = (-360) + corrector;
            } else if (corrector < -180) {
                corrector= 360 + corrector;
            }
        }
        oldTime = timeMS;
        oldHeading = heading;
        return corrector;
    }

}
