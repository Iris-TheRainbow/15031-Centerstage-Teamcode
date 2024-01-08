package org.firstinspires.ftc.teamcode.boltbusterz;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;

@SuppressWarnings("unused")
public class IMUFixer {
    public double corrector, finalCorrector;
    public double oldHeading, oldTime, angVel;
    public boolean imuReset;
    public int counter;
    public boolean failed;
    public IMUFixer(){
        failed = false;
        oldHeading = 0;
        oldTime = 0;
        corrector = 0;
        counter = 0;
        imuReset = false;
    }
    public double fixIMU(double heading, double timeMS){
        if (!failed) {
            angVel = Math.abs(1000 * (heading - oldHeading) / (timeMS - oldTime));
            if (angVel > Math.toDegrees(MAX_ANG_VEL)) {
                imuReset = true;
                corrector = oldHeading + corrector;
                if (corrector > 180) {
                    corrector = (-360) + corrector;
                } else if (corrector < -180) {
                    corrector = 360 + corrector;
                }
            }
            if (heading == (double) 0) {
                counter = counter + 1;
                if (counter > 150) {
                    finalCorrector = 99999999;
                    counter = -2;
                }
                if (counter == -1) {
                    finalCorrector = 0;
                    failed = true;
                }
            } else {
                finalCorrector = corrector;
            }
            oldTime = timeMS;
            oldHeading = heading;

        }
        if (failed){
            finalCorrector = 0;
        }
        return finalCorrector;
    }

}
