package org.firstinspires.ftc.teamcode.boltbusterz.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

import dev.frozenmilk.dairy.calcified.Calcified;
import dev.frozenmilk.dairy.calcified.hardware.sensor.CalcifiedIMU;
import dev.frozenmilk.dairy.core.util.OpModeLazyCell;

@TeleOp
@Calcified.Attach
public class DairyTest extends OpMode {
    private final Supplier<CalcifiedIMU> imu = new OpModeLazyCell<>(() -> Calcified.getControlHub().getIMU((byte) 0));
    @Override
    public void init() {
        telemetry.addData("heading", imu.get().getHeading());
        telemetry.addData("heading velocity", imu.get().getHeadingVelocity());
    }

    @Override
    public void loop() {
        telemetry.addData("heading", imu.get().getHeading());
        telemetry.addData("heading velocity", imu.get().getHeadingVelocity());
    }
}