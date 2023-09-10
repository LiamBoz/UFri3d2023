package org.firstinspires.ftc.teamcode.utils.ri3d;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.commands.AutoCameraControl;

import java.util.Date;
import java.util.function.BooleanSupplier;

public class ChoosePath {
    public static PropPlacement getPlacement(@NonNull AutoCameraControl cam, Telemetry telemetry, BooleanSupplier isStopRequested) {
        PropPlacement placement = cam.getPlacement();
        if(placement.name().equals(PropPlacement.UNKNOWN.name())) {
            telemetry.addData("choosing at", new Date().getTime());
            cam.periodic();
            if(isStopRequested.getAsBoolean()) {
                cam.getCamera().stop();
                return null;
            }
            return getPlacement(cam, telemetry, isStopRequested);
        }

        cam.getCamera().stop();
        return placement;
    }
}
