package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.commands.AutoCameraControl;
import org.firstinspires.ftc.teamcode.robot.subsystems.Camera;
import org.firstinspires.ftc.teamcode.utils.ri3d.ChoosePath;
import org.firstinspires.ftc.teamcode.utils.ri3d.PropPlacement;

import java.util.Date;
import java.util.Objects;

public class TestCamera extends LinearOpMode {
    private AutoCameraControl cam;

    @Override
    public void runOpMode() throws InterruptedException {
        cam = new AutoCameraControl(new Camera(hardwareMap, telemetry), gamepad1, gamepad2, telemetry);

        while (!cam.getCamera().hasInitialized && !opModeIsActive() && !isStopRequested());

        telemetry.addLine("Camera initialized");
        telemetry.addLine("Control enabled");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Time at", new Date().getTime());
            cam.periodic();
            if(isStopRequested()) {
                cam.getCamera().stop();
            }
        }

        waitForStart();

        PropPlacement placement = Objects.requireNonNull(ChoosePath.getPlacement(cam, telemetry, this::isStopRequested));

        runCommonPathBeforeSplit();

        switch (placement) {
            case LEFT:
                runLeftPath();
                break;
            case CENTER:
                runCenterPath();
                break;
            case RIGHT:
                runRightPath();
                break;
            case UNKNOWN:
                runLeftPath(); // TODO: Replace this with whichever path scores the most points
                break;
        }

        runCommonPathAfterSplit();
    }

    private void runCommonPathBeforeSplit() {
        telemetry.addLine("common path 1");
        telemetry.update();
        cam.getCamera().stop();
        sleep(1000);
    }

    private void runLeftPath() {
        telemetry.addLine("left path");
        telemetry.update();
        sleep(1000);

    }

    private void runCenterPath() {
        telemetry.addLine("center path");
        telemetry.update();
        sleep(1000);

        runCommonPathAfterSplit();
    }

    private void runRightPath() {
        telemetry.addLine("right path");
        telemetry.update();
        sleep(1000);

        runCommonPathAfterSplit();
    }

    private void runCommonPathAfterSplit() {
        telemetry.addLine("common path 2");
        telemetry.update();
        sleep(1000);
    }

}
