package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.ri3d.PropPatternPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    private HardwareMap hardwareMap;

    private OpenCvCamera webcam;
    private PropPatternPipeline vision;

    // HSV Threshold input variables
    private static final double HUE_MAX = 180.0;
    private static final double SAT_MAX = 255.0;
    private static final double VAL_MAX = 255.0;
    private static final double HSV_MIN = 0.0;

    // Initializes HSV values to the range used during testing
    private static double[] hsvHue = new double[]{36.40, 101.08};
    private static double[] hsvSat = new double[]{42.32, 255.00};
    private static double[] hsvVal = new double[]{32.11, 217.04};

    private static double rectTop   = 0.0;
    private static double rectLeft  = 0.0;
    private static double rectBot   = 246.11;
    private static double rectRight = 640;

    private static final int IMG_WIDTH = 480;
    private static final int IMG_HEIGHT = 640;

    private static boolean returnHSV = false;
    private static boolean drawRect = true;
    private static boolean showTargetColor = true;
    private static boolean showPoint = true;

    private double bound;

    public boolean hasInitialized = false;

    //Outputs
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        initHardware(telemetry);
    }

    public void initHardware(Telemetry telemetry) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                vision = new PropPatternPipeline();
                telemetry.addLine("opened");
                telemetry.update();

                webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                telemetry.addLine("gpu acc");
                telemetry.update();
                webcam.setPipeline(vision);
                telemetry.addLine("pipeline set");
                telemetry.update();
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 30);

                telemetry.addLine("stream");
                telemetry.update();

                hasInitialized = true;
                telemetry.addLine("inited");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {

            }
        });


//        FtcDashboard.getInstance().startCameraStream(webcam, 20);
    }

    public void periodic() {
        vision.setRectLeft(rectLeft);
        vision.setRectTop(rectTop);
        vision.setRectRight(rectRight);
        vision.setRectBot(rectBot);
        vision.setBound(bound);

        vision.setReturnHSV(returnHSV);
        vision.setDrawRect(drawRect);

        vision.setHsvHueMin(hsvHue[0]);
        vision.setHsvHueMax(hsvHue[1]);
        vision.setHsvSatMin(hsvSat[0]);
        vision.setHsvSatMax(hsvSat[1]);
        vision.setHsvValMin(hsvVal[0]);
        vision.setHsvValMax(hsvVal[1]);

//        filterContoursOutput = vision.filterContoursOutput();
    }

    public void stop() {
        webcam.stopRecordingPipeline();
        webcam.stopStreaming();
        FtcDashboard.getInstance().stopCameraStream();
    }

    public double[] getHsvHue() {
        return hsvHue;
    }
    public double[] getHsvSat() {
        return hsvSat;
    }
    public double[] getHsvVal() {
        return hsvSat;
    }

    public double getBound() {
        return bound;
    }

    public ArrayList<MatOfPoint> getFindContoursOutput() {
        return findContoursOutput;
    }

    public double getRectBot() {
        return rectBot;
    }
    public double getRectLeft() {
        return rectLeft;
    }
    public double getRectRight() {
        return rectRight;
    }
    public double getRectTop() {
        return rectTop;
    }

    public void setHsvHueMin(double hueMin) {
        hsvHue[0] = Math.max(hueMin, HSV_MIN);
    }
    public void setHsvHueMax(double hueMax) {
        hsvHue[1] = Math.min(hueMax, HUE_MAX);
    }

    public void setHsvSatMin(double satMin) {
        hsvSat[0] = Math.max(satMin, HSV_MIN);
    }
    public void setHsvSatMax(double satMax) {
        hsvSat[1] = Math.min(satMax, SAT_MAX);
    }

    public void setHsvValMin(double valMin) {
        hsvVal[0] = Math.max(valMin, HSV_MIN);
    }
    public void setHsvValMax(double valMax) {
        hsvVal[1] = Math.min(valMax, VAL_MAX);
    }

    public void setRectBot(double rectBot) {
        this.rectBot = rectBot;
    }
    public void setRectLeft(double rectLeft) {
        this.rectLeft = rectLeft;
    }
    public void setRectRight(double rectRight) {
        this.rectRight = rectRight;
    }
    public void setRectTop(double rectTop) {
        this.rectTop = rectTop;
    }

    public List<MatOfPoint> getContoursOutput(){
        return filterContoursOutput;
    }

    public void setReturnHSV(boolean returnHSV){
        this.returnHSV = returnHSV;
    }
    public void showTargetColor(boolean showTargetColor) {
        this.showTargetColor = showTargetColor;
    }
    public void setShowPoint(boolean showPoint) {
        this.showPoint = showPoint;
    }
    public void setDrawRect(boolean drawRect) {
        this.drawRect = drawRect;
    }

    public void setBound(double bound) {
        this.bound = bound;
    }

}

