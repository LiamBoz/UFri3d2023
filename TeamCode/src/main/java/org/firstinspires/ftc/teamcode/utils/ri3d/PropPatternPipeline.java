package org.firstinspires.ftc.teamcode.utils.ri3d;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

// TODO: Use GRIP to get the proper Pipeline for filtering and detection
public class PropPatternPipeline extends OpenCvPipeline {
    // Control which HSV values we look for
    private static double[] hsvHue = new double[]{0.0, 255.0};
    private static double[] hsvSat = new double[]{0.0, 255.0};
    private static double[] hsvVal = new double[]{0.0, 255.0};

    // Control where in the camera's field of view we look
    private static double rectTop   = 0;
    private static double rectLeft  = 0;
    private static double rectBot   = 480;
    private static double rectRight = 640;

    // Bound between middle and side views
    private static double bound = 320.0;

    // Extra stuff to look at
    private static boolean returnHSV = false;
    private static boolean drawRect = true;
    private static boolean showPoint = true;
    private static boolean showTargetColor = true;

    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

    public static ArrayList<Point> rectPoints = new ArrayList<Point>();

    @Override
    public Mat processFrame(Mat mat) {
        return null;
    }

    public void setHsvHueMin(double hsvHueMin) { hsvHue[0] = hsvHueMin;}
    public void setHsvHueMax(double hsvHueMax) { hsvHue[1] = hsvHueMax;}

    public void setHsvSatMin(double hsvSatMin) { hsvSat[0] = hsvSatMin;}
    public void setHsvSatMax(double hsvSatMax) { hsvSat[1] = hsvSatMax;}

    public void setHsvValMin(double hsvValMin) { hsvVal[0] = hsvValMin;}
    public void setHsvValMax(double hsvValMax) { hsvVal[1] = hsvValMax;}

    public void setRectTop(double top)      { rectTop   =     top;}
    public void setRectLeft(double left)    { rectLeft  =    left;}
    public void setRectBot(double bot)      { rectBot   =     bot;}
    public void setRectRight(double right)  { rectRight =   right;}

    public void setBound(double bound)     { this.bound    = bound;}

    public void setReturnHSV(boolean retHsv)    { returnHSV = retHsv;}
    public void setDrawRect(boolean draw)       { drawRect  = draw;}
}
