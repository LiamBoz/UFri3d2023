package org.firstinspires.ftc.teamcode.utils.ri3d;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.Arrays;
import java.util.Locale;

public class CameraConfig {
    private static final String CONFIG_NAME = "camera.cfg";
    private double[] hue;
    private double[] sat;
    private double[] val;
    private double[] rectBounds;
    private double[] sectionBounds;

    public CameraConfig() {
        try {
            String file = (String) ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(CONFIG_NAME));
            double[] data = Arrays.stream(file.split("\n|,")).mapToDouble(Double::parseDouble).toArray();

            hue = new double[]{data[0], data[1]};
            sat = new double[]{data[2], data[3]};
            val = new double[]{data[4], data[5]};

            rectBounds = new double[]{data[6], data[7], data[8], data[9]};

            sectionBounds = new double[data.length - 10];
            for(int i = 10; i < data.length; i++)
                sectionBounds[i - 10] = data[i];


        } catch (Exception e) {

        }
    }

    public double[] getHue() {
        return hue;
    }

    public double[] getSat() {
        return sat;
    }

    public double[] getVal() {
        return val;
    }

    public double[] getRectBounds() {
        return rectBounds;
    }

    public static void save(@NonNull double[] hue, @NonNull double[] sat, @NonNull double[] val, @NonNull double[] rectBounds, @NonNull double[] sectionBounds) {
        String text = "";
        text += String.format(Locale.ENGLISH,"%f,%f\n%f,%f\n%f,%f\n", hue[0], hue[1], sat[1], sat[0], val[0], val[1]);
        text += String.format(Locale.ENGLISH, "%f,%f,%f,%f\n", rectBounds[0], rectBounds[1], rectBounds[2], rectBounds[3]);
        for(double d : sectionBounds)
            text += d + "\n";

        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(CONFIG_NAME), text);
    }
}
