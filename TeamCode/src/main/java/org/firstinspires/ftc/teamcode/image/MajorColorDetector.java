package org.firstinspires.ftc.teamcode.image;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by crazy on 9/27/2017.
 */

public class MajorColorDetector extends Detector {
    public void logTelemetry(){}

    private static final double THRESHOLD = .25;
    private static final int BINS = 8;
    private static final float MIN_VALUE = 0.0f;
    private static final float MAX_VALUE = 255.0f;
    public enum Color {
        RED, BLUE, NONE
    }
    private Color foundColor = Color.NONE;
    public void logDebug()
    {

    }
    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    private void extract() {
        Mat hsvImage = new Mat(showImg.width(), showImg.height(), showImg.type());
        Mat histHue = new Mat();
        Mat histSaturation = new Mat();

        Imgproc.cvtColor(showImg, hsvImage, Imgproc.COLOR_BGR2HSV);
        List<Mat> channels = new ArrayList<Mat>();
        Core.split(hsvImage, channels);

        //Histogram for hue
        Imgproc.calcHist(Arrays.asList( new Mat[]{channels.get(0)} ), new MatOfInt(0),
                new Mat(), histHue, new MatOfInt(BINS), new MatOfFloat(MIN_VALUE, MAX_VALUE));

        //Histogram for saturation
        Imgproc.calcHist(Arrays.asList( new Mat[]{channels.get(1)} ), new MatOfInt(0),
                new Mat(), histSaturation, new MatOfInt(BINS), new MatOfFloat(MIN_VALUE, MAX_VALUE));


        double sum = Core.sumElems(histHue).val[0];
        double[] values = new double[histHue.height()+histSaturation.height()];
        int k = 0;
        for (int i = 0; i < histHue.height(); ++i ) {
            values[k++] = histHue.get(i, 0)[0]/sum;
        }
        sum = Core.sumElems(histSaturation).val[0];
        for ( int i = 0; i < histSaturation.height(); ++i) {
            values[k++] = histSaturation.get(i, 0)[0]/sum;
        }
        // 0 + 7 == red
        // 4 + 5 == blue
       int total = 0;

        for ( int i = 0; i<BINS; i++){

            total+=values[i];
        }
        double red = values[0] + values[7];
        double blue = values[4] + values[5];
        double redp = red/total;
        double bluep = blue/total;
        if(bluep >= THRESHOLD){
            //yayRed!!! :D
            foundColor = Color.RED;
        }
        else if(redp >= THRESHOLD)
        {
            //yayBlue!!! :D
            foundColor = Color.BLUE;
        }
        else{
            //booNone!!!:(
            foundColor = Color.NONE;
        }


    }
}
