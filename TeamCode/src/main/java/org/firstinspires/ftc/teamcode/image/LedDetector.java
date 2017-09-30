package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import hallib.HalDashboard;


public class LedDetector extends Detector implements ImageProcessor
{
    private Mat showImg;
    private Mat hsvImage;
    private Mat zonedImg;

    private Mat hchy;
    private Mat outMat;
    private Mat whiteMat;

    private static final int LIGHT_CHANNEL = 1;
    private static final int A2RGB = Imgproc.COLOR_HLS2RGB;
    private static final int RGB2A = Imgproc.COLOR_RGB2HLS;
    private static final int MIN_AREA = 4;
    private static final int MIN_BRIGHTNESS = 220;
//    private static final int LIGHT_CHANNEL = 2;
//    private static final int A2RGB = Imgproc.COLOR_HSV2RGB;
//    private static final int RGB2A = Imgproc.COLOR_RGB2HSV;
//    private static final int MIN_BRIGHTNESS = 251;

    private List<Rect> leds = new ArrayList<>();
    //private RingBuffer beaconConfBuf = new RingBuffer(20);

    private int numLeds = 0;
    private int skipCnt = 0;
    private static final int SKIP = 50;

    private HalDashboard dashboard;

    private static String TAG = "SJH_LedDetector";

    public LedDetector()
    {
        dashboard = CommonUtil.getInstance().getDashboard();
    }

    @Override
    public void setImage( Mat img )
    {
        // Convert to HSV colorspace to make it easier to
        // threshold certain colors (ig red/blue)
        if(hsvImage == null)
        {
            hsvImage = new Mat();
        }
        Imgproc.cvtColor( img, hsvImage, RGB2A, 4 );
        if(showImg == null) showImg = hsvImage.clone();
        else hsvImage.copyTo(showImg);

        if ( !sensingActive ) return;
        findColors();
        firstCalcsDone = true;
    }

    @Override
    public Mat draw()
    {
        if(outMat == null) outMat = hsvImage.clone();
        else hsvImage.copyTo(outMat);
        Imgproc.cvtColor( showImg, outMat, A2RGB, 4 );
        if ( !sensingActive || !firstCalcsDone ) return outMat;

        for ( Rect ledRect : leds )
        {
            Imgproc.rectangle( outMat, ledRect.tl(), ledRect.br(), new Scalar(0,255,0), 4 );
        }

        newImage = true;

        return outMat;
    }

    public void logDebug()
    {
        RobotLog.ii(TAG, "Num LEDs: %4d", numLeds);
    }

    public void logTelemetry()
    {
        dashboard.displayPrintf(4,"LEDS: %4d", getNumLEDs());
    }

    public synchronized int getNumLEDs() { return numLeds; }

    private void findColors()
    {
        numLeds = 0;
        leds.clear();
        findLum();
        findLeds();
    }

    private void findLum()
    {
        List<Mat> channels = new ArrayList<>();
        Core.split( hsvImage, channels );

        Mat lum = channels.get( LIGHT_CHANNEL );

        if(whiteMat == null) whiteMat = lum.clone();
        else lum.copyTo(whiteMat);

        Imgproc.threshold( lum, whiteMat, MIN_BRIGHTNESS, 255, Imgproc.THRESH_BINARY );
        Imgproc.erode( whiteMat.clone(), whiteMat, Imgproc.getGaussianKernel( 5, 2 ) );

        channels.set( LIGHT_CHANNEL, whiteMat );

        if(zonedImg == null) zonedImg = hsvImage.clone();
        else hsvImage.copyTo(zonedImg);

        Core.merge( channels, zonedImg );

        //showImg = zonedImg.clone();
    }

    private void findLeds()
    {
        List<Mat> channels = new ArrayList<>();
        Core.split( zonedImg, channels );

        Mat d_value = channels.get( LIGHT_CHANNEL );

        if(hchy == null) hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(d_value, contours,
                             hchy, Imgproc.RETR_EXTERNAL,
                             Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        Rect bounded_box;

        skipCnt++;
        for (MatOfPoint wrapper : contours)
        {
            bounded_box = Imgproc.boundingRect(wrapper);

            if(bounded_box.area() < MIN_AREA) continue;

            leds.add(bounded_box);
            numLeds++;

            int ledX = bounded_box.x + bounded_box.width/2;
            int ledY = bounded_box.y + bounded_box.height/2;

            if(skipCnt % SKIP == 0)
            {
                RobotLog.ii(TAG, "LED at %5d, %5d", ledX, ledY);
            }
        }
    }
}
