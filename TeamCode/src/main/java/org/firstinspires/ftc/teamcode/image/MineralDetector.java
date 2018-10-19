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
import java.util.Iterator;
import java.util.List;

import hallib.HalDashboard;

/**
 * Created by crazy on 9/20/2018.
 */

@SuppressWarnings("unused")
public class MineralDetector extends Detector {
//        private static final double THRESHOLD = 0.4;
//        private static final int BINS = 8;
//        private static final float MIN_VALUE = 0.0f;
//        private static final float MAX_VALUE = 255.0f;
        private static MineralDetector.Position foundPosition = MineralDetector.Position.NONE;

        private HalDashboard dashboard;
        private static final String TAG = "SJH_MCD";

        public enum Position {
            CENTER, RIGHT, LEFT, NONE
        }

    public MineralDetector()
        {
            dashboard = CommonUtil.getInstance().getDashboard();
        }

    public void logTelemetry()
    {
        dashboard.displayPrintf(4,"Mineral Location Detected: %s", foundPosition);
    }

    public void logDebug()
    {
        RobotLog.ii(TAG, "Mineral Location Detected %s", foundPosition);
    }

    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    public MineralDetector.Position getMineralPos()
    {
        return foundPosition;
    }

    private void extract()
    {
        RobotLog.dd(TAG, "MineralDetector.extract()");
        //GripPipeline gpl = new GripPipeline();
        GripPipelineLonger gpl = new GripPipelineLonger();

        saveImage(showImg);

        gpl.process(showImg);

        saveImage(gpl.roiMat());
        saveImage(gpl.resizeImageOutput());
        saveImage(gpl.blurOutput());
        saveImage(gpl.hsvThresholdOutput());
        saveImage(gpl.cvErodeOutput());
        saveImage(gpl.cvDilateOutput());

        ArrayList<MatOfPoint> cntrs = gpl.convexHullsOutput();

        Iterator<MatOfPoint> each = cntrs.iterator();

        Rect bounded_box;
        int found_x_ptr = 0;
        foundPosition = Position.LEFT;

        if(cntrs.size() == 0)
        {
            RobotLog.dd(TAG, "No Countours.  Assuming left is gold");
            return;
        }
        else if(cntrs.size() > 1)
        {
            RobotLog.ee(TAG, "Too Many Countours.  I don't know which is gold");
            foundPosition = Position.CENTER;
            return;
        }

        for (MatOfPoint pts : cntrs)
        {
            MatOfPoint contour = each.next();
            bounded_box = Imgproc.boundingRect(contour);
            found_x_ptr = (bounded_box.width/2 + bounded_box.x);
        }

        RobotLog.dd(TAG, "Center of Gold = " + found_x_ptr);

        int midX = gpl.cvDilateOutput().width()/2;

        if (found_x_ptr < midX)
        {
            foundPosition = MineralDetector.Position.CENTER;
            //Yay it in deh center!!!!!!!!!!
        } else
        {
            foundPosition = MineralDetector.Position.RIGHT;
            //Yay it in deh right!!!!!!!!!!!
        }
    }

    private void extract_old() {
        Mat hsvImage = new Mat(showImg.width(), showImg.height(), showImg.type());
        Mat fltImage = new Mat();
        Mat yellow_areas = new Mat();


        Imgproc.cvtColor(showImg, hsvImage, Imgproc.COLOR_RGB2HSV);
/// Apply mask
        Core.inRange(hsvImage, new Scalar(0, 128, 73), new Scalar(44, 255, 255), yellow_areas);
// find contuors
        // merge nearby and find min rectangle
        Mat hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(yellow_areas, contours, hchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Iterator<MatOfPoint> each = contours.iterator();
        double maxArea = 0;
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
        }

        each = contours.iterator();
        Rect bounded_box;
        int found_x_ptr = 0;
        while (each.hasNext()) {

            MatOfPoint contour = each.next();

            bounded_box = Imgproc.boundingRect(contour);

            if (bounded_box.height > 200 && bounded_box.height < 500 &&
                    bounded_box.width > 200 && bounded_box.width < 500) {

                found_x_ptr = (bounded_box.width/2 + bounded_box.x);

            }
        }
        if (found_x_ptr < 1632) {
            foundPosition = MineralDetector.Position.CENTER;
            //Yay it in deh center!!!!!!!!!!
        } else if (found_x_ptr > 1632) {
            foundPosition = MineralDetector.Position.RIGHT;
            //Yay it in deh right!!!!!!!!!!!
        } else {
            foundPosition = Position.LEFT;
            //dang it. It in left...
        }
    }
}
