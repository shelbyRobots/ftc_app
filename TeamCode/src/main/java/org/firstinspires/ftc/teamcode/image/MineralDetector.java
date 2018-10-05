package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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

public class MineralDetector extends Detector {
        private static final double THRESHOLD = 0.4;
        private static final int BINS = 8;
        private static final float MIN_VALUE = 0.0f;
        private static final float MAX_VALUE = 255.0f;
        public static MineralDetector.Position foundPosition = MineralDetector.Position.NONE;

        private double redPct = 0.0;
        private double bluPct = 0.0;

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
        dashboard.displayPrintf(4,"Color Detected: %s", foundPosition);
        dashboard.displayPrintf(5,"RedPct %4.2f", redPct);
        dashboard.displayPrintf(6,"BluPct %4.2f", bluPct);
    }

    public void logDebug()
    {
        RobotLog.ii(TAG, "Color Detected %s redPct %4.2f bluPct %4.2f",
                foundPosition, redPct, bluPct);
    }

    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    public MineralDetector.Position getMajorColor()
    {
        return foundPosition;
    }

    private void extract() {
        Mat hsvImage = new Mat(showImg.width(), showImg.height(), showImg.type());
        Mat fltImage = new Mat();
        Mat yellow_areas = new Mat();


        Imgproc.cvtColor(showImg, hsvImage, Imgproc.COLOR_RGB2HSV);
/// Apply mask
        Core.inRange(hsvImage, new Scalar(0, 128, 73), new Scalar(44, 255, 255),yellow_areas);
// find contuors
        // merge nearby and find min rectangle
        Mat hchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(hsvImage, contours, hchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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


    Mat cvImage;
    public void hightlightCorners(Bitmap fullImag, List<Point2d> pts)
    {
        int cvt = CvType.CV_8UC1;
        int inHeight = fullImag.getHeight();
        int inWidth = fullImag.getWidth();

        if(cvImage == null) cvImage = new Mat(inHeight, inWidth, cvt);

        Utils.bitmapToMat(fullImag, cvImage);

        for(Point2d pt : pts)
        {
            Point p = new Point(pt.getX(), pt.getY());
            Imgproc.circle(cvImage, p, 5, new Scalar(1, 0, 0));
            RobotLog.dd(TAG, "Corner at %4.2f %4.2f", p.x, p.y);
        }

        saveImage(cvImage);
    }
}
