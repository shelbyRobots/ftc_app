package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.opencv.core.Mat;

import java.util.Random;

import hallib.HalDashboard;

/**
 * Created by crazy on 9/27/2017.
 */

public class MineralLocationDetector extends Detector {
    private static final double THRESHOLD = 0.4;
    private static final int BINS = 8;
    private static final float MIN_VALUE = 0.0f;
    private static final float MAX_VALUE = 255.0f;
    private Position mineralPos = Position.CENTER;

    private HalDashboard dashboard;
    private static final String TAG = "SJH_MLD";

    public enum Position
    {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    public MineralLocationDetector()
    {
        dashboard = CommonUtil.getInstance().getDashboard();
        //TODO:  Temporary - remove
        Random rand = new Random();
        int randomNum = rand.nextInt(3);
        mineralPos = Position.values()[randomNum];
        RobotLog.dd(TAG, "CTOR randomMineralPos = " + mineralPos);
    }

    static int lcnt = 0;
    public void logTelemetry()
    {
        dashboard.displayPrintf(4,"Gold Detected: %s", mineralPos);
        dashboard.displayPrintf(7,"lcnt   %d", lcnt);
    }

    static int dcnt = 0;
    public void logDebug()
    {
        RobotLog.ii(TAG, "Mineral Detected %s %d", mineralPos, dcnt++);
    }

    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    public Position getMineralPos() { return mineralPos; }

    private void extract() {
//        Mat hsvImage = new Mat(showImg.width(), showImg.height(), showImg.type());
//        Mat histHue = new Mat();
//        Mat histSaturation = new Mat();
//
//        Imgproc.cvtColor(showImg, hsvImage, Imgproc.COLOR_RGB2HSV);
//        List<Mat> channels = new ArrayList<>();
//        Core.split(hsvImage, channels);
//
//        //Histogram for hue
//        Imgproc.calcHist(Arrays.asList( new Mat[]{channels.get(0)} ), new MatOfInt(0),
//                new Mat(), histHue, new MatOfInt(BINS), new MatOfFloat(MIN_VALUE, 179));
//
//        //Histogram for saturation
//        Imgproc.calcHist(Arrays.asList( new Mat[]{channels.get(1)} ), new MatOfInt(0),
//                new Mat(), histSaturation, new MatOfInt(BINS), new MatOfFloat(MIN_VALUE, MAX_VALUE));
//
//
//        double sum = Core.sumElems(histHue).val[0];
//        double[] values = new double[histHue.height()+histSaturation.height()];
//
//        RobotLog.dd(TAG, "SUM_Hue %f", sum);
//        RobotLog.dd(TAG, "HueMat %d x %d", histHue.width(), histHue.height());
//        int k = 0;
//        for (int i = 0; i < histHue.height(); ++i ) {
//            values[k] = histHue.get(i, 0)[0]/sum;
//            RobotLog.dd(TAG, "Val_hue %d %f", k, values[k]);
//            k++;
//        }
//        sum = Core.sumElems(histSaturation).val[0];
//        //RobotLog.dd(TAG, "SUM_Sat %f", sum);
//        //RobotLog.dd(TAG, "SatMat %d x %d", histSaturation.width(), histSaturation.height());
//        for ( int i = 0; i < histSaturation.height(); ++i) {
//            values[k] = histSaturation.get(i, 0)[0]/sum;
//            //RobotLog.dd(TAG, "Val_sat %d %f", k, values[k]);
//            k++;
//        }
//        // 0 + 7 == red
//        // 4 + 5 == blue
//       double total = 0.0;
//
//        for ( int i = 0; i<BINS; i++){
//
//            total+=values[i];
//            RobotLog.dd(TAG, "Total %4.2f", total);
//        }
//        double red =  values[0] + values[7];
//        double blue = values[4] + values[5];
//        double redp = red/total;
//        double bluep = blue/total;
//        redPct = redp;
//        bluPct = bluep;
//
//        if(redp >= THRESHOLD && redp > bluep){
//            //yayRed!!! :D
//            foundColor = Color.RED;
//        }
//        else if(bluep >= THRESHOLD && bluep > redp)
//        {
//            //yayBlue!!! :D
//            foundColor = Color.BLUE;
//        }
//        else{
//            //booNone!!!:(
//            foundColor = Color.NONE;
//        }
    }
}
