package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

@SuppressWarnings("WeakerAccess, unused")
public class ImageTracker
{
    public ImageTracker(HardwareMap hardwareMap,
                        Telemetry telemetry,
                        VuforiaInitializer.Challenge challenge)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.challenge = challenge;
        setupVuforia();
    }

    private void setupVuforia()
    {
        vInit = new VuforiaInitializer();
        vuforia = vInit.getLocalizer(hardwareMap, useScreen);

        trackables = vInit.setupTrackables(challenge);
    }

    @SuppressWarnings("unused")
    public OpenGLMatrix getRobotLocation()
    {
        /*
          getUpdatedRobotLocation() will return null if no new information is available
          since the last time that call was made, or if the trackable is not currently
          visible.
          getRobotLocation() will return null if the trackable is not currently visible.
         */
        OpenGLMatrix robotLocationTransform = null;
        for (VuforiaTrackable trackable : trackables)
        {
            VuforiaTrackableDefaultListener l =
                    (VuforiaTrackableDefaultListener) trackable.getListener();

            RobotLog.dd(TAG, "Trackable " + trackable.getName() + " " + l.isVisible());
            telemetry.addData(trackable.getName(), l.isVisible() ? "Visible" : "Not Visible");

            robotLocationTransform = l.getUpdatedRobotLocation();
            if (robotLocationTransform != null)
            {
                lastVisName = trackable.getName();
                break;
            }
        }
        return robotLocationTransform;
    }

    @SuppressWarnings("unused")
    public OpenGLMatrix getImagePose(VuforiaTrackable target, boolean raw)
    {
        VuforiaTrackableDefaultListener l = (VuforiaTrackableDefaultListener) target.getListener();
        if (!l.isVisible()) return null;
        OpenGLMatrix pose;
        String poseType = "";
        if (!raw)
        {
            pose = l.getPose();
        } else
        {
            pose = l.getRawPose();
            if (pose == null) return null;

            poseType += "Raw";

            getImageCorners(pose);
        }
        if (pose != null)
        {
            RobotLog.dd("SJH", poseType + "Pose: " + VuforiaInitializer.getLocString(pose));
        }
        return pose;
    }

    //public OpenGLMatrix getRobotLocation()
    public void updateRobotLocationInfo()
    {
        /*
          getUpdatedRobotLocation() will return null if no new information is available
          since the last time that call was made, or if the trackable is not currently
          visible.
          getRobotLocation() will return null if the trackable is not currently visible.
         */
        OpenGLMatrix robotLocationTransform;
        for (VuforiaTrackable trackable : trackables)
        {
            VuforiaTrackableDefaultListener l =
                    (VuforiaTrackableDefaultListener) trackable.getListener();
            robotLocationTransform = l.getUpdatedRobotLocation();

            if (robotLocationTransform != null)
            {
                lastVisName = trackable.getName();
                float xyz[] = robotLocationTransform.getTranslation().getData();
                currPos = new Point2d(xyz[0] / MM_PER_INCH, xyz[1] / MM_PER_INCH);
                currOri = Orientation.getOrientation(robotLocationTransform,
                        AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
                currYaw = (double) currOri.firstAngle;

                if (keyLoc == RelicRecoveryVuMark.UNKNOWN &&
                    RelicRecoveryVuMark.from(trackable) != RelicRecoveryVuMark.UNKNOWN)
                {
                    keyLoc = RelicRecoveryVuMark.from(trackable);
                    RobotLog.ii("SJH", "VuMark KEY = " + keyLoc);
                    getImagePose(trackable, true);
                    getImagePose(trackable, false);
                    telemetry.addData("VuMark", keyLoc);
                    if (breakOnVumarkFound) vInit.setActive(false);
                }

                break;
            } else
            {
                currPos = null;
                currOri = null;
                currYaw = null;
            }
        }
    }

    public Point2d getSensedPosition()
    {
        return currPos;
    }

    public double getSensedFldHeading()
    {
        return currYaw;
    }

    public String getLocString()
    {
        String locStr = null;
        if (currPos != null && currYaw != null)
        {
            locStr = "";
            if (lastVisName != null) locStr = String.format(Locale.US,
                    "%10s ", lastVisName);

            locStr += String.format(Locale.US,
                    "POS: %5.2f, %5.2f  ROT: %4.1f",
                    currPos.getX() / MM_PER_INCH, currPos.getY() / MM_PER_INCH,
                    currYaw);
        }
        return locStr;
    }

    public Bitmap getImage()
    {
        VuforiaLocalizer.CloseableFrame frame = null;
        try
        {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e)
        {
            RobotLog.ee("SJH", "InterruptedException in ImageTracker.getImage");
        }

        if (frame == null)
        {
            RobotLog.ii("SJH", "getImage frame null");
            return null;
        }

        long numImages = frame.getNumImages();

        Image imgdata = null;
        for (int i = 0; i < numImages; i++)
        {
            int format = frame.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565)
            {
                imgdata = frame.getImage(i);
                break;
            }
        }

        if (imgdata == null)
        {
            RobotLog.ii("SJH", "imgData null");
            return null;
        }

        imgW = imgdata.getWidth();
        imgH = imgdata.getHeight();
        Bitmap.Config imgT = Bitmap.Config.RGB_565;
        if (rgbImage == null) rgbImage = Bitmap.createBitmap(imgW, imgH, imgT);

        rgbImage.copyPixelsFromBuffer(imgdata.getPixels());

        frame.close();

        return rgbImage;
    }

    /**
     * @param rawPose the pose of the beacon image VuforiaTrackable object
     * @return the list of 4 Point2d for the trackable image corners
     */
    public List<Point2d> getImageCorners(OpenGLMatrix rawPose)
    {
        Matrix34F rawPoseMx = new Matrix34F();

        OpenGLMatrix poseTransposed = rawPose.transposed();

        if (poseTransposed == null) return null;
        rawPoseMx.setData(Arrays.copyOfRange(poseTransposed.getData(), 0, 12));

        CameraCalibration camCal = vuforia.getCameraCalibration();
        //top left, top right, bottom left, bottom right
        List<Vec2F> vec2fList = Arrays.asList(
           Tool.projectPoint(camCal, rawPoseMx,
                   new Vec3F(-JEWEL_TARGET_WIDTH / 2, JEWEL_TARGET_HEIGHT / 2, 0)),  //top left)
           Tool.projectPoint(camCal, rawPoseMx,
                   new Vec3F(JEWEL_TARGET_WIDTH / 2, JEWEL_TARGET_HEIGHT / 2, 0)),   //top right
           Tool.projectPoint(camCal, rawPoseMx,
                   new Vec3F(-JEWEL_TARGET_WIDTH / 2, -JEWEL_TARGET_HEIGHT / 2, 0)), //bottom left
           Tool.projectPoint(camCal, rawPoseMx,
                   new Vec3F(JEWEL_TARGET_WIDTH / 2, -JEWEL_TARGET_HEIGHT / 2, 0))   //bottom right
        );

        RobotLog.ii(TAG, "unscaled frame size: " + imgW + " , " + imgH);
        RobotLog.ii(TAG, "unscaled tl: " + vec2fList.get(0));
        RobotLog.ii(TAG, "unscaled tr: " + vec2fList.get(1));
        RobotLog.ii(TAG, "unscaled bl: " + vec2fList.get(2));
        RobotLog.ii(TAG, "unscaled br: " + vec2fList.get(3));
//
//        //get average width from the top width and bottom width
////        double w = ((vec2fList.get(1).getData()[1] - vec2fList.get(0).getData()[1]) + (vec2fList.get(3).getData()[1] - vec2fList.get(2).getData()[1])) / 2;
//        //same for height
////        double h = ((vec2fList.get(2).getData()[0] - vec2fList.get(0).getData()[0]) + (vec2fList.get(3).getData()[0] - vec2fList.get(1).getData()[0])) / 2;
//
////        Log.i(TAG, "beacon picture size: " + new Vector2D(w, h));
//
        //convert the Vec2F list to a Vector2D list and scale it to match the requested frame size
        List<Point2d> corners = new ArrayList<>();
        for (Vec2F vec2f : vec2fList)
        {
            corners.add(new Point2d((imgH - vec2f.getData()[1]) * widthRequest / imgH,
                                     vec2f.getData()[0] * heightRequest / imgW));
        }

        return corners;
    }

    public void setFrameQueueSize(int size)
    {
        vuforia.setFrameQueueCapacity(size);
    }

    public void setActive(boolean active)
    {
        vInit.setActive(active);
    }

    // Vuforia units are mm = units used in XML for the trackables
    private static final float MM_PER_INCH        = 25.4f;
    private static final String TAG = "SJH ImageTracker";

    private static final int JEWEL_TARGET_WIDTH = 127 * 2;
    private static final int JEWEL_TARGET_HEIGHT = 92 * 2;

    private int imgW;
    private int imgH;


    private static final int FRAME_SIZE = 16;
//    private static final int FRAME_SIZE = 8;
//    private static final int FRAME_SIZE = 64;

    private static final int FRAME_WIDTH = 3 * FRAME_SIZE;
    private static final int FRAME_HEIGHT = 4 * FRAME_SIZE;

    private int widthRequest = FRAME_WIDTH;
    private int heightRequest = FRAME_HEIGHT;

    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private List<VuforiaTrackable> trackables;
    private RelicRecoveryVuMark keyLoc = RelicRecoveryVuMark.UNKNOWN;

    VuforiaInitializer vInit = null;
    VuforiaInitializer.Challenge challenge = VuforiaInitializer.Challenge.RR;

    private boolean breakOnVumarkFound = true;

    private Point2d currPos = null;
    private Double  currYaw = null;
    private Orientation currOri = null;
    private String lastVisName = "UNKNOWN";
    private boolean useScreen = true;

    private Bitmap rgbImage = null;
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
}
