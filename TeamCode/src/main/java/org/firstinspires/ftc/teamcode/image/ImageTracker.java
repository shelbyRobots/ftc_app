package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
import org.firstinspires.ftc.teamcode.field.VvField;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

@SuppressWarnings("WeakerAccess, unused")
public class ImageTracker
{
    public ImageTracker()
    {
        setupTrackables();
        setupPhoneOnRobot();
    }

    private void setupTrackables()
    {
        //To see camera feedback, pass the view id
        //For competition, we don't want this - so use the no param ctor
        if (useScreen)
        {
            parameters = new
                                 VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        } else
        {
            parameters = new VuforiaLocalizer.Parameters();
        }

        //SJH Teams license
        parameters.vuforiaLicenseKey =
                "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
                        "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
                        "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
                        "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
                        "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
                        "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
                        "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
                        "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //Set the image sets to allow getting frames from vuforia
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(10);
        RobotLog.ii("SJH", "Vuforia LicKey: " + parameters.vuforiaLicenseKey);

        if (useVv)
        {
            vvImages = vuforia.loadTrackablesFromAsset("FTC_2016-17");
            //Wheels are on blue side closest to blue corner
            blueWheels = vvImages.get(0);
            blueWheels.setName("BlueWheels");

            //Legos are on blud side furthest from blue corner
            blueLegos = vvImages.get(2);
            blueLegos.setName("BlueLegos");

            //Tools are on red side furthest from red corner
            redTools = vvImages.get(1);
            redTools.setName("RedTools");

            //Gears are on red side closest to red corner
            redGears = vvImages.get(3);
            redGears.setName("RedGears");

            allTrackables.addAll(vvImages);

            redTools.setLocation(VvField.redToolsLocationOnField);
            RobotLog.ii(TAG, "Red Tools=%s", getLocString(redTools.getLocation()));

            redGears.setLocation(VvField.redGearsLocationOnField);
            RobotLog.ii(TAG, "Red Gears=%s", getLocString(redGears.getLocation()));

            blueWheels.setLocation(VvField.blueWheelsLocationOnField);
            RobotLog.ii(TAG, "Blue Wheels=%s", getLocString(blueWheels.getLocation()));

            blueLegos.setLocation(VvField.blueLegosLocationOnField);
            RobotLog.ii(TAG, "Blue Legos=%s", getLocString(blueLegos.getLocation()));
        }

        if (useRr)
        {
            rrImages = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            rrTemplate = rrImages.get(0);
            rrTemplate.setName("RelicTrack");

            allTrackables.addAll(rrImages);

            //Position on field not usable for RR - but set to identity for now
            rrTemplate.setLocation(OpenGLMatrix.identityMatrix());
            RobotLog.ii(TAG, "RR Template=%s", getLocString(rrTemplate.getLocation()));
        }
    }

    private void setupPhoneOnRobot()
    {
        OpenGLMatrix phoneLocationOnRobot = ShelbyBot.phoneLocationOnRobot;
        RobotLog.ii(TAG, "phone=%s", getLocString(phoneLocationOnRobot));

        /*
          A brief tutorial: here's how all the math is going to work:

          C = phoneLocationOnRobot     maps   phone coords        -> robot coords
          P = tracker.getPose()        maps   image target coords -> phone coords
          L = redTargetLocationOnField maps   image target coords -> field coords

          C.inverted()                 maps   robot coords -> phone coords
          P.inverted()                 maps   phone coords -> imageTarget coords

          L x P.inverted() x C.inverted() maps robot coords to field coords.

          @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        if (useVv)
        {
            ((VuforiaTrackableDefaultListener) redTools.getListener())
                    .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) redGears.getListener())
                    .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) blueLegos.getListener())
                    .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) blueWheels.getListener())
                    .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        if (useRr)
        {
            ((VuforiaTrackableDefaultListener) rrTemplate.getListener())
                    .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
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
        for (VuforiaTrackable trackable : allTrackables)
        {
            robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener())
                                             .getUpdatedRobotLocation();
            if (robotLocationTransform != null)
            {
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
            RobotLog.dd("SJH", poseType + "Pose: " + getLocString(pose));
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
        for (VuforiaTrackable trackable : allTrackables)
        {
            VuforiaTrackableDefaultListener l =
                    (VuforiaTrackableDefaultListener) trackable.getListener();
            robotLocationTransform = l.getUpdatedRobotLocation();

            if (robotLocationTransform != null)
            {
                lastVisName = trackable.getName();
                float xyz[] = robotLocationTransform.getTranslation().getData();
                currPos = new Point2d(xyz[0] / MM_PER_INCH, xyz[1] / MM_PER_INCH);
                RobotLog.ii("SJH", "Found Image " + lastVisName);
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
                    if (breakOnVumarkFound) setActive(false);
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

    public String getLocString(OpenGLMatrix mat)
    {
        String locStr = null;
        if (mat != null)
        {
            float xyz[] = mat.getTranslation().getData();
            Orientation ori = Orientation.getOrientation(mat,
                    AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

            locStr = String.format(Locale.US,
                    "POS: %5.2f, %5.2f, %5.2f ROT: %4.1f, %4.1f, %4.1f",
                    xyz[0] / MM_PER_INCH, xyz[1] / MM_PER_INCH, xyz[2] / MM_PER_INCH,
                    ori.firstAngle, ori.secondAngle, ori.thirdAngle);
        }
        return locStr;
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
                RobotLog.dd("SJH", "Image " + i + " format = " + format);
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
     * @return the list of 4 Vector2D objects that represent the points of the corners of the beacon image
     */
    public List<Point2d> getImageCorners(OpenGLMatrix rawPose)
    {
        Matrix34F rawPoseMx = new Matrix34F();

        OpenGLMatrix poseTransposed = rawPose.transposed();

        if (poseTransposed == null) return null;
        rawPoseMx.setData(Arrays.copyOfRange(poseTransposed.getData(), 0, 12));

        List<Point2d> imageCorners = null; //vuforia.getImageCorners(rawPose);
        return imageCorners;
    }
//    public List<Point2d> getImageCorners(Matrix34F rawPose)
//    {
//        CameraCalibration camCal = vuforia.getCameraCalibration();
//        //top left, top right, bottom left, bottom right
//        List<Vec2F> vec2fList = Arrays.asList(
//           Tool.projectPoint(camCal, rawPose,
//                   new Vec3F(-JEWEL_TARGET_WIDTH / 2, JEWEL_TARGET_HEIGHT / 2, 0)),  //top left)
//           Tool.projectPoint(camCal, rawPose,
//                   new Vec3F(JEWEL_TARGET_WIDTH / 2, JEWEL_TARGET_HEIGHT / 2, 0)),   //top right
//           Tool.projectPoint(camCal, rawPose,
//                   new Vec3F(-JEWEL_TARGET_WIDTH / 2, -JEWEL_TARGET_HEIGHT / 2, 0)), //bottom left
//           Tool.projectPoint(camCal, rawPose,
//                   new Vec3F(JEWEL_TARGET_WIDTH / 2, -JEWEL_TARGET_HEIGHT / 2, 0))   //bottom right
//        );
//
//        RobotLog.i(TAG, "unscaled frame size: " + imgW + " , " + imgH);
//        RobotLog.i(TAG, "unscaled tl: " + vec2fList.get(0));
//        RobotLog.i(TAG, "unscaled tr: " + vec2fList.get(1));
//        RobotLog.i(TAG, "unscaled bl: " + vec2fList.get(2));
//        RobotLog.i(TAG, "unscaled br: " + vec2fList.get(3));
////
////        //get average width from the top width and bottom width
//////        double w = ((vec2fList.get(1).getData()[1] - vec2fList.get(0).getData()[1]) + (vec2fList.get(3).getData()[1] - vec2fList.get(2).getData()[1])) / 2;
////        //same for height
//////        double h = ((vec2fList.get(2).getData()[0] - vec2fList.get(0).getData()[0]) + (vec2fList.get(3).getData()[0] - vec2fList.get(1).getData()[0])) / 2;
////
//////        Log.i(TAG, "beacon picture size: " + new Vector2D(w, h));
////
//        //convert the Vec2F list to a Vector2D list and scale it to match the requested frame size
//        List<Point2d> corners = new ArrayList<>();
//        for (Vec2F vec2f : vec2fList)
//        {
//            corners.add(new Point2d((imgH - vec2f.getData()[1]) * widthRequest / imgH,
//                                     vec2f.getData()[0] * heightRequest / imgW));
//        }
//
//        return corners;
//    }

    public void setActive(boolean active)
    {
        if(active)
        {
            if(useVv) vvImages.activate();
            if(useRr) rrImages.activate();
        }
        else
        {
            if(useVv) vvImages.deactivate();
            if(useRr) rrImages.deactivate();
        }
    }

    public void setFrameQueueSize(int size)
    {
        vuforia.setFrameQueueCapacity(size);
    }

    // Vuforia units are mm = units used in XML for the trackables
    private static final float MM_PER_INCH        = 25.4f;
    private static final String TAG = "SJH ImageTracker";

    private static final boolean useVv = false;
    private static final boolean useRr = true;

    private List<VuforiaTrackable> allTrackables = new ArrayList<>();

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
    private VuforiaTrackable blueWheels;
    private VuforiaTrackable blueLegos;
    private VuforiaTrackable redTools;
    private VuforiaTrackable redGears;
    private VuforiaTrackable rrTemplate;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables vvImages;
    private VuforiaTrackables rrImages;
    private RelicRecoveryVuMark keyLoc = RelicRecoveryVuMark.UNKNOWN;

    private boolean breakOnVumarkFound = true;

    private Point2d currPos = null;
    private Double  currYaw = null;
    private Orientation currOri = null;
    private String lastVisName = "";
    private boolean useScreen = true;

    private Bitmap rgbImage = null;
}
