package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import hallib.HalDashboard;

@SuppressWarnings({"WeakerAccess, unused", "FieldCanBeLocal"})
public class ImageTracker
{
    public ImageTracker(VuforiaInitializer.Challenge challenge)
    {
        com = CommonUtil.getInstance();
        dashboard = com.getDashboard();
        this.challenge = challenge;
        setupVuforia();
    }

    private void setupVuforia()
    {
        vInit = com.getVuforiaInitializer();
        trackables = vInit.setupTrackables(challenge);
        vuforia = com.getVuforiaLocalizer();
    }

    public RelicRecoveryVuMark getKeyLoc() {return keyLoc;}

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
            dashboard.displayPrintf(5, "%s", trackable.getName(),
                                             l.isVisible() ? "Visible" : "Not Visible");

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
        }
        else
        {
            pose = l.getRawPose();
            poseType += "Raw";
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

                OpenGLMatrix rawPose;
                if (keyLoc == RelicRecoveryVuMark.UNKNOWN &&
                    RelicRecoveryVuMark.from(trackable) != RelicRecoveryVuMark.UNKNOWN)
                {
                    keyLoc = RelicRecoveryVuMark.from(trackable);
                    RobotLog.ii("SJH", "VuMark KEY = " + keyLoc);
                    rawPose = getImagePose(trackable, true);
                    dashboard.displayPrintf(6, "VuMark key = %s", keyLoc);
                    RobotLog.dd(TAG, "Rawpose: " + format(rawPose));

                    Bitmap fullPic = getImage();
                    List<Point2d>  trackablePixCorners =
                            getTrackableCornersInCamera(rawPose);
                    List<Point2d> jewelBoxPixCorners =
                            getJewelBoxCornersInCamera(rawPose, false);
                            //getCropCorners(trackablePixCorners);
                   Bitmap jewelImg = getCroppedImage(jewelBoxPixCorners, fullPic);

                    //TODO - pass the jewelImage to a particle detector

                    if (breakOnVumarkFound)
                    {
                        vInit.setActive(false);
                        break;
                    }
                }

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

    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ?
                       transformationMatrix.formatAsTransform() :
                       "null";
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

        int imgW = imgdata.getWidth();
        int imgH = imgdata.getHeight();
        Bitmap.Config imgT = Bitmap.Config.RGB_565;
        if (rgbImage == null) rgbImage = Bitmap.createBitmap(imgW, imgH, imgT);

        rgbImage.copyPixelsFromBuffer(imgdata.getPixels());

        frame.close();

        return rgbImage;
    }

    private Bitmap getCroppedImage(List<Point2d> cropCorners, Bitmap fullImage)
    {
        Point2d ptl = cropCorners.get(0);
        Point2d ptr = cropCorners.get(1);
        Point2d pbr = cropCorners.get(2);
        Point2d pbl = cropCorners.get(3);

        int minX = (int)Math.min(Math.min(ptl.getX(), pbl.getX()),
                                 Math.min(ptr.getX(), ptl.getX()));
        int maxX = (int)Math.max(Math.max(ptl.getX(), pbl.getX()),
                                 Math.max(ptr.getX(), ptl.getX()));
        int minY = (int)Math.min(Math.min(ptl.getY(), pbl.getY()),
                                 Math.min(ptr.getY(), ptl.getY()));
        int maxY = (int)Math.max(Math.max(ptl.getY(), pbl.getY()),
                                 Math.max(ptr.getY(), ptl.getY()));

        int w = maxX - minX;
        int h = maxY - minY;
        //Rect aoi = new Rect(minX, minY, w, h);

        int fullImgW = fullImage.getWidth();
        int fullImgH = fullImage.getHeight();

        if(minX + w > fullImgW) w = fullImgW - minX;
        if(minY + h > fullImgH) h = fullImgH - minY;

        RobotLog.dd(TAG, "Cropping " + w + "x" + h + " out of " + fullImgW + "x" + fullImgH);

        Bitmap bitmap = Bitmap.createBitmap(fullImage, minX, minY, w, h);

        String fileName = "sbh_test.png";
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        //String directoryPath  = Environment.getExternalStorageDirectory().getPath() +
        //                                "/FIRST/DataLogger";
        //String filePath       = directoryPath + "/" + fileName ;
        File dest = new File(path, fileName);

        //File dest = new File(filePath);
        FileOutputStream out;
        try
        {
            out = new FileOutputStream(dest);
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.close();
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "ERROR saving file. " + e);
        }

        return bitmap;
    }

    private List<Point2d> getCropCorners(List<Point2d> trackableCorners)
    {
        Point2d ptl = trackableCorners.get(0);
        Point2d ptr = trackableCorners.get(1);
        Point2d pbr = trackableCorners.get(2);
        Point2d pbl = trackableCorners.get(3);

        //get average width and height
        double pixw = Math.abs(((ptr.getX() - ptl.getX()) + (pbr.getX() - pbl.getX())) / 2);
        double pixh = Math.abs(((ptl.getY() - pbl.getY()) + (ptr.getY() - pbr.getY())) / 2);

        double pixPerMmX = pixw/TARGET_WIDTH;
        double pixPerMmY = pixh/TARGET_HEIGHT;

        RobotLog.ii(TAG, "trackable pixel size: " + pixw + " " + pixh);
        RobotLog.ii(TAG, "PixPerMm: " + pixPerMmX + " " + pixPerMmY);

        //Left jewel center is aligned with right edge of tracker image sheet
        //Jewels are 6in center to center, and are (3.75/2)" radius.
        //So, left edge of left jewel is a ball radius (3.75/2)" to the left of image BR.
        //    right edge of right jewel is 7.875" right of image right edge.
        //
        //Image bottom edge is 1.5" from floor.
        //Neglecting tile thickness for now, we will use this for Y
        //The jewel "box" is 9.75" wide by ~4" high

        double distImgRtoJewelR = (6 + 3.75/2) * MM_PER_INCH;
        double distImgBtoJewelB = 1.5 * MM_PER_INCH;
        double distImgRtoJewelL = (3.75/2) * MM_PER_INCH;
        double jewelPixDistR = distImgRtoJewelR * pixPerMmX;
        double jewelPixDistL = distImgRtoJewelL * pixPerMmX;
        double jewelPixDistB = distImgBtoJewelB * pixPerMmY;
        double jewelBoxWidth  = 9.75 * MM_PER_INCH;
        double jewelBoxHeight = 4.0  * MM_PER_INCH;
        int jewelPixWidth  = (int)(jewelBoxWidth  * pixPerMmX);
        int jewelPixHeight = (int)(jewelBoxHeight * pixPerMmY);

        Point2d jewelBoxBL = new Point2d(pbr.getX() - distImgRtoJewelL,
                                                pbr.getY() - distImgBtoJewelB);
        Point2d jewelBoxTL = new Point2d(jewelBoxBL.getX(),
                                         jewelBoxBL.getY() + jewelPixHeight);
        Point2d jewelBoxTR = new Point2d(jewelBoxTL.getX() + jewelBoxWidth,
                                         jewelBoxTL.getY());
        Point2d jewelBoxBR = new Point2d(jewelBoxTL.getX(),
                                         jewelBoxBL.getY());

        List<Point2d> cropCorners = new ArrayList<>();
        cropCorners.add(jewelBoxTL);
        cropCorners.add(jewelBoxTR);
        cropCorners.add(jewelBoxBR);
        cropCorners.add(jewelBoxBL);

        RobotLog.ii("TAG", "Jewel Box LL: " + jewelBoxBL);
        RobotLog.ii("TAG", "Jewel Box width:  " + jewelBoxWidth  + "(" + jewelPixWidth  + ")");
        RobotLog.ii("TAG", "Jewel Box height: " + jewelBoxHeight + "(" + jewelPixHeight + ")");

        RobotLog.ii(TAG, "Cropped points");
        RobotLog.ii(TAG, "tl: " + jewelBoxTL);
        RobotLog.ii(TAG, "tr: " + jewelBoxTR);
        RobotLog.ii(TAG, "bl: " + jewelBoxBL);
        RobotLog.ii(TAG, "br: " + jewelBoxBR);

        return cropCorners;
    }

    /**
     * @param rawPose the pose of the beacon image VuforiaTrackable object
     * @return the list of 4 Point2d for the trackable image corners
     */
    private List<Point2d> getTrackableCornersInCamera(OpenGLMatrix rawPose)
    {
        Matrix34F rawPoseMx = new Matrix34F();
        OpenGLMatrix poseTransposed = rawPose.transposed();

        final int TL = 0;
        final int TR = 1;
        final int BR = 2;
        final int BL = 3;

        if (poseTransposed == null) return null;
        rawPoseMx.setData(Arrays.copyOfRange(poseTransposed.getData(), 0, 12));

        List<Vec3F> trackableCorners = Arrays.asList(
                new Vec3F((float)-TARGET_WIDTH/2, (float) TARGET_HEIGHT/2, 0.0f),
                new Vec3F((float) TARGET_WIDTH/2, (float) TARGET_HEIGHT/2, 0.0f),
                new Vec3F((float) TARGET_WIDTH/2, (float)-TARGET_HEIGHT/2, 0.0f),
                new Vec3F((float)-TARGET_WIDTH/2, (float)-TARGET_HEIGHT/2, 0.0f)
        );

        //Project the trackable "real" corners in field or trackable picture local coords
        //to pixel coordinates in camera image.
        //To do this, use the inverse of the raw pose matrix and the corner locations
        //from the printed picture size
        CameraCalibration camCal = vuforia.getCameraCalibration();

        List<Vec2F> trackableImageCorners = Arrays.asList(
           Tool.projectPoint(camCal, rawPoseMx, trackableCorners.get(TL)), //top left)
           Tool.projectPoint(camCal, rawPoseMx, trackableCorners.get(TR)), //top right
           Tool.projectPoint(camCal, rawPoseMx, trackableCorners.get(BR)), //bottom left
           Tool.projectPoint(camCal, rawPoseMx, trackableCorners.get(BL))  //bottom right
        );

        //These are trackable corners in trackable sheet space - using Point2d for naming
        List<Point2d> trackableSheetCorners = new ArrayList<>(Arrays.asList(
                new Point2d("TSTL", -TARGET_WIDTH/2,  TARGET_HEIGHT/2),
                new Point2d("TSTR",  TARGET_WIDTH/2,  TARGET_HEIGHT/2),
                new Point2d("TSBR",  TARGET_WIDTH/2, -TARGET_HEIGHT/2),
                new Point2d("TSBL", -TARGET_WIDTH/2, -TARGET_HEIGHT/2)));

        //These are trackable corners in camera pixel space
        List<Point2d> trackablePixelCorners = new ArrayList<>(Arrays.asList(
                new Point2d("TPTL", trackableImageCorners.get(TL).getData()),
                new Point2d("TPTR", trackableImageCorners.get(TR).getData()),
                new Point2d("TPBR", trackableImageCorners.get(BR).getData()),
                new Point2d("TPBL", trackableImageCorners.get(BL).getData())));

        RobotLog.ii(TAG, "trackable size (mm): " + TARGET_WIDTH + " , " + TARGET_HEIGHT);
        RobotLog.ii(TAG, "Trackable corners in trackable sheet space");
        for(Point2d p : trackableSheetCorners)
        {
            RobotLog.ii(TAG, "" + p);
        }
        RobotLog.ii(TAG, "Trackable corners in pixel space");
        for(Point2d p : trackablePixelCorners)
        {
            RobotLog.ii(TAG, "" + p);
        }

        return trackablePixelCorners;
    }

    private List<Point2d> getJewelBoxCornersInCamera(OpenGLMatrix rawPose, boolean bothJewels)
    {
        Matrix34F rawPoseMx = new Matrix34F();
        OpenGLMatrix poseTransposed = rawPose.transposed();

        final int TL = 0;
        final int TR = 1;
        final int BR = 2;
        final int BL = 3;

        if (poseTransposed == null) return null;
        rawPoseMx.setData(Arrays.copyOfRange(poseTransposed.getData(), 0, 12));

        //The jewels will lie roughly along the line of the tracker
        //picture bottom.  The center of the left jewel
        //will be ~ at the lower right corner.
        //The left edge of the left jewel will be one jewel radius
        //"left" of the corner.  The right edge of the right jewel
        //will be 1 jewel radius + 6" beyond the end of the line.
        //The jewels will be +/- radius perpindicular to the line.
        //According to RR manual part 2, the bottom of the image is
        //1.5" above the floor, and tiles are 5/8" thick.
        //So, bottom of jewel sitting on tiles will be 5/8"
        //above floor.  1.5" trackable bottom - 5/8" jewel bottom
        //= -7/8"

        double tileToTrackableBottom = 1.5 - 0.625;
        double jewelRadius   = 3.75/2.0 * MM_PER_INCH;
        double jewelCtrToCtr = 6.0      * MM_PER_INCH;
        double jewelLeft   =  TARGET_WIDTH/2 - jewelRadius;
        double jewelRight  =  TARGET_WIDTH/2 + jewelCtrToCtr + jewelRadius;
        if(!bothJewels) jewelRight = jewelLeft + 2*jewelRadius;
        double jewelBottom = -TARGET_HEIGHT/2 - tileToTrackableBottom ;
        double jewelTop    =  jewelBottom + 2*jewelBottom;
        List<Vec3F> jewelCorners = Arrays.asList(
                new Vec3F((float)jewelLeft,  (float)jewelTop,    0.0f),
                new Vec3F((float)jewelRight, (float)jewelTop,    0.0f),
                new Vec3F((float)jewelRight, (float)jewelBottom, 0.0f),
                new Vec3F((float)jewelLeft,  (float)jewelBottom, 0.0f)
        );

        CameraCalibration camCal = vuforia.getCameraCalibration();

        List<Vec2F> jewelImageCorners = Arrays.asList(
                Tool.projectPoint(camCal, rawPoseMx, jewelCorners.get(TL)), //top left)
                Tool.projectPoint(camCal, rawPoseMx, jewelCorners.get(TR)), //top right
                Tool.projectPoint(camCal, rawPoseMx, jewelCorners.get(BR)), //bottom left
                Tool.projectPoint(camCal, rawPoseMx, jewelCorners.get(BL))  //bottom right
        );

        List<Point2d> jewelBoxSheetCorners = new ArrayList<>(Arrays.asList(
                new Point2d("JSTL", jewelLeft,  jewelTop ),
                new Point2d("JSTR", jewelRight, jewelTop),
                new Point2d("JSBR", jewelRight, jewelBottom ),
                new Point2d("JSBL", jewelLeft,  jewelBottom )
        ));

        List<Point2d> jewelBoxPixelCorners = new ArrayList<>(Arrays.asList(
                new Point2d("JPTL", jewelImageCorners.get(TL).getData()),
                new Point2d("JPTR", jewelImageCorners.get(TR).getData()),
                new Point2d("JPBR", jewelImageCorners.get(BR).getData()),
                new Point2d("JPBL", jewelImageCorners.get(BL).getData())
        ));

        RobotLog.ii(TAG, "Jewel box corners in trackable sheet space");
        for(Point2d p : jewelBoxSheetCorners)
        {
            RobotLog.ii(TAG, "" + p);
        }
        RobotLog.ii(TAG, "Jewel box corners in pixel space");
        for(Point2d p : jewelBoxPixelCorners)
        {
            RobotLog.ii(TAG, "" + p);
        }

        return jewelBoxPixelCorners;
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

    //Note: asset file has 304mm x 224mm (12"x8.8")for RR !?!?
    //Need to figure out what xml coordinates really mean
    private static final int TARGET_WIDTH = 127 * 2;
    private static final int TARGET_HEIGHT = 92 * 2;

    CommonUtil com;
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
    private HalDashboard dashboard;
    private boolean configureLayout = false;
}
