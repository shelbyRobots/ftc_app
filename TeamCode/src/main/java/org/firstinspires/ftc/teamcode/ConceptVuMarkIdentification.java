/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.Frame;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static android.R.attr.height;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Concept: VuMark Id", group ="Concept")
//@Disabled
public class ConceptVuMarkIdentification extends LinearOpMode
{

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

//    /**
//     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//     * localization engine.
//     */
    //ZZVuforiaLocalizer vuforia;

    @Override
    public void runOpMode()
    {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        //ZZint cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //ZZVuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        //ZZparameters.vuforiaLicenseKey =
        //ZZ        "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
        //ZZ                "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
        //ZZ                "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
        //ZZ                "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
        //ZZ                "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
        //ZZ                "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
        //ZZ                "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
        //ZZ                "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        //ZZparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //ZZthis.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        //ZZVuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        //ZZVuforiaTrackable relicTemplate = relicTrackables.get(0);
        //ZZrelicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //ZZVuforia.setFrameFormat( PIXEL_FORMAT.RGB565, true );
        //ZZvuforia.setFrameQueueCapacity(10);

        tracker = new ImageTracker();

        imgProc = new BeaconDetector();
        bd = (BeaconDetector) imgProc;
        bf = (BeaconFinder) imgProc;

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        //ZZrelicTrackables.activate();

        //ZZOpenGLMatrix pose = OpenGLMatrix.identityMatrix();
        //ZZOpenGLMatrix rawpose = OpenGLMatrix.identityMatrix();
        while (opModeIsActive())
        {
            ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            tracker.setActive(true);

            Point2d sensedBotPos = null;
            double  sensedFldHdg;
            Bitmap bmap = null;
            while(sensedBotPos == null && itimer.milliseconds() < 10000)
            {
                tracker.updateRobotLocationInfo();
                sensedBotPos = tracker.getSensedPosition();

                if(sensedBotPos != null)
                {
                    curPos = sensedBotPos;
                    sensedFldHdg = tracker.getSensedFldHeading();
                    curHdg = sensedFldHdg;
                    tracker.getImage();
                    break;
                }
                sleep(50);
            }

            tracker.setActive(false);

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            //ZZRelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            //ZZif (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            //ZZ     telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            //ZZ     pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            //ZZ     telemetry.addData("Pose", format(pose));

            //ZZ     rawpose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            //ZZ     if (pose != null) {
            //ZZ         VectorF trans = pose.getTranslation();
            //ZZ         Orientation rot = Orientation.getOrientation(
            //ZZ                 pose,
            //ZZ                 AxesReference.EXTRINSIC,
            //ZZ                 AxesOrder.XYZ,
            //ZZ                 AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            //ZZ         double tX = trans.get(0);
            //ZZ         double tY = trans.get(1);
            //ZZ         double tZ = trans.get(2);

            // Extract the rotational components of the target relative to the robot
            //ZZ         double rX = rot.firstAngle;
            //ZZ         double rY = rot.secondAngle;
            //ZZ         double rZ = rot.thirdAngle;

            //ZZ         RobotLog.dd("SJH", "Trans: %10.4f %10.4f %10.4f", tX, tY, tZ);
            //ZZ         RobotLog.dd("SJH", "Rot:   %10.4f %10.4f %10.4f", rX, rY, rZ);
            //ZZ     }
            //ZZ }
            //ZZ else {
            //ZZ     telemetry.addData("VuMark", "not visible");

            //ZZ     telemetry.addData("Trackable",
            //ZZ        ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).isVisible() ?
            //ZZ        "is visible" : "not visible");
            //ZZ}

            telemetry.update();

            //ZZimgProc.startSensing();

            //ZZbd.setBitMap(retImg);

            //ZZBeaconFinder.BeaconSide bs = bd.getRedPosSide();
            //ZZRobotLog.dd("SJH", "Redside = " + bs);
            //ZZimgProc.stopSensing();

            sleep(10);
        }
    }


    public void doStuff()
    {
        RobotLog.ii("SJH", "FIND BEACON ORDER!!!");
        int timeout = 1000;
        BeaconFinder.LightOrder ord = BeaconFinder.LightOrder.UNKNOWN;
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        tracker.setFrameQueueSize(10);
        tracker.setActive(true);
        while  ((ord == BeaconFinder.LightOrder.UNKNOWN ||
                         ord == BeaconFinder.LightOrder.RED_RED ||
                         ord == BeaconFinder.LightOrder.BLUE_BLUE) &&
                        itimer.milliseconds() < timeout)
        {
            Bitmap bmap = tracker.getImage();
            if(bmap != null)
            {
                bd.setBitmap(bmap);
                ord = bd.getLightOrder();
            }
            sleep(50);
        }
        tracker.setActive(false);
        tracker.setFrameQueueSize(0);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private ImageTracker tracker;

    private int width, height;

    private static final int FRAME_SIZE = 16;
//    private static final int FRAME_SIZE = 8;
//    private static final int FRAME_SIZE = 64;

    private static final int FRAME_WIDTH = 3 * FRAME_SIZE;
    private static final int FRAME_HEIGHT = 4 * FRAME_SIZE;

    private int widthRequest = FRAME_WIDTH;
    private int heightRequest = FRAME_HEIGHT;

    private static Point2d curPos = null;
    private static double  curHdg = 0.0;

    private ImageProcessor imgProc = null;
    private BeaconDetector bd;
    private BeaconFinder bf;
    private Bitmap rgbImage = null;
    private VuforiaLocalizer.CloseableFrame frame = null;
}
