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
package org.firstinspires.ftc.teamcode.test;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.image.BeaconDetector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.util.Point2d;

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
        tracker = new ImageTracker(hardwareMap,
                                   telemetry,
                                   VuforiaInitializer.Challenge.RR);

        //imgProc = new BeaconDetector();
        //bd = (BeaconDetector) imgProc;
        //bf = (BeaconFinder) imgProc;

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            tracker.setActive(true);

            Point2d sensedBotPos = null;
            double  sensedFldHdg = 0.0;
            Bitmap bmap = null;
            int frm = 0;
            while(sensedBotPos == null && itimer.milliseconds() < 1000)
            {
                tracker.updateRobotLocationInfo();
                sensedBotPos = tracker.getSensedPosition();

                if(sensedBotPos != null)
                {
                    curPos = sensedBotPos;
                    sensedFldHdg = tracker.getSensedFldHeading();
                    curHdg = sensedFldHdg;
                    //rgbImage = tracker.getImage();
                    break;
                }
                frm++;
                sleep(50);
            }

            if ( sensedBotPos != null )
            {
                double t = itimer.seconds();
                RobotLog.ii("SJH", "Senesed Pos: %s %5.2f %2.3f", sensedBotPos, sensedFldHdg, t);
                RobotLog.ii("SJH", "IMG %s frame %d", tracker.getLocString(), frm);
                telemetry.addData("SLOC", "SLOC: %s %4.1f", sensedBotPos, sensedFldHdg);
                telemetry.addData("IMG", "%s  frame %d", tracker.getLocString(), frm);
            }

            tracker.setFrameQueueSize(10);
            Bitmap rgbImage = tracker.getImage();

            if(rgbImage == null) continue;
            bd.setBitmap(rgbImage);
            tracker.setFrameQueueSize(0);
            tracker.setActive(false);

            telemetry.update();

            //ZZimgProc.startSensing();

            //ZZbd.setBitmap(rgbImage);

            //ZZBeaconFinder.BeaconSide bs = bd.getRedPosSide();
            //ZZRobotLog.dd("SJH", "Redside = " + bs);
            //ZZimgProc.stopSensing();

            sleep(10);
        }
        tracker.setActive(false);
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

    //private ImageProcessor imgProc = null;
    private BeaconDetector bd = new BeaconDetector();
    //private BeaconFinder bf;
    private Bitmap rgbImage = null;
    private VuforiaLocalizer.CloseableFrame frame = null;
}
