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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.image.BeaconDetector;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.LedDetector;
import org.firstinspires.ftc.teamcode.image.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.Point2d;

import hallib.HalDashboard;

@Autonomous(name="Concept: VuMark Id", group ="Concept")
//@Disabled
public class VuMarkOpenCvTest extends InitLinearOpMode
{
    public static final String TAG = "SJH_VuMark";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, true, true, false, true);
        CommonUtil com = CommonUtil.getInstance();
        super.runOpMode();
        tracker = new ImageTracker(VuforiaInitializer.Challenge.RR);

        //bd = new BeaconDetector();
        RobotLog.dd("SJH_LEDA", "Creating new LedDetector");
        ld = new LedDetector();
        RobotLog.dd("SJH_LEDA", "Back from Creating new LedDetector");

        telemetry.update();
        waitForStart();

        RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;
        tracker.setActive(true);
        RobotLog.dd("SJH", "Finding VuMark first");
        while (opModeIsActive() && key == RelicRecoveryVuMark.UNKNOWN)
        {
            ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            Point2d sensedBotPos = null;
            double  sensedFldHdg = 0.0;
            Bitmap bmap = null;
            int frm = 0;
            //while(sensedBotPos == null && itimer.milliseconds() < 1000)
            //{
                tracker.updateRobotLocationInfo();
                sensedBotPos = tracker.getSensedPosition();

                if(sensedBotPos != null)
                {
                    curPos = sensedBotPos;
                    sensedFldHdg = tracker.getSensedFldHeading();
                    curHdg = sensedFldHdg;
                    key = tracker.getKeyLoc();
                    //rgbImage = tracker.getImage();
            //        break;
                }
                frm++;
            //    sleep(50);
            //}

            if ( sensedBotPos != null )
            {
                double t = itimer.seconds();
                RobotLog.ii("SJH", "Senesed Pos: %s %5.2f %2.3f", sensedBotPos, sensedFldHdg, t);
                RobotLog.ii("SJH", "IMG %s frame %d", tracker.getLocString(), frm);
                dashboard.displayPrintf(1, "SLOC: %s %4.1f", sensedBotPos, sensedFldHdg);
                dashboard.displayPrintf(2, "IMG", "%s  frame %d", tracker.getLocString(), frm);
            }

            telemetry.update();

        }

        RobotLog.ii(TAG, "KEY : " + key);
        dashboard.displayPrintf(0, "KEY: " + key);
        RobotLog.dd("SJH", "Turn off image tracker");
        tracker.setActive(false);
        RobotLog.dd("SJH", "Set qsize to get frames");
        tracker.setFrameQueueSize(1);
        RobotLog.dd("SJH", "Start LD sensing");
        ld.startSensing();

        while(opModeIsActive())
        {
            Bitmap rgbImage = tracker.getImage();

            if(rgbImage == null) continue;
            ld.setBitmap(rgbImage);
            ld.logDebug();
            ld.logTelemetry();
            telemetry.update();
            sleep(100);

            //ZZBeaconFinder.BeaconSide bs = bd.getRedPosSide();
            //ZZRobotLog.dd("SJH", "Redside = " + bs);
        }
        RobotLog.dd("SJH", "Out of LED loop - stop sensing");
        ld.stopSensing();
        RobotLog.dd("SJH", " cleanupCamera");
        //ld.
        RobotLog.dd("SJH", " frameQueue 0");
        tracker.setFrameQueueSize(0);
        RobotLog.dd("SJH", "ByeBye");
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private ImageTracker tracker;

    private int width, height;

    private static final int FRAME_SIZE = 16;

    private static final int FRAME_WIDTH = 3 * FRAME_SIZE;
    private static final int FRAME_HEIGHT = 4 * FRAME_SIZE;

    private int widthRequest = FRAME_WIDTH;
    private int heightRequest = FRAME_HEIGHT;

    private static Point2d curPos = null;
    private static double  curHdg = 0.0;

    //private ImageProcessor bd = null;
    private BeaconDetector bd;
    private Detector ld;
    private Bitmap rgbImage = null;
    private VuforiaLocalizer.CloseableFrame frame = null;
}
