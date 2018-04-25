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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.field.RrField;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.MajorColorDetector;
import org.firstinspires.ftc.teamcode.image.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.List;

@Autonomous(name="Concept: VuMark Id", group ="Concept")
@Disabled
public class VuMarkOpenCvTest extends InitLinearOpMode
{
    private static ImageTracker tracker;
    public static final String TAG = "SJH_VuMark";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, true, true, false, false);
        super.runOpMode();

        RobotLog.dd(TAG, "Creating new Detector");
        Detector det = new MajorColorDetector();
        RobotLog.dd(TAG, "Back from Creating new Detector");

        tracker = new ImageTracker(VuforiaInitializer.Challenge.RR);

        setupCropCorners();

        //waitForStart();

        RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;
        tracker.setActive(true);
        RobotLog.dd(TAG, "Finding VuMark first");

        while (!isStarted() && key == RelicRecoveryVuMark.UNKNOWN)
        {
            ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            Point2d sensedBotPos;
            double  sensedFldHdg = 0.0;
            int frm = 0;

            tracker.updateRobotLocationInfo();
            sensedBotPos = tracker.getSensedPosition();

            if(sensedBotPos != null)
            {
                sensedFldHdg = tracker.getSensedFldHeading();
                key = tracker.getKeyLoc();
            }
            frm++;

            if ( sensedBotPos != null )
            {
                double t = itimer.seconds();
                RobotLog.ii("SJH", "Senesed Pos: %s %5.2f %2.3f",
                        sensedBotPos, sensedFldHdg, t);
                RobotLog.ii("SJH", "IMG %s frame %d",
                        tracker.getLocString(), frm);
                dashboard.displayPrintf(1, "SLOC: %s %4.1f",
                        sensedBotPos, sensedFldHdg);
                dashboard.displayPrintf(2, "IMG", "%s  frame %d",
                        tracker.getLocString(), frm);
            }

            telemetry.update();
        }

        RobotLog.ii(TAG, "KEY : " + key);
        dashboard.displayPrintf(0, "KEY: " + key);
        RobotLog.dd(TAG, "Turn off image tracker");
        tracker.setActive(false);
        RobotLog.dd(TAG, "Set qsize to get frames");
        tracker.setFrameQueueSize(1);
        RobotLog.dd(TAG, "Start LD sensing");
        det.startSensing();
        sleep(100);

        MajorColorDetector.Color leftJewelColor = MajorColorDetector.Color.NONE;
        while(!isStarted() && leftJewelColor == MajorColorDetector.Color.NONE)
        {
            Bitmap rgbImage;
            rgbImage = tracker.getLastCroppedImage();

            if(rgbImage == null) continue;
            det.setBitmap(rgbImage);
            det.logDebug();
            det.logTelemetry();
            if(det instanceof MajorColorDetector)
                leftJewelColor = ((MajorColorDetector) det).getMajorColor();
            sleep(100);
        }
//        Bitmap fullImage = tracker.getLastImage();
//        List<Point2d> tpcs = tracker.getTrackableCornersInCamera(tracker.getRawImagePose());
//        MajorColorDetector mcd = (MajorColorDetector) det;
//        mcd.hightlightCorners(fullImage, tpcs);

        waitForStart();

        while(opModeIsActive())
        {
            //Just sit here until stopped
            sleep(100);
        }

        RobotLog.dd(TAG, "Out of detector loop - stop sensing");
        det.stopSensing();
        RobotLog.dd(TAG, " cleanupCamera");
        RobotLog.dd(TAG, " frameQueue 0");
        tracker.setFrameQueueSize(0);
        RobotLog.dd(TAG, "ByeBye");
    }

    @SuppressWarnings("ConstantConditions")
    private void setupCropCorners()
    {
        tracker.setTrackableRelativeCropCorners(RrField.getTrackableRelativeCropCorners());
    }
}
