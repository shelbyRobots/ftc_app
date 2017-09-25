package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.LedDetector;

@Autonomous(name="OpenCvLedAuton", group ="Test")
public class OpenCvLedAuton extends InitLinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, true, false, true);
        super.runOpMode();
        Detector imgProc = new LedDetector(true, true);
        imgProc.setTelemetry(telemetry);

        waitForStart();

        imgProc.startSensing();

        while(opModeIsActive())
        {
            if(imgProc.isNewImageReady())
            {
                imgProc.logDebug();
                imgProc.logTelemetry();
            }
            telemetry.update();
        }

        imgProc.stopSensing();
        imgProc.cleanupCamera();
    }
}
