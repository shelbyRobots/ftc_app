package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.LedDetector;

@Autonomous(name="OpenCvLedAuton", group ="Test")
public class OpenCvLedAuton extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        Detector imgProc = new LedDetector(hardwareMap, true, true);
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
