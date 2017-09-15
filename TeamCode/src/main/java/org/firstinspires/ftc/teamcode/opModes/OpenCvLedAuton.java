package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.image.LedDetector;
import org.firstinspires.ftc.teamcode.image.OpenCvCameraOpMode;

@Autonomous(name="OpenCvLedAuton", group ="Test")
public class OpenCvLedAuton extends OpenCvCameraOpMode
{
    @Override
    public void runOpMode()
    {
        initOpenCv();

        imgProc = new LedDetector();
        imgProc.setTelemetry(telemetry);

        waitForStart();

        imgProc.startSensing();

        while(opModeIsActive())
        {
            if(newImage)
            {
                newImage = false;
                imgProc.logDebug();
                imgProc.logTelemetry();
            }
            telemetry.update();
        }

        imgProc.stopSensing();

        cleanupCamera();
    }
}
