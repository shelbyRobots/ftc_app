package org.firstinspires.ftc.teamcode.test;

import android.util.SparseArray;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

/**
 * This OpMode steps n motors speed up and down based on D-Pad user inputs.
 * This code assumes a DC motor configured with the name "testmotor#".
 * If motors are on a bot, left side motors should be odd and right side even
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 */
@Autonomous(name = "Concept: Step Servo Speed", group = "Test")
//@Disabled
public class StepServoTest extends InitLinearOpMode
{

    private static final double INCREMENT = 0.02;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 50;       // period of each cycle
    private static final double   MAX_POS =  1.0;     // Maximum servo pos
    private static final double   MIN_POS =  0.0;     // Minimum servo pos
    private double pos = 0.5;

    // Define class members
    private static final int MAX_SERVOS = 4;

    private static final String TAG = "SJH_SST";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        SparseArray<Servo> servos = new SparseArray<>(MAX_SERVOS);

        for(int m = 0; m < MAX_SERVOS; m++)
        {
            String servoName = "testservo" + m;
            Servo srv = null;
            try
            {
                srv = hardwareMap.servo.get(servoName);
            }
            catch(Exception e)
            {
                RobotLog.ee(TAG, "Problem finding servo " + servoName);
                continue;
            }

            servos.put(m, srv);
            RobotLog.dd(TAG, "Found motor " + servoName);
            if(srv != null)
            {
                srv.setPosition(pos);
            }
        }

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run servos.");
        while(!isStarted())
        {
            for(int m = 0; m < MAX_SERVOS; m++)
            {
                Servo srv = servos.get(m);
                if(srv != null)
                    dashboard.displayPrintf(m, "SRV_%d %.2f", m, srv.getPosition());
            }
            sleep(10);
        }
        waitForStart();

        while(opModeIsActive())
        {
            gpad1.update();
            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);

            if(step_up && pos < MAX_POS - INCREMENT)         pos += INCREMENT;
            else if(step_down && pos > MIN_POS + INCREMENT)  pos -= INCREMENT;
            else if(zeroize)                                 pos = 0.0;

            // Display the current value
            dashboard.displayPrintf(MAX_SERVOS, "Servo pos %4.2f", pos);
            for(int m = 0; m < MAX_SERVOS; m++)
            {
                Servo srv = servos.get(m);
                if(srv != null)
                {
                    srv.setPosition(pos);
                }
            }

            dashboard.displayText(MAX_SERVOS + 1, "Press Stop to end test.");
            dashboard.displayText(MAX_SERVOS + 2, "Incr power : Dpad up");
            dashboard.displayText(MAX_SERVOS + 3, "Decr power : Dpad down");
            dashboard.displayText(MAX_SERVOS + 4, "Zero power : Dpad right");

            sleep(CYCLE_MS);
        }

        dashboard.displayText(MAX_SERVOS + 1, "Done." );
    }
}
