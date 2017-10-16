package org.firstinspires.ftc.teamcode.test;

import android.util.SparseArray;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

/**
 * This OpMode steps a single motor speed up and down based on D-Pad user inputs.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "testmotor#".
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Concept: Step Motor Speed", group = "Test")
//@Disabled
public class StepMotorTest extends InitLinearOpMode
{

    private static final double INCREMENT = 0.05;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 50;       // period of each cycle
    private static final double   MAX_FWD =  1.0;     // Maximum FWD power applied to motor
    private static final double   MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    private double power = 0;

    private static DcMotor.Direction LEFT_DIR  = DcMotor.Direction.FORWARD;

    private static final int MAX_MOTORS = 4;

    private static final String TAG = "SJH_RMT";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        SparseArray<DcMotor> motors = new SparseArray<>(MAX_MOTORS);

        for(int m = 0; m < MAX_MOTORS; m++)
        {
            String motorName = "testmotor" + m;
            DcMotor mot = null;
            try
            {
                mot = hardwareMap.dcMotor.get(motorName);
            }
            catch(Exception e)
            {
                RobotLog.ee(TAG, "Problem finding motor " + motorName);
                continue;
            }

            motors.put(m, mot);
            RobotLog.dd(TAG, "Found motor " + motorName);
            if(mot != null) mot.setDirection(LEFT_DIR);
        }

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run Motors.");
        while(!isStarted())
        {
            for(int m = 0; m < MAX_MOTORS; m++)
            {
                DcMotor mot = motors.get(m);
                if(mot != null)
                    dashboard.displayPrintf(m, "CNT_%d %d", m, mot.getCurrentPosition());
            }
            sleep(10);
        }
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive())
        {
            gpad1.update();
            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);

            if(step_up && power < MAX_FWD)         power += INCREMENT;
            else if(step_down && power > MAX_REV)  power -= INCREMENT;
            else if(zeroize)                       power = 0.0;

            // Display the current value
            dashboard.displayPrintf(MAX_MOTORS, "Motor Power %4.2f", power);
            for(int m = 0; m < MAX_MOTORS; m++)
            {
                DcMotor mot = motors.get(m);
                if(mot != null)
                {
                    dashboard.displayPrintf(m, "CNT_%d %d", m, mot.getCurrentPosition());
                    mot.setPower(power);
                }
            }

            dashboard.displayText(MAX_MOTORS + 1, "Press Stop to end test." );

            sleep(CYCLE_MS);
        }

        for(int m = 0; m < MAX_MOTORS; m++)
        {
            DcMotor mot = motors.get(m);
            if(mot != null)
            {
                mot.setPower(0.0);
            }
        }

        dashboard.displayText(MAX_MOTORS + 1, "Done." );
    }
}
