package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;


@TeleOp(name = "Mecanum")
public class MecanumTeleop extends InitLinearOpMode
{
    private boolean fieldAlign = false;
    private boolean useSetVel = false;

    private TilerunnerMecanumBot robot = new TilerunnerMecanumBot();

    private static final String TAG = "SJH_MTD";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);
        Input_Shaper ishaper = new Input_Shaper();

        double dSpd = 0.0;

        robot.init(this);

        // Send telemetry message to signify robot waiting;
        dashboard.displayText(0, "Hello Driver");

        while (!isStarted())
        {
            gpad1.update();
            gpad2.update();
            gpad1.log(1);
            gpad2.log(2);
            idle();
        }

        RobotLog.dd(TAG, "Telop_Driver starting");

        while (opModeIsActive())
        {
            gpad1.update();
            gpad2.update();

            double lr_x = gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
            double fb_y = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            double turn = gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);

            boolean rgt = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
            boolean lft = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
            boolean fwd = gpad1.pressed(ManagedGamepad.Button.D_UP);
            boolean bak = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
            boolean incr = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decr = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean toggleVel = gpad1.just_pressed(ManagedGamepad.Button.Y);

            boolean step_driveType = gpad1.just_pressed(ManagedGamepad.Button.A);

            int l = 1;
            dashboard.displayPrintf(l++, "RAW LR_X %4.2f FB_Y %4.2f TRN %4.2f",
                    lr_x, fb_y, turn);

            lr_x = ishaper.shape(lr_x);
            fb_y = ishaper.shape(fb_y);
            turn = ishaper.shape(turn);

            dashboard.displayPrintf(l++, "SHP LR_X %4.2f FB_Y %4.2f TRN %4.2f",
                    lr_x, fb_y, turn);

            if (step_driveType) fieldAlign = !fieldAlign;
            if (toggleVel) useSetVel = !useSetVel;

            double speed = Math.sqrt(lr_x * lr_x + fb_y * fb_y);
            //speed = Math.min(1.0, speed);

            if (incr) dSpd += 0.1;
            else if (decr) dSpd -= 0.1;

            if (lft)
            {
                lr_x = -1.0;
                fb_y = 0.0;
                speed = dSpd;
            } else if (rgt)
            {
                lr_x = 1.0;
                fb_y = 0.0;
                speed = dSpd;
            } else if (fwd)
            {
                fb_y = 1.0;
                lr_x = 0.0;
                speed = dSpd;
            } else if (bak)
            {
                fb_y = -1.0;
                lr_x = 0.0;
                speed = dSpd;
            }

            final double direction = Math.atan2(lr_x, fb_y) +
               (fieldAlign ? Math.toRadians(Math.PI/2.0 - robot.getGyroFhdg()) : 0.0);

            dashboard.displayPrintf(l++, "DIR %4.2f FALGN %s USEVEL %s",
                    direction, fieldAlign, useSetVel);

            double lf = speed * Math.sin(direction + Math.PI / 4.0) + turn;
            double rf = speed * Math.cos(direction + Math.PI / 4.0) - turn;
            double lr = speed * Math.cos(direction + Math.PI / 4.0) + turn;
            double rr = speed * Math.sin(direction + Math.PI / 4.0) - turn;

            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lr), Math.abs(rr)));

            dashboard.displayPrintf(l++, "lf %4.2f rf %4.2f lr %4.2f rr %4.2f",
                    lf, rf, lr, rr);

            if (max > 1.0)
            {
                lf /= max;
                rf /= max;
                lr /= max;
                rr /= max;
            }

            dashboard.displayPrintf(l++, "lf %4.2f rf %4.2f lr %4.2f rr %4.2f",
                    lf, rf, lr, rr);

            dashboard.displayPrintf(l++, "LFC %d RFC %d LRC %d RRC %d",
                    robot.lfMotor.getCurrentPosition(),
                    robot.rfMotor.getCurrentPosition(),
                    robot.lrMotor.getCurrentPosition(),
                    robot.rrMotor.getCurrentPosition());

            double diam = 4.0;  //Inches
            double maxSpd = 30.0; //Inches per Second
            if (useSetVel)
            {
                double rps = maxSpd / (diam * Math.PI);
                double dps = 360.0 * rps;
                double lfSpd = lf * dps;
                double rfSpd = rf * dps;
                double lrSpd = lr * dps;
                double rrSpd = rr * dps;
                ((DcMotorEx) robot.lfMotor).setVelocity(lfSpd, AngleUnit.DEGREES);
                ((DcMotorEx) robot.rfMotor).setVelocity(rfSpd, AngleUnit.DEGREES);
                ((DcMotorEx) robot.lrMotor).setVelocity(lrSpd, AngleUnit.DEGREES);
                ((DcMotorEx) robot.rrMotor).setVelocity(rrSpd, AngleUnit.DEGREES);

                dashboard.displayPrintf(l++, "lf %4.2f rf %4.2f lr %4.2f rr %4.2f",
                        lfSpd, rfSpd, lrSpd, rrSpd);
            } else
            {
                robot.lfMotor.setPower(lf);
                robot.rfMotor.setPower(rf);
                robot.lrMotor.setPower(lr);
                robot.rrMotor.setPower(rr);
            }
        }
    }
}