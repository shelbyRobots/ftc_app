package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;


@TeleOp(name = "Mecanum")
public class MecanumTeleop extends InitLinearOpMode
{
    private boolean fieldAlign = false;

    private TilerunnerMecanumBot robot = new TilerunnerMecanumBot();

    private static final String TAG = "SJH_MTD";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);
        Input_Shaper ishaper = new Input_Shaper();

        robot.init(this);

        // Send telemetry message to signify robot waiting;
        dashboard.displayText(0, "Hello Driver");

        while(!isStarted())
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

            double lr_x  = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
            double fb_y  =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            double turn  =  gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);

            boolean rgt = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
            boolean lft = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
            boolean fwd = gpad1.pressed(ManagedGamepad.Button.D_UP);
            boolean bak = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
            boolean incr = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decr = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean zero = gpad1.just_pressed(ManagedGamepad.Button.A);
            double dSpd = 0.0;

            if(zero)      dSpd  = 0.0;
            if(incr)      dSpd += 0.1;
            else if(decr) dSpd -= 0.1;

            boolean step_driveType = gpad1.just_pressed(ManagedGamepad.Button.A);

            int l = 1;
            dashboard.displayPrintf(l++, "RAW LR_X %4.2f FB_Y %4.2f TRN %4.2f",
                    lr_x, fb_y, turn);

            lr_x = ishaper.shape(lr_x);
            fb_y = ishaper.shape(fb_y);
            turn = ishaper.shape(turn);

            dashboard.displayPrintf(l++, "SHP LR_X %4.2f FB_Y %4.2f TRN %4.2f",
                    lr_x, fb_y, turn);

            if (step_driveType)
            {
                fieldAlign = !fieldAlign;
            }

            double speed = Math.min(1.0, Math.sqrt(lr_x * lr_x + fb_y * fb_y));

            if(lft)      {lr_x = -1.0; speed = dSpd;}
            else if(rgt) {lr_x =  1.0; speed = dSpd;}
            else if(fwd) {fb_y =  1.0; speed = dSpd;}
            else if(bak) {fb_y = -1.0; speed = dSpd;}

            final double direction = Math.atan2(lr_x, fb_y) +
                                     (fieldAlign ? robot.getGyroFhdg() : 0.0);

            dashboard.displayPrintf(l++, "DIR %4.2f FALGN %s", direction, fieldAlign);

            double lf = speed * Math.sin(direction + Math.PI / 4.0) + turn;
            double rf = speed * Math.cos(direction + Math.PI / 4.0) - turn;
            double lr = speed * Math.cos(direction + Math.PI / 4.0) + turn;
            double rr = speed * Math.sin(direction + Math.PI / 4.0) - turn;

            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                                  Math.max(Math.abs(lr), Math.abs(rr)));

            dashboard.displayPrintf(l++, "lf %4.2f %rf %4.2f lr %4.2f rr %4.2f",
                    lf, rf, lr, rr);

            if(max > 1.0)
            {
                lf /= max;
                rf /= max;
                lr /= max;
                rr /= max;
            }

            dashboard.displayPrintf(l++, "lf %4.2f %rf %4.2f lr %4.2f rr %4.2f",
                    lf, rf, lr, rr);

            dashboard.displayPrintf(l++, "LFC %d RFC %d LRC %d RRC %d",
                    robot.lfMotor.getCurrentPosition(),
                    robot.rfMotor.getCurrentPosition(),
                    robot.lrMotor.getCurrentPosition(),
                    robot.rrMotor.getCurrentPosition());

            robot.lfMotor.setPower(lf);
            robot.rfMotor.setPower(rf);
            robot.lrMotor.setPower(lr);
            robot.rrMotor.setPower(rr);
        }
    }
}