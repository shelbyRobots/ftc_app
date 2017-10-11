package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
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

            boolean step_driveType = gpad1.just_pressed(ManagedGamepad.Button.A);

            lr_x = ishaper.shape(lr_x);
            fb_y = ishaper.shape(fb_y);
            turn = ishaper.shape(turn);

            if (step_driveType)
            {
                fieldAlign = !fieldAlign;
            }

            final double direction = Math.atan2(lr_x, fb_y) +
                                     (fieldAlign ? robot.getGyroFhdg() : 0.0);
            final double speed = Math.min(1.0, Math.sqrt(lr_x * lr_x + fb_y * fb_y));

            final double lf = speed * Math.sin(direction + Math.PI / 4.0) + turn;
            final double rf = speed * Math.cos(direction + Math.PI / 4.0) - turn;
            final double lr = speed * Math.cos(direction + Math.PI / 4.0) + turn;
            final double rr = speed * Math.sin(direction + Math.PI / 4.0) - turn;

            robot.lfMotor.setPower(lf);
            robot.rfMotor.setPower(rf);
            robot.lrMotor.setPower(lr);
            robot.rrMotor.setPower(rr);

        }
    }
}