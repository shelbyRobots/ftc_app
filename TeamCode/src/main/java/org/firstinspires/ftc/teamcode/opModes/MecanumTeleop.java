package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;


@TeleOp(name = "Mecanum")
public class MecanumTeleop extends InitLinearOpMode
{
    @SuppressWarnings("FieldCanBeLocal")
    private boolean fieldAlign = false;
    private boolean useSetVel = true;

    private ElapsedTime resetTimer = new ElapsedTime();

    private TilerunnerMecanumBot robot = new TilerunnerMecanumBot();

    private static final String TAG = "SJH_MTD";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);
        Input_Shaper ishaper = new Input_Shaper();

        robot.setName(pmgr.getBotName());

        double dSpd = 0.0;

        robot.init(this);

        if (robot.numLmotors  > 0 &&
                    robot.numRmotors  > 0)
        {
            RobotLog.dd(TAG, "Initialize drivetrain");
            dtrn.init(robot);

            dtrn.setRampUp(false);
            dtrn.setRampDown(false);
            robot.setDriveDir(ShelbyBot.DriveDir.INTAKE);
            RobotLog.dd(TAG, "Start Hdg %.2f", robot.getAutonEndHdg());
        }

        // Send telemetry message to signify robot waiting;
        dashboard.displayPrintf(0, "Hello Driver - I'm %s", robot.getName());

        while (!isStarted())
        {
            gpad1.update();
            gpad2.update();
            gpad1.log(1);
            gpad2.log(2);
            idle();
        }

//        boolean pActive = false;
        boolean eActive = false;


        RobotLog.dd(TAG, "Mecanum_Driver starting");

        RobotLog.dd(TAG, "Setting gripper to init pos %.2f",
                TilerunnerMecanumBot.GRIPPER_CLOSE_POS);

        robot.closeGripper();

        if(robot.elevMotor != null)
        {
            robot.elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevMotor.setPower(0.0);
            robot.elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        robot.raiseFlicker();

        while (opModeIsActive())
        {
            gpad1.update();
            gpad2.update();

            double raw_lr_x =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
            double raw_fb_y = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            double raw_turn =  gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);

            boolean rgt  = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
            boolean lft  = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
            boolean fwd  = gpad1.pressed(ManagedGamepad.Button.D_UP);
            boolean bak  = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
            boolean incr = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decr = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean toggleVel = gpad1.just_pressed(ManagedGamepad.Button.Y);

            boolean lowerElev         = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean raiseElev         = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
            boolean holdElev          = gpad2.just_pressed(ManagedGamepad.Button.D_LEFT);

            boolean gripper_open_par  = gpad2.pressed(ManagedGamepad.Button.A);
            boolean gripper_open      = gpad2.pressed(ManagedGamepad.Button.B);

            boolean toggle_jflicker   = gpad2.just_pressed(ManagedGamepad.Button.Y);

            double elev         = -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);

            //boolean step_driveType = gpad1.just_pressed(ManagedGamepad.Button.A);

            int l = 1;
            dashboard.displayPrintf(l++, "RAW LR_X %4.2f FB_Y %4.2f TRN %4.2f",
                    raw_lr_x, raw_fb_y, raw_turn);

            double lr_x = ishaper.shape(raw_lr_x, 0.1);
            double fb_y = ishaper.shape(raw_fb_y, 0.1);
            double turn = ishaper.shape(raw_turn, 0.1);
            elev = ishaper.shape(elev, 0.1);

            dashboard.displayPrintf(l++, "SHP LR_X %4.2f FB_Y %4.2f TRN %4.2f",
                    lr_x, fb_y, turn);

            //if (step_driveType) fieldAlign = !fieldAlign;
            if (toggleVel) useSetVel = !useSetVel;

            double speed = Math.sqrt(lr_x * lr_x + fb_y * fb_y);
            //speed = Math.min(1.0, speed);

            double dScl = Math.sqrt(2.0);
            if (incr) dSpd += 0.1;
            else if (decr) dSpd -= 0.1;

            if (lft)
            {
                lr_x = -1.0;
                fb_y = 0.0;
                speed = dSpd * dScl;
            } else if (rgt)
            {
                lr_x = 1.0;
                fb_y = 0.0;
                speed = dSpd * dScl;
            } else if (fwd)
            {
                fb_y = 1.0;
                lr_x = 0.0;
                speed = dSpd * dScl;
            } else if (bak)
            {
                fb_y = -1.0;
                lr_x = 0.0;
                speed = dSpd * dScl;
            }

            final double direction = Math.atan2(lr_x, fb_y) +
               (fieldAlign ? Math.toRadians(90.0 - robot.getGyroFhdg()) : 0.0);

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

            dashboard.displayPrintf(l++, "LFC %d RFC %d LRC %d RRC %d",
                    robot.lfMotor.getCurrentPosition(),
                    robot.rfMotor.getCurrentPosition(),
                    robot.lrMotor.getCurrentPosition(),
                    robot.rrMotor.getCurrentPosition());

            if (useSetVel)
            {
                double diam = 4.0;  //Inches

                double maxIPS = 30.0;
                double maxRPS = maxIPS/(diam*Math.PI);
                double maxDPS = maxRPS*360.0;

                double lfSpd = lf * maxDPS;
                double rfSpd = rf * maxDPS;
                double lrSpd = lr * maxDPS;
                double rrSpd = rr * maxDPS;
                ((DcMotorEx) robot.lfMotor).setVelocity(lfSpd, AngleUnit.DEGREES);
                ((DcMotorEx) robot.rfMotor).setVelocity(rfSpd, AngleUnit.DEGREES);
                ((DcMotorEx) robot.lrMotor).setVelocity(lrSpd, AngleUnit.DEGREES);
                ((DcMotorEx) robot.rrMotor).setVelocity(rrSpd, AngleUnit.DEGREES);

                dashboard.displayPrintf(l, "OUT: lf %4.2f rf %4.2f lr %4.2f rr %4.2f",
                        lfSpd, rfSpd, lrSpd, rrSpd);
            } else
            {
                robot.lfMotor.setPower(lf);
                robot.rfMotor.setPower(rf);
                robot.lrMotor.setPower(lr);
                robot.rrMotor.setPower(rr);
                dashboard.displayPrintf(l, "OUT: lf %4.2f rf %4.2f lr %4.2f rr %4.2f",
                        lf, rf, lr, rr);
            }

            if(Math.abs(elev) > 0.001)
            {
                eActive = false;
            }

            boolean isBottom = robot.isElevTouchPressed();

            int curElevPos = robot.elevMotor.getCurrentPosition();
            if(!eActive)
            {
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                int minElev = robot.liftPositions.get(0) + 20;
                int maxElev = robot.liftPositions.get(robot.liftPositions.size() - 1);
                if(curElevPos < minElev && elev < 0.0)
                    elev = 0.0;
//                if(curElevPos > maxElev && elev > 0.0)
//                    elev = 0.0;
                if(isBottom) elev = 0.0;
                robot.elevMotor.setPower(elev);
            }

            int nextDown = 0;
            int nextUp   = 1;
            int ethresh  = (int)(1.5 * robot.ELEV_CPI);

            for(int eIdx = 0; eIdx < robot.liftPositions.size() - 1; eIdx++)
            {
                if(raiseElev && curElevPos + ethresh >= robot.liftPositions.get(eIdx))
                {
                    nextUp = eIdx + 1;
                }
            }

            for(int eIdx = robot.liftPositions.size() - 1; eIdx > 0; eIdx--)
            {
                if(lowerElev && curElevPos - ethresh <= robot.liftPositions.get(eIdx))
                {
                    nextDown = eIdx - 1;
                }
            }

            nextDown = Math.max(0, nextDown);
            nextUp   = Math.min(robot.liftPositions.size() - 1, nextUp);

            if(lowerElev && !isBottom)
            {
                int dPos = robot.liftPositions.get(nextDown);
                RobotLog.dd(TAG, "LowerElev. curPos=%d nextDown=%d dPos=%d",
                        curElevPos, nextDown, dPos);

                robot.elevMotor.setTargetPosition(dPos);
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.35;
                robot.elevMotor.setPower(elev);
                eActive = true;
            }
            else if(raiseElev)
            {
                int uPos = robot.liftPositions.get(nextUp);
                RobotLog.dd(TAG, "RaiseElev. curPos=%d nextUp=%d uPos=%d",
                        curElevPos, nextUp, uPos);

                robot.elevMotor.setTargetPosition(uPos);
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.55;
                robot.elevMotor.setPower(elev);
                eActive = true;
            }
            else if (holdElev)
            {
                robot.elevMotor.setTargetPosition(curElevPos);
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.65;
                robot.elevMotor.setPower(elev);
                eActive = true;
            }

            if(isBottom && resetTimer.seconds() > 1.0)
            {
                robot.elevMotor.setPower(0.0);
                robot.elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.elevMotor.setTargetPosition(0);
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.25;
                robot.elevMotor.setPower(elev);
                eActive = true;
                resetTimer.reset();
            }

            // Gripper (a: Somewhat Open, b: all the way open, neither: closed)
            if (gripper_open)
            {
                robot.openGripper();
            }
            else if (gripper_open_par)
            {
                robot.partialGripper();
            }
            else
            {
                robot.closeGripper();
            }

            // Jewel Flicker (y: toggles between up and down positions)
            if (toggle_jflicker)
            {
                currentFlickerState = (currentFlickerState == FlickerState.DOWN) ?
                        FlickerState.UP : FlickerState.DOWN;

                switch (currentFlickerState)
                {
                    case DOWN:
                        //robot.jflicker.setPosition(TilerunnerMecanumBot.JFLICKER_DOWN_POS);
                        robot.deployFlicker();
                        break;
                    case UP:
                        //robot.jflicker.setPosition(TilerunnerMecanumBot.JFLICKER_UP_POS);
                        robot.raiseFlicker();
                        break;
                }
            }
        }
    }

    private Drivetrain dtrn = new Drivetrain();

//    private enum PitchState { PITCH_UP, PITCH_DOWN }
//    private PitchState currentPitchState = PitchState.PITCH_UP;

    private enum FlickerState { UP, DOWN }
    private FlickerState currentFlickerState = FlickerState.UP;
}
// b o n e s