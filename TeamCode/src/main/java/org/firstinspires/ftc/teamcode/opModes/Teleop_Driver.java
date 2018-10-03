package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.RrField;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerGtoBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

@SuppressWarnings("unused")
@TeleOp(name="TeleopDriver", group="Tele")
//@Disabled
public class Teleop_Driver extends InitLinearOpMode
{
    private void initPreStart()
    {
        robot.setName(pmgr.getBotName());

        /* Initialize the hardware variables. */
        RobotLog.dd(TAG, "Initialize robot");
        //robot.init(this);
        robot.init(this, false);

        if (robot.numLmotors  > 0 &&
            robot.numRmotors  > 0)
        {
            RobotLog.dd(TAG, "Initialize drivetrain");
            dtrn.init(robot);

            dtrn.setRampUp(false);
            dtrn.setRampDown(false);
            robot.setDriveDir(ShelbyBot.DriveDir.INTAKE);
            RobotLog.dd(TAG, "Start Aend fHdg %.2f", robot.getAutonEndHdg());
            //RobotLog.dd(TAG, "Start Hdg %.2f", robot.get);
            RobotLog.dd(TAG, "Start Pos %s", robot.getAutonEndPos().toString());
            RobotLog.dd(TAG, "Start mode to %s", robot.leftMotor.getMode());

            dtrn.setCurrPt(robot.getAutonEndPos());

            lex = (DcMotorEx)(robot.leftMotors.get(0));
            rex = (DcMotorEx)(robot.rightMotors.get(0));
        }
    }

    private void initPostStart()
    {
        robot.closeGripper();
        robot.stowHolder();

        if(robot.relClamp != null)
        {
            RobotLog.dd(TAG, "Setting RelClamp to 1.0 at start");
            robot.relClamp.setPosition(1.0);
        }

        if(robot.gpitch != null)
        {
            //robot.gpitch.setPosition(robot.GPITCH_UP_POS);
            robot.retractGpitch();
            pActive = true;
            currentPitchState = PitchState.PITCH_UP;
        }

//        if(robot.elevMotor != null)
//        {
//            robot.initElevZero();
//        }

        robot.setElevZero();
    }

    private void controlGrippers()
    {
        boolean gripper_open_par  = gpad2.pressed(ManagedGamepad.Button.A);
        boolean gripper_open      = gpad2.pressed(ManagedGamepad.Button.B);

        // Gripper (a: Somewhat Open, b: all the way open, neither: closed)
        if (gripper_open)
        {
            //robot.gripper.setPosition(robot.GRIPPER_OPEN_POS);
            robot.openGripper();
        }
        else if (gripper_open_par)
        {
            //robot.gripper.setPosition(robot.GRIPPER_PARTIAL_POS);
            robot.partialGripper();
        }
        else
        {
            //robot.gripper.setPosition(robot.GRIPPER_CLOSE_POS);
            robot.closeGripper();
        }
    }

    private void controlElevator()
    {
        double elev               = -gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);

        boolean lowerElev         = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
        boolean raiseElev         = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        boolean holdElev          = gpad2.just_pressed(ManagedGamepad.Button.L_BUMP);
        boolean decrElev          = gpad2.just_pressed(ManagedGamepad.Button.D_LEFT);
        boolean incrElev          = gpad2.just_pressed(ManagedGamepad.Button.D_RIGHT);
        boolean overrideElev      = gpad2.pressed(ManagedGamepad.Button.R_BUMP);

        int curElevPos = 0;

        elev  = ishaper.shape(elev);

        if(robot.elevMotor != null)
        {
            curElevPos = robot.elevMotor.getCurrentPosition();
        }
        else if(robot.elevServo != null)
        {
            double sPos = robot.elevServo.getPosition();
            curElevPos = (int)(sPos * robot.MICRO_RNG + robot.MICRO_MIN);
        }

        if(Math.abs(elev) > 0.001)
        {
            eActive = false;
        }

        if(!eActive && robot.elevMotor != null)
        {
            robot.elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int minElev = robot.getMinElev();
            int maxElev = robot.getMaxElev();
            if(!overrideElev)
            {
                if (curElevPos < minElev && elev < 0.0)
                    elev = 0.0;
                if (curElevPos > maxElev && elev > 0.0)
                    elev = 0.0;
            }
            robot.elevMotor.setPower(elev);
        }

        int nextDown = 0;
        int nextUp   = 1;
        int ethresh  = (int)(1.5 * robot.ELEV_CPI);

        for(int eIdx = 0; eIdx < robot.liftPositions.size() - 1; eIdx++)
        {
            if(raiseElev && curElevPos + ethresh >=
                                    robot.liftPositions.get(eIdx) + robot.MICRO_MIN)
            {
                nextUp = eIdx + 1;
            }
        }

        for(int eIdx = robot.liftPositions.size() - 1; eIdx > 0; eIdx--)
        {
            if(lowerElev && curElevPos - ethresh <=
                                    robot.liftPositions.get(eIdx) + robot.MICRO_MIN)
            {
                nextDown = eIdx - 1;
            }
        }

//            RobotLog.dd(TAG,"UP_DOWN cur %d dwn %d %d up %d %d",
//                    curElevPos,
//                    nextDown, robot.liftPositions.get(nextDown),
//                    nextUp, robot.liftPositions.get(nextUp));

        nextDown = Math.max(0, nextDown);
        nextUp   = Math.min(robot.liftPositions.size() - 1, nextUp);

        if(lowerElev)
        {
            int dPos = robot.liftPositions.get(nextDown);
            RobotLog.dd(TAG, "LowerElev. curPos=%d nextDown=%d dPos=%d",
                    curElevPos, nextDown, dPos);
            if(robot.elevMotor != null)
            {
                robot.elevMotor.setTargetPosition(dPos);
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.75;
                robot.elevMotor.setPower(elev);
                eActive = true;
            }
            else if(robot.elevServo != null)
            {
                double nrmPos = (dPos * 1.0)/robot.MICRO_RNG;
                RobotLog.dd(TAG, "LowerElev. dPos=%d nrmPos=%.2f", dPos, nrmPos);
                robot.elevServo.setPosition(nrmPos);
            }
        }
        else if(raiseElev)
        {
            int uPos = robot.liftPositions.get(nextUp);
            RobotLog.dd(TAG, "RaiseElev. curPos=%d nextUp=%d uPos=%d",
                    curElevPos, nextUp, uPos);
            if(robot.elevMotor != null)
            {
                robot.elevMotor.setTargetPosition(uPos);
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.75;
                robot.elevMotor.setPower(elev);
                eActive = true;
            }
            else if(robot.elevServo != null)
            {
                double nrmPos = (uPos * 1.0)/robot.MICRO_RNG;
                RobotLog.dd(TAG, "RaiseElev. uPos=%d nrmPos=%.2f", uPos, nrmPos);
                robot.elevServo.setPosition(nrmPos);
            }
        }
        else if (holdElev && robot.elevMotor != null)
        {
            int curPos = robot.elevMotor.getCurrentPosition();
            robot.elevMotor.setTargetPosition(curPos);
            robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elev = 0.65;
            robot.elevMotor.setPower(elev);
            eActive = true;
        }
        else if (decrElev && robot.elevServo != null)
        {
            int dPos = curElevPos - (int)(1.0 * robot.ELEV_CPI) - robot.MICRO_MIN;
            int limit = robot.liftPositions.get(0);
            RobotLog.dd(TAG, "DecrElev. dPos=%d limit=%d", dPos, limit);
            dPos = Math.max(limit, dPos);
            double nrmPos = dPos * 1.0/robot.MICRO_RNG;
            RobotLog.dd(TAG, "DecrElev. dPos=%d nrmPos=%.2f", dPos, nrmPos);
            robot.elevServo.setPosition(nrmPos);
        }
        else if (incrElev && robot.elevServo != null)
        {
            int uPos = curElevPos + (int)(1.0 * robot.ELEV_CPI)  - robot.MICRO_MIN;
            int limit = robot.liftPositions.get(robot.liftPositions.size() - 1);
            RobotLog.dd(TAG, "IncrElev. uPos=%d limit=%d", uPos, limit);
            uPos = Math.min(limit, uPos);
            double nrmPos = uPos * 1.0/robot.MICRO_RNG;
            RobotLog.dd(TAG, "IncrElev. uPos=%d nrmPos=%.2f", uPos, nrmPos);
            robot.elevServo.setPosition(nrmPos);
        }
    }

    private void controlPitch()
    {
        if(robot.gpitch == null)
        {
            return;
        }
        boolean toggle_gpitch = gpad2.just_pressed(ManagedGamepad.Button.X);
        double pitch          = -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);

        // Pitch (Gripper angle servo) (x: toggles between closed and open position)

        double locMin = robot.getGpitchMin();
        double locMax = robot.getGpitchMax();

        double dp = robot.getGpitchDownPos();
        double up = robot.getGpitchUpPos();

        if(dp > up)
        {
            locMin = robot.getGpitchMax();
            locMax = robot.getGpitchMin();
        }

        double outPitch = dp;

        double thresh = 0.001;

        if(pitch > thresh)  outPitch = locMax;
        else if(pitch < -thresh) outPitch = locMin;

        if (toggle_gpitch)
        {
            if (currentPitchState == PitchState.PITCH_UP)
            {
                currentPitchState = PitchState.PITCH_DOWN;
                robot.retractGpitch();
            }
            else if (currentPitchState == PitchState.PITCH_DOWN)
            {
                currentPitchState = PitchState.PITCH_UP;
                robot.deployGpitch();
            }
            pActive = true;
        }

        if (Math.abs(pitch) > 0.001)
        {
            pActive = false;
        }

        if(!pActive && robot.gpitch != null)
        {
            robot.gpitch.setPosition(outPitch);
        }
    }

    private void controlPusher()
    {
        boolean toggle_jflicker   = gpad2.just_pressed(ManagedGamepad.Button.Y);
        // Jewel Flicker (y: toggles between up and down positions)
        if (toggle_jflicker)
        {
            if (currentFlickerState == FlickerState.DOWN)
            {
                currentFlickerState = FlickerState.UP;
                robot.deployFlicker();
            }
            else if (currentFlickerState == FlickerState.UP)
            {
                currentFlickerState = FlickerState.DOWN;
                robot.stowFlicker();
            }
        }
    }

    private void controlArm()
    {
        if(!robot.getCapability("arm")) return;

        double  rslide      = -gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);
        boolean pitchUp     =  gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        boolean pitchDown   =  gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
        boolean openRgrip   =  gpad2.pressed(ManagedGamepad.Button.L_BUMP);

        rslide = ishaper.shape(rslide);

        int pitchDir = 1;
        if(pitchDown) pitchDir = -1;

        int CNT_PER_MOTOR_REV = 28;
        int GEAR_1 = 40;
        double GEAR_2 = 40.0/120.0;

        if((pitchDown || pitchUp) && robot.relPitch != null)
        {
            int curPos = robot.relPitch.getCurrentPosition();
            double CNT_PER_PITCH_REV =
                    CNT_PER_MOTOR_REV * GEAR_1 * GEAR_2;

            double CNT_PER_PITCH_DEG = CNT_PER_PITCH_REV / 360.0;

            int DEG_PER_STEP = 20;

            int pitchInc = (int)(DEG_PER_STEP * CNT_PER_PITCH_DEG);
            int newPos = prevRpitch + pitchDir * pitchInc;
            RobotLog.dd(TAG, "Moving Relic Pitch from %d to %d. CPD=%.2f",
                    curPos, newPos, CNT_PER_PITCH_DEG);
            robot.relPitch.setTargetPosition(newPos);
            robot.relPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.relPitch.setPower(0.7);
            prevRpitch += pitchDir * pitchInc;
        }

        if(robot.relClamp != null)
        {
            double clampPos;
            if(openRgrip)
            {
                clampPos = relClampOpenPos;
            }
            else
            {
                clampPos = relClampClosedPos;
            }

            RobotLog.dd(TAG, "Setting RelClamp to %.2f", clampPos);
            robot.relClamp.setPosition(clampPos);
        }


        if(robot.relExtend != null)
        {
            double rslideSpd = Math.abs(rslide);

            if(rslide >= 0.0)
            {
                robot.relExtend.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else
            {
                robot.relExtend.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            robot.relExtend.setPower(rslideSpd);
        }
    }

    private void controlDrive()
    {
        // Run wheels in tank mode
        // (note: The joystick goes negative when pushed forwards, so negate it)
        boolean toggle_run_mode   = gpad1.just_pressed(ManagedGamepad.Button.X);
        boolean invert_drive_dir  = gpad1.just_pressed(ManagedGamepad.Button.Y);

        boolean step_driveType    = gpad1.just_pressed(ManagedGamepad.Button.A);
        boolean toggle_float      = gpad1.just_pressed(ManagedGamepad.Button.B);
        boolean toggle_vel        = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);

        boolean left_step         = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
        boolean right_step        = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
        boolean fwd_step          = gpad1.pressed(ManagedGamepad.Button.D_UP);
        boolean back_step         = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
        boolean fst_dtl           = gpad1.pressed(ManagedGamepad.Button.L_BUMP);

        double raw_left     = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
        double raw_right    = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        double raw_turn     =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);

        boolean goBox       =  gpad1.just_pressed(ManagedGamepad.Button.L_TRIGGER);
        boolean goPit       =  gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);

        if(goBox)
        {
            goToBox();
            return;
        }

        if(goPit)
        {
            goToPit();
            return;
        }

        double shp_left  = ishaper.shape(raw_left,  0.1);
        double shp_right = ishaper.shape(raw_right, 0.1);
        double shp_turn  = ishaper.shape(raw_turn, 0.1);

        dashboard.displayPrintf(0, "TMODE " + driveType);
        dashboard.displayPrintf(1, "L_IN %4.2f", raw_left);
        dashboard.displayPrintf(2, "R_IN %4.2f", raw_right);

        double arcadeTurnScale = 0.5;

        if(toggle_vel) useSetVel = !useSetVel;

        double step_dist = 2.0;
        double step_spd  = 0.4;
        double step_ang  = 5.0;

        double fHdg = robot.getGyroFhdg();
        double detail_speed = 0.18;
        if(fst_dtl) detail_speed = 0.3;

        double maxIPS = 45.0;
        double maxRPS = maxIPS/(4.0*Math.PI);
        double maxDPS = maxRPS*360.0;

        double left  = 0.0;
        double right = 0.0;
        double turn  = 0.0;

        if(fwd_step)
        {
            RobotLog.dd(TAG, "In fwd_step");
            left  = detail_speed;
            right = detail_speed;
            //dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.FORWARD);
            RobotLog.dd(TAG, "Done fwd_step");
        }
        else if(back_step)
        {
            RobotLog.dd(TAG, "In back_step");
            left  = -detail_speed;
            right = -detail_speed;
            //dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.REVERSE);
            RobotLog.dd(TAG, "Done back_step");
        }
        else if(left_step)
        {
            RobotLog.dd(TAG, "In left_step");
            left  = -detail_speed;
            right =  detail_speed;
            //dtrn.setInitValues();
            //dtrn.ctrTurnToHeading(fHdg + step_ang, step_spd);
            RobotLog.dd(TAG, "Done left_step");
        }
        else if(right_step)
        {
            RobotLog.dd(TAG, "In right_step");
            left  =  detail_speed;
            right = -detail_speed;
            //dtrn.setInitValues();
            //dtrn.ctrTurnToHeading(fHdg - step_ang, step_spd);
            RobotLog.dd(TAG, "Done right_step");
        }
        else
        {
            if(useSetVel)
            {
                switch (driveType)
                {
                    case TANK_DRIVE:
                        left  = raw_left;
                        right = raw_right;
                        break;

                    case ARCADE_DRIVE:
                        left  = raw_right;
                        right = left;
                        left  += raw_turn*arcadeTurnScale;
                        right -= raw_turn*arcadeTurnScale;
                }
            }
            else
            {
                switch (driveType)
                {
                    case TANK_DRIVE:
                        left  = shp_left;
                        right = shp_right;
                        break;

                    case ARCADE_DRIVE:
                        left = shp_right;
                        right = left;
                        turn = (1 - Math.abs(left)) * turn;
                        if(turn < 0.95) turn *= arcadeTurnScale;
                        left  += turn;
                        right -= turn;
                        break;
                }
            }

            double vmax = Math.max(Math.abs(left), Math.abs(right));
            if(vmax > 1.0)
            {
                left  /= vmax;
                right /= vmax;
            }
        }

        @SuppressWarnings("UnusedAssignment")
        double out_left = left;
        @SuppressWarnings("UnusedAssignment")
        double out_right = right;

        if(useSetVel)
        {
            out_left  = maxDPS*left;
            out_right = maxDPS*right;
            lex.setVelocity(out_left,  AngleUnit.DEGREES);
            rex.setVelocity(out_right, AngleUnit.DEGREES);
        }
        else
        {
            double governor = 0.8;
            out_left  = left * governor;
            out_right = right * governor;
            robot.leftMotors.get(0).setPower(out_left);
            robot.rightMotors.get(0).setPower(out_right);
        }

        dashboard.displayPrintf(4, "TURN  %4.2f", turn);
        dashboard.displayPrintf(5, "L_OUT %4.2f", left);
        dashboard.displayPrintf(6, "R_OUT %4.2f", right);
        dashboard.displayPrintf(7, "L_CNT %d", robot.leftMotor.getCurrentPosition());
        dashboard.displayPrintf(8, "R_CNT %d", robot.rightMotor.getCurrentPosition());
        dashboard.displayPrintf(9, "DTYPE " + driveType);
        dashboard.displayPrintf(10,"D_DIR " + robot.getDriveDir());
        dashboard.displayPrintf(11,"Z_PWR " + zeroPwr);
        dashboard.displayPrintf(12,"RMODE " + robot.leftMotor.getMode());
        dashboard.displayPrintf(13,"L_DIR " + robot.leftMotor.getDirection());
        dashboard.displayPrintf(14,"R_DIR " + robot.rightMotor.getDirection());
        dashboard.displayPrintf(15,"SVEL " + useSetVel);

        if(toggle_float)
        {
            if(zeroPwr == DcMotor.ZeroPowerBehavior.BRAKE)
                zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;
            else
                zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
            if(robot.leftMotor  != null) robot.leftMotor.setZeroPowerBehavior(zeroPwr);
            if(robot.rightMotor != null) robot.rightMotor.setZeroPowerBehavior(zeroPwr);
        }

        if(invert_drive_dir)
        {
            robot.invertDriveDir();
        }

        if(toggle_run_mode && robot.leftMotor != null && robot.rightMotor != null)
        {
            DcMotor.RunMode currMode = robot.leftMotor.getMode();
            if(currMode == DcMotor.RunMode.RUN_USING_ENCODER)
            {
                dtrn.setMode(robot.leftMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                dtrn.setMode(robot.rightMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else
            {
                dtrn.setMode(robot.leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
                dtrn.setMode(robot.rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
            }
            RobotLog.dd(TAG, "Change mode to %s", robot.leftMotor.getMode());
        }

        if(step_driveType && robot.leftMotor != null && robot.rightMotor != null)
        {
            switch(driveType)
            {
                case TANK_DRIVE:
                    driveType = TELEOP_DRIVE_TYPE.ARCADE_DRIVE;
                    break;
                case ARCADE_DRIVE:
                    driveType = TELEOP_DRIVE_TYPE.TANK_DRIVE;
                    break;
            }
        }
    }

    private void processControllerInputs()
    {
        boolean shiftControls = gpad2.pressed(ManagedGamepad.Button.R_TRIGGER);

        if(!shiftControls)
        {
            controlElevator();
            controlPitch();
            controlGrippers();
            controlPusher();
        }
        else
        {
            controlArm();
        }
    }

    private void processDriverInputs()
    {
        controlDrive();
    }

    private void goToBox()
    {
        Point2d curPt = dtrn.getEstPos();
        Field.Alliance alliance = robot.getAlliance();
        double box_y = RrField.getTTy();
        if (alliance == Field.Alliance.BLUE) box_y = -box_y;
        Point2d boxPt = new Point2d("BXPT", RrField.getTTx(), box_y);
        Segment seg = new Segment("toBox", curPt, boxPt);
        seg.setSpeed(0.6);
        RobotLog.dd(TAG, "goToBox seg=%s", seg.toString());
        doEncoderTurn(seg.getFieldHeading(), "box");
        doGyroTurn(seg.getFieldHeading(), "box");
        doMove(seg);
    }

    private void goToPit()
    {
        Point2d curPt = dtrn.getEstPos();
        Field.Alliance alliance = robot.getAlliance();
        double ppy = RrField.getPPy1();
        if (alliance == Field.Alliance.BLUE) ppy = -ppy;
        Point2d pitPt = new Point2d("PTPT", RrField.getPPx(), ppy);
        Segment seg = new Segment("toPit", curPt, pitPt);
        seg.setSpeed(0.6);
        RobotLog.dd(TAG, "goToHome seg=%s", seg.toString());
        doEncoderTurn(seg.getFieldHeading(), "pit");
        doGyroTurn(seg.getFieldHeading(), "pit");
        doMove(seg);
    }

    private void doMove(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested()) return;

        dtrn.setInitValues();
        dtrn.logData(true, seg.getName() + " move");
        dtrn.setDrvTuner(seg.getDrvTuner());

        dtrn.setBusyAnd(true);
        String  snm = seg.getName();
        Point2d spt = seg.getStrtPt();
        Point2d ept = seg.getTgtPt();
        double  fhd = seg.getFieldHeading();
        ShelbyBot.DriveDir dir = seg.getDir();
        double speed = seg.getSpeed();
        double fudge = seg.getDrvTuner();
        Segment.TargetType ttype = seg.getTgtType();

        RobotLog.ii(TAG, "Drive %s %s %s %6.2f %3.2f %s tune: %4.2f %s",
                snm, spt, ept, fhd, speed, dir, fudge, ttype);

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;

        timer.reset();

        double targetHdg = seg.getFieldHeading();
        dtrn.driveToPointLinear(ept, speed, ddir, targetHdg);

        dtrn.setCurrPt(ept);

        RobotLog.ii(TAG, "Completed move %s. Time: %6.3f HDG: %6.3f",
                seg.getName(), timer.time(), robot.getGyroFhdg());
    }

    private void doEncoderTurn(double fHdg, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        dtrn.setBusyAnd(true);
        dtrn.setInitValues();
        dtrn.logData(true, prefix);
        double cHdg = dtrn.curHdg;
        double angle = fHdg - cHdg;
        RobotLog.ii(TAG, "doEncoderTurn CHDG %6.3f THDG %6.3f", cHdg, fHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 4.0) return;

        RobotLog.ii(TAG, "Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        dtrn.ctrTurnLinear(angle, 0.6, Drivetrain.TURN_BUSYTHRESH);
        cHdg = robot.getGyroFhdg();
        RobotLog.ii(TAG, "Completed turn %5.2f. Time: %6.3f CHDG: %6.3f",
                angle, timer.time(), cHdg);
    }

    private void doGyroTurn(double fHdg, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;

        dtrn.setInitValues();
        dtrn.logData(true, prefix);
        double cHdg = dtrn.curHdg;

        RobotLog.ii(TAG, "doGyroTurn CHDG %4.2f THDG %4.2f", cHdg, fHdg);

        if(Math.abs(fHdg-cHdg) < 1.0)
            return;

        timer.reset();
        dtrn.ctrTurnToHeading(fHdg, 0.4);

        cHdg = dtrn.curHdg;
        RobotLog.ii(TAG, "Completed turnGyro %4.2f. Time: %6.3f CHDG: %4.2f",
                fHdg, timer.time(), cHdg);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        initPreStart();

        // Send telemetry message to signify robot waiting;
        dashboard.displayPrintf(0, "Hello Driver - I am %s", robot.getName());

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted())
        {
            gpad1.update();
            gpad2.update();
//            gpad1.log(1);
//            gpad2.log(2);

            if(robot.relPitch != null)
            {
                dashboard.displayPrintf(5, "Rpitch cnt %d",
                        robot.relPitch.getCurrentPosition());
            }
            idle();
        }

        RobotLog.dd(TAG, "Telop_Driver starting");

        initPostStart();

        // run until the end of the match (driver presses STOP)
        ElapsedTime prntTimer = new ElapsedTime();
        prntTimer.reset();
        ElapsedTime teleTimer = new ElapsedTime();
        teleTimer.reset();

        while (opModeIsActive())
        {
            if(estimatePos)
            {
                dtrn.setCurValues();
                dtrn.logData();
            }

            if(prntTimer.seconds() > 1.0)
            {
                prntTimer.reset();
                RobotLog.dd(TAG, "EstPos %s %4.3f Teletime=%4.3f",
                        dtrn.getEstPos(), robot.getGyroFhdg(),
                        teleTimer.seconds());
            }

            gpad1.update();
            gpad2.update();

            processDriverInputs();
            processControllerInputs();

            // Pause for metronome tick.
            robot.waitForTick(10);
        }
    }

    private enum TELEOP_DRIVE_TYPE
    {
        TANK_DRIVE,
        ARCADE_DRIVE
    }

    private TELEOP_DRIVE_TYPE driveType = TELEOP_DRIVE_TYPE.ARCADE_DRIVE;

    public final double minPwr = 0.10;
    public final double minTrn = 0.10;

    private DcMotorEx lex = null;
    private DcMotorEx rex = null;

    private TilerunnerGtoBot robot = new TilerunnerGtoBot();
    private Drivetrain dtrn = new Drivetrain();
    private Input_Shaper ishaper = new Input_Shaper();

    private DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
    private boolean useSetVel = true;

    private enum PitchState {PITCH_UP, PITCH_DOWN }
    private PitchState currentPitchState = PitchState.PITCH_UP;

    private enum FlickerState { UP, DOWN }
    private FlickerState currentFlickerState = FlickerState.UP;

    @SuppressWarnings("FieldCanBeLocal")
    private final double relClampOpenPos = 1.0;
    @SuppressWarnings("FieldCanBeLocal")
    private final double relClampClosedPos = 0.0;

    private boolean pActive = false;
    private boolean eActive = false;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean estimatePos = true;

    private int prevRpitch = 0;

    private ElapsedTime timer = new ElapsedTime();

    private static final String TAG = "SJH_TD";
}

