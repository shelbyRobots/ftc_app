package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.RoRuBot;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
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
        prevOpModeType = RoRuBot.curOpModeType;
        RoRuBot.curOpModeType = ShelbyBot.OpModeType.TELE;

        /* Initialize the hardware variables. */
        RobotLog.dd(TAG, "Initialize robot");
        //robot.init(this);
        robot.init(this, false);

        if (robot.numLmotors  > 0 &&
            robot.numRmotors  > 0)
        {
            RobotLog.dd(TAG, "Initialize drivetrain");
            robot.setDriveDir(ShelbyBot.DriveDir.INTAKE);
            dtrn.init(robot);

            dtrn.setRampUp(false);
            dtrn.setRampDown(false);

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
//        robot.zeroArmPitch();
//        robot.zeroArmExtend();
//        robot.armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //if(prevOpModeType == ShelbyBot.OpModeType.AUTO) robot.threadputHolderAtPrelatch();
        //robot.stowMarker();
        robot.stowParker();
    }

    private boolean useCnts = false;

    private int counts = 0;
    private int oldCounts = 0;

    private void controlArm()
    {
        if(!robot.getCapability("arm")) return;

        if(robot.armPitch == null) return;
        if(robot.armExtend == null) return;

        if(robot.isElevTouchPressed() && !lastArmTouchPressed)
        {
            robot.zeroArmPitch();
        }

        lastArmTouchPressed = robot.isElevTouchPressed();

        int stowCounts  = 0;
        int dropCounts  = -2400; //-(int)(10 * robot.ARM_CPD);
        int hoverCounts = -4500; //-(int)(20 * robot.ARM_CPD);
        int grabCounts  = -4500; //-(int)(30 * robot.ARM_CPD);
        int maxCounts   = -8000;

        double stowAngle = robot.ARM_ZERO_ANGLE;

        double  aslide      = -gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);
        double  apitch      = -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);

        boolean dStow       =  gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
        boolean dDrop       =  gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        boolean dHover      =  gpad2.just_pressed(ManagedGamepad.Button.D_LEFT);
        boolean dGrab       =  gpad2.just_pressed(ManagedGamepad.Button.D_RIGHT);
        boolean changeMode  =  gpad2.just_pressed(ManagedGamepad.Button.A);
        boolean intakeIn    =  gpad2.pressed(ManagedGamepad.Button.L_BUMP);
        boolean intakeOut   =  gpad2.pressed(ManagedGamepad.Button.L_TRIGGER);
        boolean override    =  gpad2.pressed(ManagedGamepad.Button.R_BUMP);
//        aslide = ishaper.shape(aslide);
//        apitch = ishaper.shape(apitch);
        double THE_ANSWER_TO_LIFE_THE_UNIVERSE_AND_EVERYTHING = 0.42;
        double MAX_APITCH_SPD = THE_ANSWER_TO_LIFE_THE_UNIVERSE_AND_EVERYTHING;
        //apitch = Math.min(apitch,  MAX_APITCH_SPD);
        //apitch = Math.max(apitch, -MAX_APITCH_SPD);
        apitch *= MAX_APITCH_SPD;

        aslide *= -1;

        if(changeMode)
        {
            useCnts = !useCnts;
        }

        int pitchDir = 1;

        if(intakeIn)       robot.intakeIn();
        else if(intakeOut) robot.intakeOut();
        else               robot.intakeStop();

        int curArmCounts = robot.armPitch.getCurrentPosition();
        if(useCnts)
        {
            if(changeMode)  counts = curArmCounts;
            else if(dDrop)  counts = dropCounts;
            else if(dHover) counts = hoverCounts;
            else if(dGrab)  counts = grabCounts;
            else if(dStow)  counts = stowCounts;
            if(counts  != oldCounts)
            {
                RobotLog.dd(TAG, "Moving to arm pos %d from %d", counts, curArmCounts);
                robot.armPitch.setTargetPosition(counts);
                robot.armPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armPitch.setPower(0.5);
            }
            oldCounts = counts;
        }
        else
        {
            robot.armPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if(curArmCounts > 100 && !override) apitch = 0.0;
//            if(curArmCounts < maxCounts) apitch = 0.0;
            robot.setArmSpeed(apitch, false);
        }

        RobotLog.dd(TAG, "ArmPitch = %d", curArmCounts);

        dashboard.displayPrintf(4, "extcounts %d", robot.armExtend.getCurrentPosition());
        dashboard.displayPrintf(5, "tgtcounts %d", counts);
        dashboard.displayPrintf(6, "armcounts %d", robot.armPitch.getCurrentPosition());

        if(robot.armExtend != null)
        {
            int curArmExtend = robot.armExtend.getCurrentPosition();
            if(aslide != 0.0)
            {
                RobotLog.dd(TAG, "moving slide %4.3f at %d", aslide, curArmExtend);
            }
            int ENC_SAFE = 10;
            //if(curArmExtend < (ENC_SAFE) && aslide < 0.0 && !override) aslide = 0.0;

            robot.armExtend.setPower(aslide);
        }
    }

    private void controlDrive()
    {
        if(robot.leftMotors.size() == 0 && robot.rightMotors.size() == 0) return;
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
        boolean xfst_dtl          = gpad1.pressed(ManagedGamepad.Button.L_TRIGGER);

        double raw_left     = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
        double raw_right    = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        double raw_turn     =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);

//        boolean goBox       =  gpad1.just_pressed(ManagedGamepad.Button.L_TRIGGER);
//        boolean goPit       =  gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);
//
//        if(goBox)
//        {
//            goToBox();
//            return;
//        }
//
//        if(goPit)
//        {
//            goToPit();
//            return;
//        }

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
        if(xfst_dtl) detail_speed = 0.5;

        double maxIPS = 40.0;
        double maxRPS = maxIPS/(4.0*Math.PI);
        double maxDPS = maxRPS*360.0;

        double left  = 0.0;
        double right = 0.0;
        double turn  = 0.0;

        if(fwd_step)
        {
            left  = detail_speed;
            right = detail_speed;
            //dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.FORWARD);
        }
        else if(back_step)
        {
            left  = -detail_speed;
            right = -detail_speed;
            //dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.REVERSE);
        }
        else if(left_step)
        {
            left  = -detail_speed;
            right =  detail_speed;
            //dtrn.setInitValues();
            //dtrn.ctrTurnToHeading(fHdg + step_ang, step_spd);
        }
        else if(right_step)
        {
            left  =  detail_speed;
            right = -detail_speed;
            //dtrn.setInitValues();
            //dtrn.ctrTurnToHeading(fHdg - step_ang, step_spd);
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

        //dashboard.displayPrintf(4, "TURN  %4.2f", turn);
        //dashboard.displayPrintf(5, "L_OUT %4.2f", left);
        //dashboard.displayPrintf(6, "R_OUT %4.2f", right);
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

    private boolean joyHolder = false;
    private void controlHolder()
    {
//        boolean lowerHolder      = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
//        boolean raiseHolder      = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        boolean lowerOne         = gpad2.just_pressed(ManagedGamepad.Button.D_LEFT);
        boolean raiseOne         = gpad2.just_pressed(ManagedGamepad.Button.D_RIGHT);
        double hldrSpd           = -gamepad2.left_stick_y;
        boolean overrideLims     = gpad2.pressed(ManagedGamepad.Button.L_BUMP);
        double moveDist          = 1.0;

//        lowerHolder = false;
//        raiseHolder = false;
//        if(lowerHolder)
//        {
//            //setting to false lowers holder which raises the bot
//            robot.putHolderAtStow();
//        }
//        else if (raiseHolder)
//        {
//            //setting to true raises holder
//            robot.putHolderAtLatch();
//        }

        if(Math.abs(hldrSpd) > 0.01) joyHolder = true;

        if(lowerOne)
        {
            //Lowers holder by one unit
            joyHolder = false;
            robot.moveHolder(-moveDist);
        }
        else if (raiseOne)
        {
            //Raises holder by one unit
            joyHolder = false;
            robot.moveHolder(moveDist);
        }
        else if (joyHolder)
        {
            robot.setHolderSpeed(hldrSpd, overrideLims);
        }
    }

    private boolean lastShift = false;

    private void processControllerInputs()
    {
        boolean shiftControls = gpad2.pressed(ManagedGamepad.Button.R_TRIGGER);

        if(!shiftControls)
        {
            controlHolder();
            if(lastShift)
            {
                robot.setArmSpeed(0.0, false);
                if(robot.armExtend != null) robot.armExtend.setPower(0.0);
                robot.intakeStop();
            }
        }
        else
        {
            controlArm();
        }

        lastShift = shiftControls;
    }

    private void processDriverInputs()
    {
        controlDrive();
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

            if(robot.armPitch != null)
            {
                dashboard.displayPrintf(5, "ArmPitch cnt %d",
                        robot.armPitch.getCurrentPosition());
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
                if (robot.leftMotors.size() > 0 && robot.rightMotors.size() >0)
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

    private RoRuBot robot = new RoRuBot();
    private Drivetrain dtrn = new Drivetrain();
    private Input_Shaper ishaper = new Input_Shaper();

    private DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
    private boolean useSetVel = true;

    private enum PitchState {PITCH_UP, PITCH_DOWN }
    private PitchState currentPitchState = PitchState.PITCH_UP;

    private enum FlickerState { UP, DOWN }
    private FlickerState currentFlickerState = FlickerState.UP;

    private boolean intakeOpen = false;

    private boolean pActive = false;
    private boolean eActive = false;

    private boolean lastArmTouchPressed = false;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean estimatePos = true;

    private int prevRpitch = 0;

    private ShelbyBot.OpModeType prevOpModeType = ShelbyBot.OpModeType.UNKNOWN;

    private ElapsedTime timer = new ElapsedTime();

    private static final String TAG = "SJH_TD";
}

