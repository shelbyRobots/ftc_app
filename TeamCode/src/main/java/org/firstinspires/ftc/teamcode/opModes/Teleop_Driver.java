package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.TilerunnerGtoBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@SuppressWarnings("unused")
@TeleOp(name="Telop Driver", group="Tele")
//@Disabled
public class Teleop_Driver extends InitLinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);
        Input_Shaper ishaper = new Input_Shaper();
        DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
        boolean useSetVel = true;

        /* Initialize the hardware variables. */
        robot.init(this);


        if (robot.leftMotor  != null &&
            robot.rightMotor != null)
        {
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dtrn.init(robot);

            dtrn.setRampUp(false);
            dtrn.setRampDown(false);
            //robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        }

        if(robot.gripper != null)
        {
            robot.gripper.setPosition(GRIPPER_CLOSE_POS);
        }

        if(robot.gpitch != null)
        {
            robot.gpitch.setPosition(GPITCH_UP_POS);
        }

        if(robot.elevMotor != null)
        {
            robot.elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevMotor.setPower(0.0);
            robot.elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Send telemetry message to signify robot waiting;
        dashboard.displayText(0, "Hello Driver");

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted())
        {
            gpad1.update();
            gpad2.update();
            gpad1.log(1);
            gpad2.log(2);
            idle();
        }

        RobotLog.dd(TAG, "Telop_Driver starting");

        boolean toggle = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            gpad1.update();
            gpad2.update();

            boolean toggle_run_mode   = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean invert_drive_dir  = gpad1.just_pressed(ManagedGamepad.Button.Y);

            boolean step_driveType    = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean toggle_float      = gpad1.just_pressed(ManagedGamepad.Button.B);
            boolean toggle_vel        = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);

            boolean left_step         = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean right_step        = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean fwd_step          = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean back_step         = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);

            boolean lowerElev         = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean raiseElev         = gpad2.just_pressed(ManagedGamepad.Button.D_UP);

            boolean gripper_open_par  = gpad2.pressed(ManagedGamepad.Button.A);
            boolean gripper_open      = gpad2.pressed(ManagedGamepad.Button.B);
            boolean toggle_gpitch     = gpad2.just_pressed(ManagedGamepad.Button.X);
            boolean toggle_jflicker   = gpad2.just_pressed(ManagedGamepad.Button.Y);

            double raw_left     = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            double raw_right    = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            double raw_turn     =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);

            double elev         = -gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            double pitch        = -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);

            DcMotor.ZeroPowerBehavior zeroPowBeh = DcMotor.ZeroPowerBehavior.UNKNOWN;

            if(robot.leftMotor  != null) zeroPowBeh = robot.leftMotor.getZeroPowerBehavior();

            // Run wheels in tank mode
            // (note: The joystick goes negative when pushed forwards, so negate it)

            dashboard.displayPrintf(0, "TMODE " + driveType);
            dashboard.displayPrintf(1, "L_IN %4.2f", raw_left);
            dashboard.displayPrintf(2, "R_IN %4.2f", raw_right);
            double left  = ishaper.shape(raw_left,  0.1);
            double right = ishaper.shape(raw_right, 0.1);
            double turn  = ishaper.shape(raw_turn, 0.1);
            elev  = ishaper.shape(elev);

            double outPitch = Range.scale(pitch, -1.0, 1.0, GPITCH_MIN, GPITCH_MAX);

            double speed = right;
            double arcadeTurnScale = 0.5;

            switch (driveType)
            {
                case TANK_DRIVE:
                    left  = left;
                    right = right;
                    break;

                case ARCADE_DRIVE:
                    turn = (1 - Math.abs(speed)) * turn;
                    if(turn < 0.95) turn *= arcadeTurnScale;
                    left  = speed + turn;
                    right = speed - turn;
                    break;

                case CAR_DRIVE:
                    left =((1 - Math.abs(turn)) * speed +
                           (1 - Math.abs(speed)) * turn  +
                           turn + speed) / 2;


                    right = ((1 - Math.abs(turn))  * speed -
                            (1 - Math.abs(speed)) * turn  -
                            turn + speed) / 2;

                    break;
            }

            double max = Math.max(Math.abs(left), Math.abs(right));
            if(max > 1.0)
            {
                left  /= max;
                right /= max;
            }

            if(toggle_vel) useSetVel = !useSetVel;

            double step_dist = 2.0;
            double step_spd  = 0.4;
            double step_ang  = 5.0;

            if(fwd_step)
            {
                RobotLog.dd(TAG, "In fwd_step");
                dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.FORWARD);
                RobotLog.dd(TAG, "Done fwd_step");
            }
            else if(back_step)
            {
                RobotLog.dd(TAG, "In back_step");
                dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.REVERSE);
                RobotLog.dd(TAG, "Done back_step");
            }
            else if(left_step)
            {
                RobotLog.dd(TAG, "In left_step");
                dtrn.ctrTurnLinear(step_ang, step_spd);
                RobotLog.dd(TAG, "Done left_step");
            }
            else if(right_step)
            {
                RobotLog.dd(TAG, "In right_step");
                dtrn.ctrTurnLinear(-step_ang, step_spd);
                RobotLog.dd(TAG, "Done right_step");
            }
            else
            {
                lex = (DcMotorEx)(robot.leftMotor);
                rex = (DcMotorEx)(robot.rightMotor);
                double maxIPS = 60.0;
                double maxRPS = maxIPS/(4.0*Math.PI);
                double maxDPS = maxRPS*360.0;

                if(useSetVel)
                {
                    double lspd = raw_left;
                    double rspd = raw_right;
                    if(driveType == TELEOP_DRIVE_TYPE.ARCADE_DRIVE)
                    {
                        lspd = raw_right + raw_turn*arcadeTurnScale;
                        rspd = raw_right - raw_turn*arcadeTurnScale;
                        double vmax = Math.max(Math.abs(lspd), Math.abs(rspd));
                        if(vmax > 1.0)
                        {
                            lspd /= vmax;
                            rspd /= vmax;
                        }
                    }
                    lex.setVelocity(maxDPS*lspd,  AngleUnit.DEGREES);
                    rex.setVelocity(maxDPS*rspd, AngleUnit.DEGREES);
                }
                else
                {
                    robot.leftMotor.setPower(left);
                    robot.rightMotor.setPower(right);
                }
            }

            dashboard.displayPrintf(3, "SPEED %4.2f", speed);
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

            if(lowerElev && curElevIdx > 0)
            {
                curElevIdx--;
                robot.elevMotor.setTargetPosition(elevPositions[curElevIdx]);
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.75;
                robot.elevMotor.setPower(elev);
            }
            else if(raiseElev && curElevIdx < 3)
            {
                curElevIdx++;
                robot.elevMotor.setTargetPosition(elevPositions[curElevIdx]);
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.85;
                robot.elevMotor.setPower(elev);
            }
            else if(Math.abs(elev) > 0.001)
            {
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(robot.elevMotor.getCurrentPosition() < 10) elev = 0.0;
                robot.elevMotor.setPower(elev);
            }

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
                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                else
                {
                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
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
//                    case CAR_DRIVE:
//                        driveType = TELEOP_DRIVE_TYPE.TANK_DRIVE;
//                        break;
                }
            }

            // Gripper (a: Somewhat Open, b: all the way open, neither: closed)
            if (gripper_open)
                robot.gripper.setPosition(GRIPPER_OPEN_POS);
            else if (gripper_open_par)
                robot.gripper.setPosition(GRIPPER_MID_POS);
            else
                robot.gripper.setPosition(GRIPPER_CLOSE_POS);

            // Pitch (Gripper angle servo) (x: toggles between closed and open position)
            if (toggle_gpitch)
            {
                currentPitchState = (currentPitchState == PitchState.PITCH_UP) ?
                                            PitchState.PITCH_DOWN : PitchState.PITCH_UP;

                if (currentPitchState == PitchState.PITCH_UP)
                    robot.gpitch.setPosition(GPITCH_UP_POS);
                else if (currentPitchState == PitchState.PITCH_DOWN)
                    robot.gpitch.setPosition(GPITCH_DOWN_POS);
            }
            else if(Math.abs(pitch) > 0.05)
            {
                robot.gpitch.setPosition(outPitch);
            }

            // Jewel Flicker (y: toggles between up and down positions)
            if (toggle_jflicker)
            {
                currentFlickerState = (currentFlickerState == FlickerState.DOWN) ?
                                              FlickerState.UP : FlickerState.DOWN;

                if (currentFlickerState == FlickerState.DOWN)
                    robot.jflicker.setPosition(JFLICKER_DOWN_POS);
                else if (currentFlickerState == FlickerState.UP)
                    robot.jflicker.setPosition(JFLICKER_UP_POS);
            }

            // Pause for metronome tick.
            robot.waitForTick(10);
        }
    }

    private enum TELEOP_DRIVE_TYPE
    {
        TANK_DRIVE,
        ARCADE_DRIVE,
        CAR_DRIVE
    }

    private int elevPositions[] =
            {
                    TilerunnerGtoBot.LIFT_POS_A,
                    TilerunnerGtoBot.LIFT_POS_B,
                    TilerunnerGtoBot.LIFT_POS_C,
                    TilerunnerGtoBot.LIFT_POS_D
            };

    private int curElevIdx = 0;

    private TELEOP_DRIVE_TYPE driveType = TELEOP_DRIVE_TYPE.ARCADE_DRIVE;

    public  final static double JFLICKER_UP_POS   = 0.71;
    public  final static double JFLICKER_DOWN_POS = 0.0;

    public  final static double GRIPPER_CLOSE_POS = 0.85;
    public  final static double GRIPPER_OPEN_POS  = 0.0;
    public  final static double GRIPPER_MID_POS   = 0.71;

    public  final static double GPITCH_DOWN_POS = 0.64;
    public  final static double GPITCH_UP_POS   = 0.1;
    public  final static double GPITCH_MIN      = 0.1;
    public  final static double GPITCH_MAX      = 0.8;

    // GTO1
//    public  final static double JFLICKER_UP_POS   = 0.1;
//    public  final static double JFLICKER_DOWN_POS = 0.75;
//
//    public  final static double GRIPPER_CLOSE_POS = 0.88;
//    public  final static double GRIPPER_OPEN_POS  = 0.6;
//    public  final static double GRIPPER_MID_POS   = 0.75;
//
//    public  final static double GPITCH_DOWN_POS = 0.4;
//    public  final static double GPITCH_UP_POS   = 0.9;
//    public  final static double GPITCH_MIN      = 0.2;
//    public  final static double GPITCH_MAX      = 0.9;

    public final double minPwr = 0.10;
    public final double minTrn = 0.10;

    private DcMotorEx lex = null;
    private DcMotorEx rex = null;

    private TilerunnerGtoBot robot = new TilerunnerGtoBot();
    private Drivetrain dtrn = new Drivetrain();

    enum PitchState {PITCH_UP, PITCH_DOWN };
    private PitchState currentPitchState = PitchState.PITCH_UP;

    private enum FlickerState { UP, DOWN }
    private FlickerState currentFlickerState = FlickerState.UP;

    private static final String TAG = "SJH_TD";
}