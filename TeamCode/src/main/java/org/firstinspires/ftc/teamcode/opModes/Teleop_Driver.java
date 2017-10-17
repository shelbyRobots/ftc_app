package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
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
        DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;
        double shoot_scale = 0.75;
        boolean useSetVel = true;

        /* Initialize the hardware variables. */
        robot.init(this);

        if (robot.leftMotor  != null &&
            robot.rightMotor != null)
        {
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dtrn.init(robot);
            //robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);
        }

        // Send telemetry message to signify robot waiting;
        dashboard.displayText(0, "Hello Driver");

//        if(dtrnType == Drivetrain.DrivetrainType.RWD_2_2X40)
//        {
//            robot.LEFT_DIR = DcMotorSimple.Direction.REVERSE;
//            robot.RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
//        }

        double curLpushPos = L_DN_PUSH_POS;
        double curRpushPos = R_DN_PUSH_POS;
        if(robot.rpusher != null) robot.rpusher.setPosition(curRpushPos);
        if(robot.lpusher != null) robot.lpusher.setPosition(curLpushPos);

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

            boolean use_auto_shoot = false;

            boolean toggle_run_mode   = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean invert_drive_dir  = gpad1.just_pressed(ManagedGamepad.Button.Y);
            boolean toggle_rpusher    = gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);
            boolean toggle_lpusher    = gpad1.just_pressed(ManagedGamepad.Button.L_TRIGGER);
            boolean step_driveType    = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean toggle_float      = gpad1.just_pressed(ManagedGamepad.Button.B);
            boolean toggle_vel        = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decr_shoot_scale  = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean incr_shoot_scale  = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
            boolean toggle_shoot_mtr  = gpad2.just_pressed(ManagedGamepad.Button.R_TRIGGER);
            boolean toggle_bshoot_mtr = gpad2.just_pressed(ManagedGamepad.Button.L_TRIGGER);
            boolean auto_shoot        = gpad2.just_pressed(ManagedGamepad.Button.L_BUMP);

            double left         = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            double right        = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            double turn         =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
            double shooter      =  gpad2.value(ManagedGamepad.AnalogInput.R_TRIGGER_VAL);
            double bkwr_shooter =  gpad2.value(ManagedGamepad.AnalogInput.L_TRIGGER_VAL);
            double elev         =  gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            double sweep        =  gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);

            int shotmotor1Pos = 0;
            int shotmotor2Pos = 0;
            DcMotor.ZeroPowerBehavior zeroPowBeh = DcMotor.ZeroPowerBehavior.UNKNOWN;

            if(robot.shotmotor1 != null) robot.shotmotor1.getCurrentPosition();
            if(robot.shotmotor2 != null) robot.shotmotor2.getCurrentPosition();
            if(robot.leftMotor  != null) zeroPowBeh = robot.leftMotor.getZeroPowerBehavior();

            if(robot.elevMotor  != null) robot.elevMotor.setPower(elev);
            if(robot.sweepMotor != null) robot.sweepMotor.setPower(sweep);

            // Run wheels in tank mode
            // (note: The joystick goes negative when pushed forwards, so negate it)

            dashboard.displayPrintf(0, "TMODE " + driveType);
            dashboard.displayPrintf(1, "L_IN %4.2f", left);
            dashboard.displayPrintf(2, "R_IN %4.2f", right);
            left  = ishaper.shape(left);
            right = ishaper.shape(right);
            turn  = ishaper.shape(turn);

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
                    if(turn < 0.75) turn /= 2.0;
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

            lex = (DcMotorEx)(robot.leftMotor);
            rex = (DcMotorEx)(robot.rightMotor);
            double maxIPS = 60.0;
            double maxRPS = maxIPS/(4.0*Math.PI);
            double maxDPS = maxRPS*360.0;


            if(useSetVel)
            {
                lex.setVelocity(maxDPS*left,  AngleUnit.DEGREES);
                rex.setVelocity(maxDPS*right, AngleUnit.DEGREES);
            }
            else
            {
                robot.leftMotor.setPower(left);
                robot.rightMotor.setPower(right);
            }

            if(decr_shoot_scale)      shoot_scale -= 0.05;
            else if(incr_shoot_scale) shoot_scale += 0.05;
            shoot_scale = Range.clip(shoot_scale, 0.0, 1.0);

            if(toggle_shoot_mtr)
            {
                toggle = !toggle;

                if(toggle)
                    shooter_motors(shoot_scale);
                else
                    shooter_motors(0.0);
            }

            if(toggle_bshoot_mtr)
            {
                toggle = !toggle;

                if(toggle)
                    shooter_motors(-1.0);
                else
                    shooter_motors(0.0);
            }

            if(toggle_rpusher)
            {
                if(curRpushPos == R_UP_PUSH_POS)
                {
                    curRpushPos = R_DN_PUSH_POS;
                }
                else
                {
                    curRpushPos = R_UP_PUSH_POS;
                }
                if(robot.rpusher != null) robot.rpusher.setPosition(curRpushPos);
            }

            if(toggle_lpusher)
            {
                if(curLpushPos == L_UP_PUSH_POS)
                {
                    curLpushPos = L_DN_PUSH_POS;
                }
                else
                {
                    curLpushPos = L_UP_PUSH_POS;
                }
                if(robot.lpusher != null) robot.lpusher.setPosition(curLpushPos);
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

            if(auto_shoot)
            {
                RobotLog.ii("SJH", "AUTOSHOOT");
                if(robot.shotmotor1 != null) robot.shotmotor1.setPower(shoot_scale);
                if(robot.shotmotor2 != null) robot.shotmotor2.setPower(shoot_scale);
                if(robot.sweepMotor != null) robot.sweepMotor.setPower(-1.0);
                dtrn.driveDistance(35.0, 0.8, Drivetrain.Direction.REVERSE);
                while(opModeIsActive() && dtrn.isBusy())
                {
                    idle();
                }
                dtrn.stopAndReset();

                RobotLog.ii("SJH", "DONE AUTOSHOOT MOVE");
                if(use_auto_shoot)
                {
                    sleep(500);
                    if(robot.sweepMotor != null) robot.sweepMotor.setPower(-1.0);
                    if(robot.elevMotor != null) robot.elevMotor.setPower(-1.0);
                    sleep(1500);
                    if(robot.shotmotor1 != null) robot.shotmotor1.setPower(0);
                    if(robot.shotmotor2 != null) robot.shotmotor2.setPower(0);
                    if(robot.sweepMotor != null) robot.sweepMotor.setPower(0);
                    if(robot.elevMotor != null) robot.elevMotor.setPower(0);
                    RobotLog.ii("SJH", "DONE AUTOSHOOT");
                }
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
                        driveType = TELEOP_DRIVE_TYPE.ARCADE_DRIVE;
                        break;
//                    case CAR_DRIVE:
//                        driveType = TELEOP_DRIVE_TYPE.TANK_DRIVE;
//                        break;
                }
            }

//            dashboard.displayPrintf(7, "shoot_scale", "%.2f", shoot_scale);
//            dashboard.displayPrintf(8, "SHT1CNT", "%5d", shotmotor1Pos);
//            dashboard.displayPrintf(9, "SHT2CNT", "%5d", shotmotor2Pos);
//            dashboard.displayPrintf(10, "zmode", "%s", zeroPowBeh);

            // Pause for metronome tick.
            robot.waitForTick(10);
        }
    }

    private void shooter_motors(double speed)
    {
        if(robot.shotmotor1 != null) robot.shotmotor1.setPower(speed);
        if(robot.shotmotor2 != null) robot.shotmotor2.setPower(speed);
    }

    private enum TELEOP_DRIVE_TYPE
    {
        TANK_DRIVE,
        ARCADE_DRIVE,
        CAR_DRIVE
    }

    private TELEOP_DRIVE_TYPE driveType = TELEOP_DRIVE_TYPE.TANK_DRIVE;

    private final static double L_DN_PUSH_POS = 1.0;
    private final static double R_DN_PUSH_POS = 0.05;
    private final static double L_UP_PUSH_POS = 0.05;
    private final static double R_UP_PUSH_POS = 1.0;

    private DcMotorEx lex = null;
    private DcMotorEx rex = null;

    private ShelbyBot robot = new TilerunnerGtoBot();
    private Drivetrain dtrn = new Drivetrain();

    private static final String TAG = "SJH_TD";
}
