package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Telop Tank", group="Tele")
//@Disabled
public class TeleopTank_Driver extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Input_Shaper ishaper = new Input_Shaper();
        DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;
        double shoot_scale = 0.75;

        /* Initialize the hardware variables. */
        robot.init(this);

        if (robot.leftMotor  != null &&
            robot.rightMotor != null &&
            robot.gyro       != null)
        {
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dtrn.init(robot);
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        robot.setDriveDir(ShelbyBot.DriveDir.SWEEPER);

        double curLpushPos = L_DN_PUSH_POS;
        double curRpushPos = R_DN_PUSH_POS;
        robot.rpusher.setPosition(curRpushPos);
        robot.lpusher.setPosition(curLpushPos);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean toggle = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            gpad1.update();
            gpad2.update();

            boolean use_auto_shoot = false;

            boolean toggle_run_mode = gpad1.just_pressed(ManagedGampad.Button.X);
            boolean invert_drive_dir = gpad1.just_pressed(ManagedGampad.Button.Y);
            boolean toggle_rpusher = gpad1.just_pressed(ManagedGampad.Button.R_TRIGGER);
            boolean toggle_lpusher = gpad1.just_pressed(ManagedGampad.Button.L_TRIGGER);
            boolean toggle_float = gpad1.just_pressed(ManagedGampad.Button.B);
            boolean decr_shoot_scale = gpad2.just_pressed(ManagedGampad.Button.D_DOWN);
            boolean incr_shoot_scale = gpad2.just_pressed(ManagedGampad.Button.D_UP);
            boolean toggle_shoot_mtr = gpad2.just_pressed(ManagedGampad.Button.R_TRIGGER);
            boolean toggle_bshoot_mtr = gpad2.just_pressed(ManagedGampad.Button.L_TRIGGER);
            boolean auto_shoot = gpad2.just_pressed(ManagedGampad.Button.L_BUMP);

            double shooter = gpad2.value(ManagedGampad.AnalogInput.R_TRIGGER_VAL);
            double bkwr_shooter = gpad2.value(ManagedGampad.AnalogInput.L_TRIGGER_VAL);
            double elev = gpad2.value(ManagedGampad.AnalogInput.L_STICK_Y);
            double sweep = gpad2.value(ManagedGampad.AnalogInput.R_STICK_Y);
            robot.elevMotor.setPower(elev);
            robot.sweepMotor.setPower(sweep);

            // Run wheels in tank mode
            // (note: The joystick goes negative when pushed forwards, so negate it)
            double left = -gpad1.value(ManagedGampad.AnalogInput.L_STICK_Y);
            double right = -gpad2.value(ManagedGampad.AnalogInput.R_STICK_Y);
            left = ishaper.shape(left);
            right = ishaper.shape(right);

            double speed = -right;
            double turn = gpad1.value(ManagedGampad.AnalogInput.R_STICK_X);

            switch (driveType)
            {
                case TANK_DRIVE:
                    robot.leftMotor.setPower(left);
                    robot.rightMotor.setPower(right);
                    break;

                case ARCADE_DRIVE:
                    robot.leftMotor.setPower(speed + turn);
                    robot.rightMotor.setPower(speed - turn);
                    break;

                case CAR_DRIVE:
                    robot.leftMotor.setPower(((1 - Math.abs(turn))  * speed +
                                              (1 - Math.abs(speed)) * turn  +
                                              turn + speed) / 2);
                    robot.rightMotor.setPower(((1 - Math.abs(turn))  * speed -
                                               (1 - Math.abs(speed)) * turn  -
                                               turn + speed) / 2);
                    break;
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
                robot.rpusher.setPosition(curRpushPos);
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
                robot.lpusher.setPosition(curLpushPos);
            }

            if(toggle_float)
            {
                if(zeroPwr == DcMotor.ZeroPowerBehavior.BRAKE)
                    zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;
                else
                    zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
                robot.leftMotor.setZeroPowerBehavior(zeroPwr);
                robot.rightMotor.setZeroPowerBehavior(zeroPwr);
            }

            if(auto_shoot)
            {
                RobotLog.ii("SJH", "AUTOSHOOT");
                robot.shotmotor1.setPower(shoot_scale);
                robot.shotmotor2.setPower(shoot_scale);
                robot.sweepMotor.setPower(-1.0);
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
                    robot.sweepMotor.setPower(-1.0);
                    robot.elevMotor.setPower(-1.0);
                    sleep(1500);
                    robot.shotmotor1.setPower(0);
                    robot.shotmotor2.setPower(0);
                    robot.sweepMotor.setPower(0);
                    robot.elevMotor.setPower(0);
                    RobotLog.ii("SJH", "DONE AUTOSHOOT");
                }
            }

            if(invert_drive_dir)
            {
                robot.invertDriveDir();
            }

            if(toggle_run_mode)
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

            telemetry.addData("left : ",  "%.2f", left);
            telemetry.addData("right : ", "%.2f", right);
            telemetry.addData("elev : ", elev);
            telemetry.addData("sweep : ",  sweep);
            telemetry.addData("shooters", "%.2f", shooter);
            telemetry.addData("shooters", "%.2f", bkwr_shooter);
            telemetry.addData("shoot_scale", "%.2f", shoot_scale);
            telemetry.addData("SHT1CNT", "%5d", robot.shotmotor1.getCurrentPosition());
            telemetry.addData("SHT2CNT", "%5d", robot.shotmotor2.getCurrentPosition());
            telemetry.addData("zmode", "%s", robot.leftMotor.getZeroPowerBehavior());
            telemetry.update();

            // Pause for metronome tick.
            robot.waitForTick(10);
        }
    }

    private void shooter_motors(double speed)
    {
        robot.shotmotor1.setPower(speed);
        robot.shotmotor2.setPower(speed);
    }

    private enum TELEOP_DRIVE_TYPE
    {
        TANK_DRIVE,
        ARCADE_DRIVE,
        CAR_DRIVE
    }

    private final static TELEOP_DRIVE_TYPE driveType = TELEOP_DRIVE_TYPE.TANK_DRIVE;

    private final static double L_DN_PUSH_POS = 1.0;
    private final static double R_DN_PUSH_POS = 0.05;
    private final static double L_UP_PUSH_POS = 0.05;
    private final static double R_UP_PUSH_POS = 1.0;

    private ShelbyBot robot = new ShelbyBot();
    private Drivetrain dtrn = new Drivetrain();
    private ManagedGampad gpad1 = new ManagedGampad(gamepad1);
    private ManagedGampad gpad2 = new ManagedGampad(gamepad2);
}
