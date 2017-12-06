package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

public class TilerunnerMecanumBot extends TilerunnerGtoBot
{
    public DcMotor lfMotor = null;
    public DcMotor lrMotor = null;
    public DcMotor rfMotor = null;
    public DcMotor rrMotor = null;

    private static final String TAG = "SJH_MEC";

    public TilerunnerMecanumBot()
    {
        super();

        name= "MEC";

        COUNTS_PER_MOTOR_REV = 28;
        DRIVE_GEARS = new double[]{40.0, 1.0};

        WHEEL_DIAMETER_INCHES = 4.0;
        TUNE = 1.00;

        BOT_WIDTH  = 14.9f; //Wheel width
        BOT_LENGTH = 18.0f;

        REAR_OFFSET = 9.0f;
        FRNT_OFFSET = BOT_LENGTH - REAR_OFFSET;

        CAMERA_X_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;

        LEFT_DIR  = DcMotorSimple.Direction.FORWARD;
        RIGHT_DIR = DcMotorSimple.Direction.REVERSE;

        gyroInverted = false;
    }

    @Override
    protected void initDriveMotors()
    {
        RobotLog.dd(TAG, "Initializing mecanum drive motors");
        try  //Drivetrain
        {
            lfMotor = hwMap.dcMotor.get("FL");
            lrMotor = hwMap.dcMotor.get("BL");
            rfMotor = hwMap.dcMotor.get("FR");
            rrMotor = hwMap.dcMotor.get("BR");
            motors.put("FR", rfMotor);
            motors.put("BR", rrMotor);
            motors.put("BL", lrMotor);
            motors.put("FL", lfMotor);
            leftMotors.add(numLmotors++, lfMotor);
            leftMotors.add(numLmotors++, lrMotor);
            rightMotors.add(numRmotors++, rfMotor);
            rightMotors.add(numRmotors++, rrMotor);
            capMap.put("drivetrain", true);
        }
        catch (Exception e)
        {
            RobotLog.ee("SJH", "ERROR get hardware map\n" + e.toString());
        }

        int mnum = 0;
        for(DcMotor mot : motors.values())
        {
            if(mot != null)
            {
                mot.setPower(0);
                mot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mot.setDirection(LEFT_DIR);

                if (mot instanceof DcMotorEx)
                {
                    DcMotorEx lex = (DcMotorEx) mot;
                    PIDCoefficients pid;
                    pid = lex.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                    RobotLog.dd(TAG, "RUN_TO_POS Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                            mnum, pid.p, pid.i, pid.d);
                    pid = lex.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                    RobotLog.dd(TAG, "RUN_USING_ENC Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                            mnum, pid.p, pid.i, pid.d);
                    //pid = lex.getPIDCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //RobotLog.dd(TAG, "RUN_WITHOUT_ENC Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                    //        mnum, pid.p, pid.i, pid.d);
                }
            }
            mnum++;
        }

        rfMotor.setDirection(RIGHT_DIR);
        rrMotor.setDirection(RIGHT_DIR);
    }

    @Override
    protected void initCollectorLifter()
    {
        try
        {
            gripper = hwMap.servo.get("gripper");
            elevMotor = hwMap.dcMotor.get("elevmotor");

            elevMotor.setPower(0);
            elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elevMotor.setDirection(DcMotor.Direction.FORWARD);

            ELEV_COUNTS_PER_MOTOR_REV = 28;
            ELEV_GEAR_ONE = 40.0;
            double ELEV_GEAR_TWO = 54.0 / 16.0;
            double ELEV_GEAR_THREE = 40.0 / 15.0;
            ELEV_CPR = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE * ELEV_GEAR_TWO *
               ELEV_GEAR_THREE;
            double ELEV_ARM_LENGTH = 16;
            ELEV_WHEEL_DIAM = 2 * ELEV_ARM_LENGTH;
            LIFT_SCALE = 1.0;
            ELEV_CPI = ELEV_CPR/(Math.PI * ELEV_WHEEL_DIAM)/LIFT_SCALE;

            GRIPPER_CLOSE_POS    = 0.68;
            GRIPPER_OPEN_POS     = 0.84;
            GRIPPER_PARTIAL_POS  = 0.78;
            GRIPPER_STOW_POS     = 0.90;

            GPITCH_UP_POS         = 0.86;
            GPITCH_DOWN_POS       = 0.36;
            GPITCH_CLEAR_POS      = 0.6;
            GPITCH_MIN            = 0.0;
            GPITCH_MAX            = 0.8;

            LIFT_AUTON_POS = (int)( 5.0 * ELEV_CPI);
            LIFT_ZERO_POS  = (int)( 0.5 * ELEV_CPI);
            LIFT_DROP_POS  = (int)( 2.0 * ELEV_CPI);
            LIFT_TIER2_POS = (int)( 7.0 * ELEV_CPI);

            liftPositions.add((int)( 0.25  * ELEV_CPI));
            liftPositions.add((int)( 7.5 * ELEV_CPI));
            liftPositions.add((int)(14.0 * ELEV_CPI));
            liftPositions.add((int)(21.0 * ELEV_CPI));
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR initCollector get hardware map\n" + e.toString());
        }

        try
        {
            gpitch = hwMap.servo.get("gpitch");
        }
        catch (Exception e)
        {
            RobotLog.ww(TAG, "WARNING initCollectorLifter - no gpitch");
        }
    }

    @Override
    protected void initPushers()
    {
        try
        {
            jflicker = hwMap.servo.get("jflicker");

            JFLICKER_UP_POS   = 0.72;
            JFLICKER_DOWN_POS = 0.12;
            JFLICKER_STOW_POS = 0.86 ;

            capMap.put("pusher", true);

            RobotLog.dd(TAG, "Jflicker up %.2f down %.2f",
                    JFLICKER_UP_POS, JFLICKER_DOWN_POS);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map\n" + e.toString());
        }
    }
}