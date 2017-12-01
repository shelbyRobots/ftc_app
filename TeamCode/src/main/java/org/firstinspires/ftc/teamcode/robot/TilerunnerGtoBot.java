package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

import java.util.ArrayList;
import java.util.List;

public class TilerunnerGtoBot extends ShelbyImuBot
{
    public Servo    gpitch     = null;
    public Servo    gripper    = null;
           Servo    jflicker   = null;

    public List<Integer> liftPositions = new ArrayList<>(4);

    public static double JFLICKER_UP_POS = 0.1;
    public static double JFLICKER_DOWN_POS = 0.6;
    public static double JFLICKER_STOW_POS = 0.0;

    public static double GRIPPER_CLOSE_POS   = 0.90;
    public static double GRIPPER_PARTIAL_POS = 0.75;
    public static double GRIPPER_OPEN_POS    = 0.5;
           static double GRIPPER_STOW_POS    = 0.3;

    public static double GPITCH_UP_POS    = 0.16;
    public static double GPITCH_DOWN_POS  = 0.58;
    public static double GPITCH_CLEAR_POS = 0.3;
    public static double GPITCH_MIN       = 0.16;
    public static double GPITCH_MAX       = 0.9;

           static int    LIFT_AUTON_POS   = 0;

    private static final String TAG = "SJH_GTO";

    public TilerunnerGtoBot()
    {
        super();

        COUNTS_PER_MOTOR_REV = 28;
        DRIVE_GEARS = new double[]{20.0, 1.0};

        WHEEL_DIAMETER_INCHES = 4.0;
        TUNE = 1.00;

        BOT_WIDTH  = 15.0f; //Wheel width
        BOT_LENGTH = 18.0f;

        REAR_OFFSET = 9.0f;
        FRNT_OFFSET = BOT_LENGTH - REAR_OFFSET;

        CAMERA_X_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;

        LEFT_DIR  = DcMotorSimple.Direction.REVERSE;
        RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

        gyroInverted = false;
    }

    @Override
    public void init(LinearOpMode op)
    {
        computeCPI();

        initOp(op);
        initDriveMotors();
        initCollectorLifter();
        initPushers();
        initSensors();
        initCapabilities();
    }

    @Override
    protected void initDriveMotors()
    {
        RobotLog.dd(TAG, "GTO initDriveMotors");
        super.initDriveMotors();
    }

    @Override
    protected void initCollectorLifter()
    {
        RobotLog.dd(TAG, "GTO initCollectorLifter");
        try  //Collector
        {
            gpitch = hwMap.servo.get("gpitch");
            gripper = hwMap.servo.get("gripper");
            elevMotor = hwMap.dcMotor.get("elevmotor");

            elevMotor.setPower(0);
            elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elevMotor.setDirection(DcMotor.Direction.REVERSE);

            int ELEV_COUNTS_PER_MOTOR_REV = 4;
            double ELEV_GEAR_ONE = 72;
            double ELEV_CPR = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE;
            double ELEV_WHEEL_DIAM = 1.5; //2.35
            double ELEV_CPI = ELEV_CPR/(Math.PI * ELEV_WHEEL_DIAM);
            double LIFT_SCALE = 1.0;

            LIFT_AUTON_POS = ((int)( 0.25/LIFT_SCALE * ELEV_CPI));

            liftPositions.add((int)( 0.25/LIFT_SCALE * ELEV_CPI));
            liftPositions.add((int)( 6.75/LIFT_SCALE * ELEV_CPI));
            liftPositions.add((int)(12.75/LIFT_SCALE * ELEV_CPI));
            liftPositions.add((int)(18.25/LIFT_SCALE * ELEV_CPI));

            if(name.equals("GTO1"))
            {
                elevMotor.setDirection(DcMotor.Direction.FORWARD);
                GRIPPER_CLOSE_POS = 0.84;
                GRIPPER_OPEN_POS = 0.5;
                GRIPPER_PARTIAL_POS = 0.62;
                GRIPPER_STOW_POS    = 0.3;

                GPITCH_DOWN_POS = 0.62;
                GPITCH_UP_POS = 0.1;
                GPITCH_CLEAR_POS = 0.4;
                GPITCH_MIN = 0.2;
                GPITCH_MAX = 0.9;
            }
            else if(name.equals("GTO2"))
            {
                elevMotor.setDirection(DcMotor.Direction.FORWARD);
                GRIPPER_CLOSE_POS = 0.98;
                GRIPPER_OPEN_POS = 0.2;
                GRIPPER_PARTIAL_POS = 0.45;

                GPITCH_DOWN_POS = 0.58;
                GPITCH_UP_POS = 0.08;
                GPITCH_CLEAR_POS = 0.25;
                GPITCH_MIN = 0.1;
                GPITCH_MAX = 0.65;
            }
            capMap.put("collector", true);

            RobotLog.dd(TAG, "Gripper close %.2f open %.2f part %.2f",
                    GRIPPER_CLOSE_POS, GRIPPER_OPEN_POS, GRIPPER_PARTIAL_POS);
            RobotLog.dd(TAG, "Gpitch up %.2f down %.2f min %.2f max %.2f",
                    GPITCH_UP_POS, GPITCH_DOWN_POS, GPITCH_MIN, GPITCH_MAX);
            RobotLog.dd(TAG, "Elev %s", elevMotor.getDirection());
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map in initCollectorLifter\n" + e.toString());
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
        RobotLog.dd(TAG, "GTO initPushers");
        try
        {
            jflicker = hwMap.servo.get("jflicker");

            if(name.equals("GTO1"))
            {
                JFLICKER_UP_POS   = 0.71;
                JFLICKER_DOWN_POS = 0.0;
                JFLICKER_STOW_POS = 1.0;
            }
            else if(name.equals("GTO2"))
            {
                JFLICKER_UP_POS   = 0.58;
                JFLICKER_DOWN_POS = 0.09;
                JFLICKER_STOW_POS = 1.0;
            }

            capMap.put("pusher", true);

            RobotLog.dd(TAG, "Jflicker up %.2f down %.2f",
                    JFLICKER_UP_POS, JFLICKER_DOWN_POS);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map in initPushers\n" + e.toString());
        }
    }

    public void setElevAuton()
    {
        if(elevMotor != null)
        {
            elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevMotor.setTargetPosition(LIFT_AUTON_POS);
            elevMotor.setPower(0.3);
        }
    }

    public void setElevZero()
    {
        if(elevMotor != null)
        {
            elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevMotor.setTargetPosition(10);
            elevMotor.setPower(0.2);
        }
    }

    public void initElevZero()
    {
        if(elevMotor != null)
        {
            elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void stowFlicker ()
    {
        if(jflicker != null) jflicker.setPosition(JFLICKER_STOW_POS);
    }
    public void deployFlicker()
    {
        if(jflicker != null) jflicker.setPosition(JFLICKER_DOWN_POS);
    }
    public void raiseFlicker()
    {
        if(jflicker != null) jflicker.setPosition(JFLICKER_UP_POS);
    }
    public void closeGripper()
    {
        if(jflicker != null)gripper.setPosition(GRIPPER_CLOSE_POS);
    }
    public void openGripper()
    {
        gripper.setPosition(GRIPPER_OPEN_POS);
    }
    public void stowGripper() { gripper.setPosition(GRIPPER_STOW_POS); }
    public void partialGripper()
    {
        gripper.setPosition(GRIPPER_PARTIAL_POS);
    }
    public void retractGpitch()
    {
        if(gpitch != null) gpitch.setPosition(GPITCH_UP_POS);
    }
    public void deployGpitch()
    {
        if(gpitch != null) gpitch.setPosition(GPITCH_DOWN_POS);
    }
    public void clearGpitch()
    {
        if(gpitch != null) gpitch.setPosition(GPITCH_CLEAR_POS);
    }
}
