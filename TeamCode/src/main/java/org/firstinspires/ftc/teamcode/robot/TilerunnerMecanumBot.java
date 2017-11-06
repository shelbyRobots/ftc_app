package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

import java.util.HashMap;
import java.util.Map;

public class TilerunnerMecanumBot extends TilerunnerGtoBot
{
    public DcMotor lfMotor = null;
    public DcMotor lrMotor = null;
    public DcMotor rfMotor = null;
    public DcMotor rrMotor = null;

    public Servo rgripper    = null;

    public double RGRIPPER_CLOSE_POS   = 0.83;
    public double RGRIPPER_PARTIAL_POS = 0.75;
    public double RGRIPPER_OPEN_POS    = 0.5;

    private static final String TAG = "SJH_MEC";

    public TilerunnerMecanumBot()
    {
        super();

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

        gyroInverted = false;
    }

    @Override
    protected void initDriveMotors()
    {
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

        for(DcMotor mot : motors.values())
        {
            if(mot != null)
            {
                mot.setPower(0);
                mot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mot.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    protected void initCollectorLifter()
    {
        gpitch = hwMap.servo.get("gpitch");
        gripper = hwMap.servo.get("gripper");
        gripper = hwMap.servo.get("rgripper");
        elevMotor = hwMap.dcMotor.get("elevmotor");

        elevMotor.setPower(0);
        elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevMotor.setDirection(DcMotor.Direction.REVERSE);

        double ELEV_COUNTS_PER_MOTOR_REV = 28;
        double ELEV_GEAR_ONE = 60.0;
        double ELEV_GEAR_TWO = 40.0/16.0;
        double ELEV_CPR = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE * ELEV_GEAR_TWO;
        double ELEV_ARM_LENGTH = 14;
        double ELEV_CPI = ELEV_CPR/(Math.PI * 2 * ELEV_ARM_LENGTH);
        double LIFT_SCALE = 1.0;

        liftPositions.add((int)( 0.25/LIFT_SCALE * ELEV_CPI));
        liftPositions.add((int)( 6.75/LIFT_SCALE * ELEV_CPI));
        liftPositions.add((int)(12.75/LIFT_SCALE * ELEV_CPI));
        liftPositions.add((int)(18.25/LIFT_SCALE * ELEV_CPI));

        GRIPPER_CLOSE_POS = 1.0;
        GRIPPER_OPEN_POS = 0.7;
        GRIPPER_PARTIAL_POS = 0.85;

        RGRIPPER_CLOSE_POS = 0.7;
        RGRIPPER_OPEN_POS = 0.85;
        RGRIPPER_PARTIAL_POS = 0.77;

        GPITCH_UP_POS = 0.71;
        GPITCH_DOWN_POS = 0.0;
        GPITCH_MIN = 0.0;
        GPITCH_MAX = 0.8;
    }

    @Override
    protected void initPushers()
    {
        JFLICKER_UP_POS = 0.1;
        JFLICKER_DOWN_POS = 0.6;
    }
}