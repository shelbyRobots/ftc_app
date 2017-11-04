package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.HashMap;
import java.util.Map;

public class TilerunnerMecanumBot extends TilerunnerGtoBot
{
    private CommonUtil com = CommonUtil.getInstance();

    public DcMotor lfMotor = null;
    public DcMotor lrMotor = null;
    public DcMotor rfMotor = null;
    public DcMotor rrMotor = null;

    private Map<String, DcMotor> motors = new HashMap<>();

    private Orientation angles;
    private Acceleration gravity;

    public Servo gpitch     = null;
    public Servo gripper    = null;
    public Servo jflicker   = null;

    private static final int ELEV_COUNTS_PER_MOTOR_REV = 28;
    private static final double ELEV_GEAR_ONE = 60;
    private static final double ELEV_GEAR_TWO = 40.0/16.0;
    private static final double ELEV_CPR = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE * ELEV_GEAR_TWO;
    private static final double ELEV_ARM_LENGTH = 14;
    private static final double ELEV_CPI = ELEV_CPR/(Math.PI * 2 * ELEV_ARM_LENGTH);
    private static final double LIFT_SCALE = 2.0;

    public static final int LIFT_POS_A = (int)( 0.00/LIFT_SCALE * ELEV_CPI);
    public static final int LIFT_POS_B = (int)( 6.25/LIFT_SCALE * ELEV_CPI);
    public static final int LIFT_POS_C = (int)(12.25/LIFT_SCALE * ELEV_CPI);
    public static final int LIFT_POS_D = (int)(18.25/LIFT_SCALE * ELEV_CPI);


    private ElapsedTime imuTimer = new ElapsedTime();

    public TilerunnerMecanumBot()
    {
        super();

        COUNTS_PER_MOTOR_REV = 28;
        DRIVE_GEARS = new double[]{20.0, 1.0};

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
            capMap.put("drivetrain", true);

            gpitch = hwMap.servo.get("gpitch");
            gripper = hwMap.servo.get("gripper");
            elevMotor = hwMap.dcMotor.get("elevmotor");

            elevMotor.setDirection(DcMotor.Direction.FORWARD);
            elevMotor.setPower(0);
            elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
}