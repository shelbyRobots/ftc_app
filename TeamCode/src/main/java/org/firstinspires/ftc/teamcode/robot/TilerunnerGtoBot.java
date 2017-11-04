package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.Locale;

public class TilerunnerGtoBot extends ShelbyBot
{
    private CommonUtil com = CommonUtil.getInstance();

    private Orientation angles;
    private Acceleration gravity;

    private ElapsedTime imuTimer = new ElapsedTime();

    public Servo    gpitch     = null;
    public Servo    gripper    = null;
    public Servo    jflicker   = null;

    private static final int ELEV_COUNTS_PER_MOTOR_REV = 4;
    private static final double ELEV_GEAR_ONE = 72;
    private static final double ELEV_CPR = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE;
    private static final double ELEV_WHEEL_DIAM = 1.5;
    private static final double ELEV_CPI = ELEV_CPR/(Math.PI * ELEV_WHEEL_DIAM);
    private static final double LIFT_SCALE = 1.0;

    public static final int LIFT_POS_A = (int)( 0.25/LIFT_SCALE * ELEV_CPI);
    public static final int LIFT_POS_B = (int)( 6.75/LIFT_SCALE * ELEV_CPI);
    public static final int LIFT_POS_C = (int)(12.75/LIFT_SCALE * ELEV_CPI);
    public static final int LIFT_POS_D = (int)(18.25/LIFT_SCALE * ELEV_CPI);

    public static double autonEndHdg = 0.0;

    private static final String TAG = "SJH_GTO";

    public TilerunnerGtoBot()
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

        LEFT_DIR  = DcMotorSimple.Direction.REVERSE;
        RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

        gyroInverted = false;
    }

    @Override
    public void init(LinearOpMode op)
    {
        System.out.println("In TilerunnerGtoBot.init");
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
        super.initDriveMotors();
    }

    @Override
    protected void initCollectorLifter()
    {
        try  //Collector
        {
            gpitch = hwMap.servo.get("gpitch");
            gripper = hwMap.servo.get("gripper");
            elevMotor = hwMap.dcMotor.get("elevmotor");

            elevMotor.setDirection(DcMotor.Direction.FORWARD);
            elevMotor.setPower(0);
            elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            capMap.put("collector", true);
        }
        catch (Exception e)
        {
            RobotLog.ee("SJH", "ERROR get hardware map\n" + e.toString());
        }
    }

    @Override
    protected void initPushers()
    {
        try
        {
            jflicker = hwMap.servo.get("jflicker");
            capMap.put("pusher", true);
        }
        catch (Exception e)
        {
            RobotLog.ee("SJH", "ERROR get hardware map\n" + e.toString());
        }
    }

    @Override
    protected void initSensors()
    {
        System.out.println("In TilerunnerGtoBot.initSensors");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMU_IMUCalibration.json";
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        try
        {
            imu = (BNO055IMU) com.getHardwareMap().get("imu");
            imu.initialize(parameters);
            capMap.put("sensor", true);
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "ERROR get imu\n" + e.toString());
        }
    }

    @Override
    public boolean calibrateGyro()
    {
        //Callibration performed internally and assisted by loading file from offline calib.
        RobotLog.dd(TAG, "CalibrateGyro isAccelCal %s isGyroCal %s calStatus %s sysStatus %s",
                imu.isAccelerometerCalibrated(),
                imu.isGyroCalibrated(),
                imu.getCalibrationStatus(),
                imu.getSystemStatus());

        gyroReady = true;
        return true;
    }

    public void resetGyro()
    {
    }

    @Override
    public double getGyroHdg()
    {
        double startTime = imuTimer.milliseconds();
        getGyroAngles();
        double yaw = angles.firstAngle;
        double endTime = imuTimer.milliseconds();
        double imuTime = endTime - startTime;
        RobotLog.dd("IMU", String.format(Locale.US,
                "%.2f,%.4f", imuTime, yaw));

        return yaw;
    }

    private void getGyroAngles()
    {
        angles = imu.getAngularOrientation();
    }
}
