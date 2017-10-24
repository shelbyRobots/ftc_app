package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        gpitch = hwMap.servo.get("gpitch");
        gripper = hwMap.servo.get("gripper");
        if(gpitch != null)capMap.put("collector", true);
    }

    @Override
    protected void initPushers()
    {
        jflicker = hwMap.servo.get("jflicker");
        if(jflicker != null)capMap.put("pusher", true);
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
