package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;

    private ElapsedTime imuTimer = new ElapsedTime();

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
        super.initDriveMotors();
    }

    @Override
    protected void initCollectorLifter()
    {
        //Add setup of collector/lifter motors
    }

    @Override
    protected void initPushers()
    {
        //Add setup of pusher
    }

    @Override
    protected void initSensors()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMU_IMUCalibration.json";
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = (BNO055IMU) com.getHardwareMap().get("imu");
        imu.initialize(parameters);
    }

    @Override
    public boolean calibrateGyro()
    {
        //Callibration performed internally and assisted by loading file from offline calib.
        return true;
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
