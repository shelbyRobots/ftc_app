package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Units;

public class RoRuBot extends TilerunnerGtoBot
{
    public RoRuBot()
    {
        super();

        CAMERA_X_IN_BOT = 0.0f * (float) Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
    }

    @Override
    public void init(LinearOpMode op, boolean initDirSensor)
    {
        computeCPI();

        initOp(op);
        initDriveMotors();
        initCollectorLifter();
        initPushers();
        initSensors(initDirSensor);
        initArm();
        initHolder();
        initCapabilities();
    }

    @Override
    public void initCollectorLifter()
    {
    }

    @Override
    public void initPushers()
    {
    }

    @Override
    public void initArm()
    {
    }

    @Override
    public void initHolder()
    {
    }

    @Override
    public void initSensors()
    {
        super.initSensors();
    }
}
