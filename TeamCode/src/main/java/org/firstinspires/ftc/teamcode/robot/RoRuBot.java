package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

public class RoRuBot extends TilerunnerGtoBot {
    private final String TAG = "SJH RoRuRobot";

    private DcMotor _liftyBoi = null;
    //5202 series yellow jacket motor w/ 5.2:1 gear box has 145.6 cpr of output shaft
    //  1150 rpm (19.17 rps) no load speed
    //
    //lift is made with 1:1 gear between motor output shaft and lead screw
    //
    //gobilda lead screw has 7.9in range, 2mm pitch, and 4 start
    //lead=2mm pitch * 4 start = 8 mm/lead screw rev
    //19.17rps * 8mm/rev = 153mm/s no load lift speed (6in/s)
    //201mm (7.9in) range in 201/153=1.31 s
    //201mm / 8mm/rev = 25.125rev
    //25.125rev * 145.6 cpr = 3658 counts

    private final double HANGER_CPER = 28; //quad encoder cnts/encoder rev
    private final double HANGER_INT_GEAR = 5.2;
    private final double HANGER_CPOR = HANGER_CPER * HANGER_INT_GEAR; //145.6 cnts/outShaftRev
    private final double HANGER_EXT_GEAR = 1.0;
    private final double HANGER_PITCH = 2.0; //2.0mm/pitch
    private final int HANGER_STARTS = 4; //pitch/rev
    private final double HANGER_LEAD = HANGER_PITCH * HANGER_STARTS; //8 mm/rev
    private final double HANGER_CPMM = HANGER_EXT_GEAR * HANGER_CPOR / HANGER_LEAD;
    private final double HANGER_CPI = HANGER_CPMM * 25.4;
    private final double HANGER_DIST = 7.5;
    private final double RELEASE_DIST = 7.5;
    private final int HANGER_CNTS = (int) (HANGER_CPI * HANGER_DIST);
    private final int RELEASE_CNTS = (int) (HANGER_CPI * RELEASE_DIST);
    @SuppressWarnings("FieldCanBeLocal")
    private final int HANGER_THRESH = (int) (0.25 * HANGER_CPI);

    //With motor forward, +ve speed = lower (retract), -ve speed - raise (extend)

    // Encoder counts to go to up pos from down
    //TOP is with lifter extended (bot down).
    //BOT is with lifter retracted (bot raised).
    @SuppressWarnings("FieldCanBeLocal")
    private final int ENC_COUNTS_HANGER_TOP = -HANGER_CNTS;
    @SuppressWarnings("FieldCanBeLocal")
    private final int ENC_COUNTS_HANGER_RELEASE = -RELEASE_CNTS;
    @SuppressWarnings("FieldCanBeLocal")
    private final int ENC_COUNTS_HANGER_BOT = 0;

    private Servo _markerServo;
    @SuppressWarnings("FieldCanBeLocal")
    private final double _markerStow = 0.5;
    @SuppressWarnings("FieldCanBeLocal")
    private final double _markerDrop = 0.75;
    @SuppressWarnings("FieldCanBeLocal")
    private final double _markerPark = 0.25;

    public RoRuBot() {
        super();

        CAMERA_X_IN_BOT = 0.0f * (float) Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float) Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 0.0f * (float) Units.MM_PER_INCH;
    }

    @Override
    public void init(LinearOpMode op, boolean initDirSensor) {
        computeCPI();

        initOp(op);
        initDriveMotors();
        initCollectorLifter();
        initPushers();
        initSensors(initDirSensor);
        initArm();
        initHolder();
        initCapabilities();
        initMarker();
    }

    @Override
    public void initCollectorLifter() {
    }

    @Override
    public void initPushers() {
    }

    @Override
    public void initArm() {
    }

    @Override
    public void initHolder() // Holder is the hanging mechanism
    {
        try {
            _liftyBoi = hwMap.dcMotor.get("liftyboi");

            _liftyBoi.setDirection(DcMotorSimple.Direction.FORWARD);
            _liftyBoi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _liftyBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            _liftyBoi.setPower(0.0f);
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get hardware map in initHolder\n" + e.toString());
        }

        capMap.put("holder", true);


    }

    @Override
    public void initSensors() {
        super.initSensors();
    }

    private void initMarker() {
        try {
            _markerServo = hwMap.servo.get("marker");
            capMap.put("holder", true);
            _markerServo.setPosition(_markerStow);
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get hardware map in initMarker\n" + e.toString());
        }
    }

    public void moveHolder(double inches)
    {
        int curPos = _liftyBoi.getCurrentPosition();
        int cnts = (int) (-inches*HANGER_CPI);
        setHolderPos(curPos + cnts);
    }

    public void putHolderAtLatch()
    {
        //This puts the holder at latch with bot on ground
        setHolderPos(ENC_COUNTS_HANGER_TOP);
    }

    public void putHolderAtStow()
    {
        //This puts the holder retracted.
        //This is start pos and holds the bot up
        setHolderPos(ENC_COUNTS_HANGER_BOT);
    }

    public void putHolderAtRelease()
    {
        //This puts the holder above the latch - clear
        setHolderPos(ENC_COUNTS_HANGER_RELEASE);
    }

    public void setHolderPos(boolean raiseBot)
    {
        int targetPos;
        if (raiseBot)
        {
            RobotLog.dd(TAG, "Raise bot holder");
            targetPos = ENC_COUNTS_HANGER_BOT;
        }
        else
        {
            RobotLog.dd(TAG, "Lower bot holder");
            targetPos = ENC_COUNTS_HANGER_RELEASE;
        }

        setHolderPos(targetPos);
    }

    @SuppressWarnings("WeakerAccess")
    public void setHolderPos(int targetPos)
    {
        if(_liftyBoi == null)
        {
            RobotLog.ee(TAG, "liftyboi is null - fix it");
            return;
        }

        _liftyBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double holderSpd = 0.5;

        _liftyBoi.setTargetPosition(targetPos);
        _liftyBoi.setPower(holderSpd);

        ElapsedTime liftTimer = new ElapsedTime();
        double liftTimeLimit = 0.0;
        while(op.opModeIsActive() && liftTimer.seconds() < liftTimeLimit)
        {
            RobotLog.dd(TAG, "In lift.  targetpos=" + targetPos + " curPos=" + _liftyBoi.getCurrentPosition());
            if(Math.abs(_liftyBoi.getCurrentPosition() - targetPos) < HANGER_THRESH)
            {
                _liftyBoi.setPower(0.0);
                break;
            }
        }
        _liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void dropMarker()
    {
        if(_markerServo == null) return;
        _markerServo.setPosition(_markerDrop);
        int dropTimeout = 500; //500ms = 0.5s
        op.sleep(dropTimeout);
        _markerServo.setPosition(_markerStow);
    }

    public void parkMarker()
    {
        if(_markerServo != null)
            _markerServo.setPosition(_markerPark);
    }

    public void stowMarker()
    {
        if(_markerServo != null)
            _markerServo.setPosition(_markerStow);
    }
}
