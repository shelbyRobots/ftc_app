package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

import java.util.ArrayList;
import java.util.List;

public class TilerunnerGtoBot extends ShelbyImuBot
{
    public  Servo    gpitch     = null;
            Servo    gripper    = null;
    private Servo    rgripper   = null;
            Servo    jflicker   = null;

    public  Servo    elevServo  = null;

    public  Servo    relClamp   = null;
    public  CRServo  relExtend  = null;
    public  DcMotor  relPitch   = null;

    private DigitalChannel elevIndexSensor = null;

    public List<Integer> liftPositions = new ArrayList<>(4);

    public static double JFLICKER_UP_POS = 0.1;
    public static double JFLICKER_DOWN_POS = 0.6;
    public static double JFLICKER_STOW_POS = 0.0;

    public static double GRIPPER_CLOSE_POS   = 0.90;
    public static double GRIPPER_PARTIAL_POS = 0.75;
    public static double GRIPPER_OPEN_POS    = 0.5;
           static double GRIPPER_STOW_POS    = 0.3;

    private static double RGRIPPER_CLOSE_POS   = 0.83;
    private static double RGRIPPER_PARTIAL_POS = 0.75;
    private static double RGRIPPER_OPEN_POS    = 0.5;
    private static double RGRIPPER_STOW_POS    = 0.25;

    public static double GPITCH_UP_POS    = 0.16;
    public static double GPITCH_DOWN_POS  = 0.58;
    public static double GPITCH_CLEAR_POS = 0.3;
           static double GPITCH_MIN       = 0.16;
           static double GPITCH_MAX       = 0.9;

           static int    LIFT_AUTON_POS   = 0;
           static int    LIFT_ZERO_POS    = 0;
           static int    LIFT_DROP_POS    = 0;
           static int    LIFT_TIER2_POS   = 0;

           int    ELEV_COUNTS_PER_MOTOR_REV = 4;
           double ELEV_GEAR_ONE = 72;
           double ELEV_WHEEL_DIAM = 1.5;
           double ELEV_CPR = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE;
           double LIFT_SCALE = 1.0;
    public double ELEV_CPI = ELEV_CPR/(Math.PI * ELEV_WHEEL_DIAM)/LIFT_SCALE;
    private int MIN_ELEV_CNT = 0;
    @SuppressWarnings("FieldCanBeLocal")
    private int MAX_ELEV_CNT = 0;

    private int IDX_ELEV_CNT = 0;

    public  int MICRO_MIN = 0;
    @SuppressWarnings("FieldCanBeLocal")
    private int MICRO_MAX = 0;
    public  int MICRO_RNG = 1;

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
        initArm();
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

        boolean useElevServo = false;
        boolean useExtendedRange = true;

        try
        {
            rgripper = hwMap.servo.get("rgripper");
        }
        catch (Exception e)
        {
            RobotLog.ww(TAG, "WARNING initCollectorLifter - no rgripper");
        }

        try  //Collector
        {
            gripper = hwMap.servo.get("gripper");

            //noinspection ConstantConditions
            if (useExtendedRange &&
                        gripper.getController() instanceof ServoControllerEx)
            {
                // Set the rotation servo for extended PWM range
                ServoControllerEx srvCntrlrEx =
                        (ServoControllerEx) gripper.getController();
                int lPort = gripper.getPortNumber();
                int rPort = rgripper.getPortNumber();
                PwmControl.PwmRange range =
                        new PwmControl.PwmRange(500, 2500);
                srvCntrlrEx.setServoPwmRange(lPort, range);
                srvCntrlrEx.setServoPwmRange(rPort, range);
            }

            elevMotor = hwMap.dcMotor.get("elevmotor");
            elevMotor.setDirection(DcMotor.Direction.REVERSE);
            elevMotor.setPower(0.0);
            //elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(name.equals("GTO1"))
            {
                GRIPPER_CLOSE_POS    = 0.92 ;
                GRIPPER_PARTIAL_POS  = 0.78;
                GRIPPER_OPEN_POS     = 0.72;
                GRIPPER_STOW_POS     = 0.56;

                RGRIPPER_CLOSE_POS   = 0.05;
                RGRIPPER_PARTIAL_POS = 0.16;
                RGRIPPER_OPEN_POS    = 0.22;
                RGRIPPER_STOW_POS    = 0.4;

                MIN_ELEV_CNT = (int)(-3.5 * ELEV_CPI);
                MAX_ELEV_CNT = MIN_ELEV_CNT + (int)(19.5 * ELEV_CPI);
            }
            else if(name.equals("GTO2"))
            {
                //gpitch = hwMap.servo.get("gpitch");

                elevMotor.setDirection(DcMotor.Direction.FORWARD);

                GRIPPER_CLOSE_POS   = 0.60;
                GRIPPER_OPEN_POS    = 0.38;
                GRIPPER_PARTIAL_POS = 0.52;
                GRIPPER_STOW_POS    = 0.28 ;

                RGRIPPER_CLOSE_POS   = 0.42;
                RGRIPPER_PARTIAL_POS = 0.48;
                RGRIPPER_OPEN_POS    = 0.64;
                RGRIPPER_STOW_POS    = 0.74;

//                GPITCH_DOWN_POS = 0.58;
//                GPITCH_UP_POS = 0.08;
//                GPITCH_CLEAR_POS = 0.25;
//                GPITCH_MIN = 0.45;
//                GPITCH_MAX = 0.65;

                MIN_ELEV_CNT = (int)(-3.5 * ELEV_CPI);
                MAX_ELEV_CNT = MIN_ELEV_CNT + (int)(19 * ELEV_CPI);
            }

            //noinspection ConstantConditions
            if(useElevServo && name.equals("GTO1"))
            {
                elevServo = hwMap.servo.get("elevservo");

                double MAX_DEG   = 2826;
                MICRO_MIN = 600;
                MICRO_MAX = 2400;
                MICRO_RNG = MICRO_MAX - MICRO_MIN;
                double DEG_P_MICRO = (MAX_DEG/MICRO_RNG);
                ELEV_COUNTS_PER_MOTOR_REV = (int)(360.0/DEG_P_MICRO);
                ELEV_GEAR_ONE = 1;
                ELEV_CPR = ELEV_COUNTS_PER_MOTOR_REV * ELEV_GEAR_ONE;
                ELEV_WHEEL_DIAM = 1.456; //2.35
                LIFT_SCALE = 1.0;
                ELEV_CPI = ELEV_CPR/(Math.PI * ELEV_WHEEL_DIAM)/LIFT_SCALE;
                MIN_ELEV_CNT = 750; //950;
                MAX_ELEV_CNT = 1775; //1925;
                RobotLog.dd(TAG, "ELEV_CPI=%.2f MIN_ELEV_CNT=%d MAX_ELEV_CNT=%d",
                        ELEV_CPI, MIN_ELEV_CNT, MAX_ELEV_CNT);
            }

            capMap.put("collector", true);

            LIFT_AUTON_POS = MIN_ELEV_CNT + (int)(5.5 * ELEV_CPI) - MICRO_MIN;
            LIFT_ZERO_POS  = MIN_ELEV_CNT + (int)(0.5 * ELEV_CPI) - MICRO_MIN;
            LIFT_DROP_POS  = MIN_ELEV_CNT + (int)(1.5 * ELEV_CPI) - MICRO_MIN;
            LIFT_TIER2_POS = MIN_ELEV_CNT + (int)(7.0 * ELEV_CPI) - MICRO_MIN;

            liftPositions.add(MIN_ELEV_CNT + (int)( 0.0 * ELEV_CPI) - MICRO_MIN);
            liftPositions.add(MIN_ELEV_CNT + (int)( 6.0 * ELEV_CPI) - MICRO_MIN);
            liftPositions.add(MIN_ELEV_CNT + (int)(12.0 * ELEV_CPI) - MICRO_MIN);
            liftPositions.add(MIN_ELEV_CNT + (int)(18.0 * ELEV_CPI) - MICRO_MIN);

            RobotLog.dd(TAG, "Gripper close %.2f open %.2f part %.2f",
                    GRIPPER_CLOSE_POS, GRIPPER_OPEN_POS, GRIPPER_PARTIAL_POS);
            RobotLog.dd(TAG, "Gpitch up %.2f down %.2f min %.2f max %.2f",
                    GPITCH_UP_POS, GPITCH_DOWN_POS, GPITCH_MIN, GPITCH_MAX);
            if(elevMotor != null)
                RobotLog.dd(TAG, "Elev %s", elevMotor.getDirection());
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map in initCollectorLifter\n" + e.toString());
        }

        try
        {
            elevIndexSensor = hwMap.get(DigitalChannel.class, "elevTouch");
            elevIndexSensor.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception e)
        {
            RobotLog.ww(TAG, "WARNING initCollectorLifter - no elevTouch");
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

    @SuppressWarnings("WeakerAccess")
    protected void initArm()
    {
        RobotLog.dd(TAG, "GTO initArm");
        try
        {
            relClamp  = hwMap.servo.get("relClamp");
            relExtend = hwMap.crservo.get("relExtend");
            relPitch  = hwMap.dcMotor.get("relPitch");
            int RELP_COUNTS_PER_MOTOR_REV = 28;
            int RELP_GEAR_ONE = 40;
            double RELP_GEAR_TWO = 40/120;
            double RELP_CPR = RELP_COUNTS_PER_MOTOR_REV * RELP_GEAR_ONE * RELP_GEAR_TWO;
            double RELP_ARM_LENGTH = 16.0;
            @SuppressWarnings("unused")
            double RELP_CPI = RELP_CPR/(Math.PI * RELP_ARM_LENGTH * 2);
            capMap.put("arm", true);
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map in initArm\n" + e.toString());
        }
    }

    public void setElevAuton()
    {
        RobotLog.dd(TAG, "setElevAuton %d", LIFT_AUTON_POS);
        if(elevMotor != null)
        {
            elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevMotor.setTargetPosition(LIFT_AUTON_POS);
            elevMotor.setPower(0.4);
        }
        else if(elevServo != null)
        {
            double sPos = LIFT_AUTON_POS * 1.0/ MICRO_RNG;
            RobotLog.dd(TAG, "setElevAuton %.2f", sPos);
            elevServo.setPosition(sPos);
        }
    }

    public void setElevDrop()
    {
        RobotLog.dd(TAG, "setElevDrop %d", LIFT_DROP_POS);
        if(elevMotor != null)
        {
            elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevMotor.setTargetPosition(LIFT_DROP_POS);
            elevMotor.setPower(0.4);
        }
        else if(elevServo != null)
        {
            double sPos = LIFT_DROP_POS * 1.0/ MICRO_RNG;
            RobotLog.dd(TAG, "setElevDrop %.2f", sPos);
            elevServo.setPosition(sPos);
        }
    }

    public void setElevTier2()
    {
        RobotLog.dd(TAG, "setElevTier2 %d", LIFT_TIER2_POS);
        if(elevMotor != null)
        {
            elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevMotor.setTargetPosition(LIFT_TIER2_POS);
            elevMotor.setPower(0.4);
        }
        else if(elevServo != null)
        {
            double sPos = LIFT_TIER2_POS * 1.0 / MICRO_RNG;
            RobotLog.dd(TAG, "setElevTier2 %.2f", sPos);
            elevServo.setPosition(sPos);
        }
    }

    public void setElevZero()
    {
        RobotLog.dd(TAG, "setElevZero %d", LIFT_ZERO_POS);
        if(elevMotor != null)
        {
            elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevMotor.setTargetPosition(LIFT_ZERO_POS);
            elevMotor.setPower(0.5);
        }
        else if(elevServo != null)
        {
            double sPos = LIFT_ZERO_POS * 1.0 / MICRO_RNG;
            RobotLog.dd(TAG, "setElevZero %.2f", sPos);
            elevServo.setPosition(sPos);
        }
    }

    public void initElevZero()
    {
        if(elevMotor != null)
        {
            elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevMotor.setPower(0.0);
        }
    }

    private void setElevPositions()
    {
        //from the top of the tilerunner side rail (~4.25 in from ground)
        double zoff = -3.5;
        double aoff =  1.0;
        double doff = -2.0;
        double toff =  2.5;
        LIFT_AUTON_POS = IDX_ELEV_CNT + (int)( aoff * ELEV_CPI) - MICRO_MIN;
        LIFT_ZERO_POS  = IDX_ELEV_CNT + (int)( zoff * ELEV_CPI) - MICRO_MIN;
        LIFT_DROP_POS  = IDX_ELEV_CNT + (int)( doff * ELEV_CPI) - MICRO_MIN;
        LIFT_TIER2_POS = IDX_ELEV_CNT + (int)( toff * ELEV_CPI) - MICRO_MIN;

        liftPositions.set(0, IDX_ELEV_CNT + (int)( zoff +  0.0 * ELEV_CPI) - MICRO_MIN);
        liftPositions.set(1, IDX_ELEV_CNT + (int)( zoff +  6.0 * ELEV_CPI) - MICRO_MIN);
        liftPositions.set(2, IDX_ELEV_CNT + (int)( zoff + 12.0 * ELEV_CPI) - MICRO_MIN);
        liftPositions.set(3, IDX_ELEV_CNT + (int)( zoff + 18.0 * ELEV_CPI) - MICRO_MIN);
    }

    public void indexElev()
    {
        if(elevServo == null || elevIndexSensor == null) return;
        setElevAuton();
        op.sleep(1500);
        stowGripper();
        for(int micsec = LIFT_AUTON_POS; micsec > MIN_ELEV_CNT; micsec--)
        {
            elevServo.setPosition(micsec/MICRO_RNG);
            if(isElevTouchPressed())
            {
                IDX_ELEV_CNT = micsec;
                setElevPositions();
                break;
            }
            op.sleep(5);
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
        if(gripper != null)   gripper.setPosition(GRIPPER_CLOSE_POS);
        if(rgripper != null) rgripper.setPosition(RGRIPPER_CLOSE_POS);
    }
    public void openGripper()
    {
        RobotLog.dd(TAG, "openGripper %.2f %.2f", GRIPPER_OPEN_POS, RGRIPPER_OPEN_POS);
        if(gripper != null)   gripper.setPosition(GRIPPER_OPEN_POS);
        if(rgripper != null) rgripper.setPosition(RGRIPPER_OPEN_POS);
    }
    public void stowGripper()
    {
        RobotLog.dd(TAG, "stowGripper %.2f %.2f", GRIPPER_STOW_POS, RGRIPPER_STOW_POS);
        if(gripper != null)   gripper.setPosition(GRIPPER_STOW_POS);
        if(rgripper != null) rgripper.setPosition(RGRIPPER_STOW_POS);
    }
    public void partialGripper()
    {
        RobotLog.dd(TAG, "partialGripper %.2f %.2f", GRIPPER_PARTIAL_POS, RGRIPPER_PARTIAL_POS);
        if(gripper != null)   gripper.setPosition(GRIPPER_PARTIAL_POS);
        if(rgripper != null) rgripper.setPosition(RGRIPPER_PARTIAL_POS);
    }
    public void retractGpitch()
    {
        if(gpitch != null) gpitch.setPosition(GPITCH_UP_POS);
    }
    public void deployGpitch()
    {
        if(gpitch != null) gpitch.setPosition(GPITCH_DOWN_POS);
    }
    public void downGpitch()
    {
        if(gpitch != null) gpitch.setPosition(GPITCH_MIN);
    }
    public void clearGpitch()
    {
        if(gpitch != null) gpitch.setPosition(GPITCH_CLEAR_POS);
    }

    public double getGpitchMin() {return GPITCH_MIN;}
    public double getGpitchMax() {return GPITCH_MAX;}
    public double getGpitchUpPos() {return GPITCH_UP_POS;}
    public double getGpitchDownPos() {return GPITCH_DOWN_POS;}

    public int getMinElev() {return (int)(MIN_ELEV_CNT + 0.5 * ELEV_CPI);}
    public int getMaxElev() {return (int)(MAX_ELEV_CNT);}

    public boolean isElevTouchPressed()
    {
        boolean touch = false;
        if(elevIndexSensor != null)
            touch = !elevIndexSensor.getState();
        return touch;
    }
}
