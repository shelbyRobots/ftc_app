package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Points;
import org.firstinspires.ftc.teamcode.field.RrPoints;
import org.firstinspires.ftc.teamcode.image.BeaconDetector;
import org.firstinspires.ftc.teamcode.image.BeaconFinder;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Locale;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;

@SuppressWarnings({"unused", "ForLoopReplaceableByForEach"})
@Autonomous(name="RrAutoShelby", group="Auton")
//@Disabled
public class RrAutoShelby extends InitLinearOpMode implements FtcMenu.MenuButtons
{
    public RrAutoShelby()
    {
        super();
    }

    private void startMode()
    {
        dashboard.clearDisplay();
        drvTrn.start();
        do_main_loop();
    }

    @Override
    public void runOpMode()
    {
        setup();
        waitForStart();
        startMode();
        stopMode();
    }

    public void runPeriodic(double elapsedTime)
    {
    }

    private void stopMode()
    {
        if(drvTrn != null) drvTrn.cleanup();
        bd.cleanupCamera();
    }

    private void setup()
    {
        telemetry.addData("_","PLEASE WAIT - STARTING");
        telemetry.update();

        initCommon(this, true, true, false, true);

        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        RobotLog.ii("SJH", "SETUP");

        robot.init(this);

        drvTrn.init(robot);
        drvTrn.setUseSpeedThreads(false);
        drvTrn.setRampUp(false);

        bd = new BeaconDetector();
        bf = (BeaconFinder) bd;

        doMenus();
        setupLogger();

        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        RobotLog.ii("SJH", "STARTPOS %s", startPos);
        RobotLog.ii("SJH", "ALLIANCE %s", alliance);
        RobotLog.ii("SJH", "DELAY    %4.2f", delay);

        Points pts = new RrPoints(startPos, alliance);
        pathSegs = pts.getSegments();

        initHdg = (int)(Math.round(pathSegs[0].getFieldHeading()));

        ShelbyBot.DriveDir startDdir = pathSegs[0].getDir();
        robot.setDriveDir(startDdir);

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");

        if (robot.leftMotor  != null &&
            robot.rightMotor != null &&
            robot.gyro       != null)
        {
            gyroReady = robot.calibrateGyro();
        }

        if(gyroReady)
            dashboard.displayPrintf(0, "GYRO CALIBATED!!");

        if(alliance == Field.Alliance.BLUE) drvTrn.setLFirst(false);

        RobotLog.ii("SJH", "ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs[0].getStrtPt();
        drvTrn.setCurrPt(currPoint);

        if(startPos == Field.StartPos.START_R_PUSHER && robot.gyroReady)
        {
            dashboard.displayPrintf(1, "PERFORM START POS MOVE NOW TO HDG %d", initHdg);
            robot.gyro.resetZAxisIntegrator();

            ElapsedTime botRotTimer = new ElapsedTime();
            while (botRotTimer.seconds() < 20)
            {
                int chdg = robot.gyro.getIntegratedZValue();
                dashboard.displayPrintf(0, "GHDG: %d", chdg);
                RobotLog.ii("SJH", "RMOVE CHDG %d", chdg);
                sleep(10);
            }

            dashboard.displayPrintf(1, "NO MORE MOVEMENT");
        }

        drvTrn.setStartHdg(initHdg);
        robot.setInitHdg(initHdg);

        RobotLog.ii("SJH", "Start %s.", currPoint);
        dashboard.displayPrintf(3, "PATH: Start at %s", currPoint);

        RobotLog.ii("SJH", "IHDG %4d", initHdg);

        bd.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        RobotLog.ii("SJH", "STARTING AT %4.2f", timer.seconds());
        if(logData)
        {
            Point2d spt = pathSegs[0].getStrtPt();
            dl.addField("START");
            dl.addField(initHdg);
            dl.addField(spt.getX());
            dl.addField(spt.getY());
            dl.newLine();
        }

        RobotLog.ii("SJH", "Delaying for %4.2f seconds", delay);
        ElapsedTime delayTimer = new ElapsedTime();
        while (opModeIsActive() && delayTimer.seconds() < delay)
        {
            idle();
        }

        RobotLog.ii("SJH", "Done delay");

        RobotLog.ii("SJH", "START CHDG %d", robot.gyro.getIntegratedZValue());

        robot.gyro.resetZAxisIntegrator();

        boolean SkipNextSegment = false;
        for (int i = 0; i < pathSegs.length; ++i)
        {
            if(!opModeIsActive() || isStopRequested()) break;

            String segName = pathSegs[i].getName();
            RobotLog.ii("SJH", "Starting segment %s at %4.2f", segName,
                    startTimer.seconds());

            //noinspection ConstantConditions
            if (SkipNextSegment)
            {
                SkipNextSegment = false;
                RobotLog.ii("SJH", "Skipping segment %s", pathSegs[i].getName());
                if(i < pathSegs.length - 1)
                {
                    RobotLog.ii("SJH", "Setting segment %s start pt to %s",
                            pathSegs[i+1].getName(),
                            pathSegs[i].getStrtPt());
                    pathSegs[i+1].setStrtPt(pathSegs[i].getStrtPt());
                }
                continue;
            }

            Segment curSeg;

            if(curPos == null || !useImageLoc)
            {
                curSeg = pathSegs[i];
            }
            else
            {
                drvTrn.setCurrPt(curPos);
                curSeg = new Segment("CURSEG", curPos, pathSegs[i].getTgtPt());
            }
            curPos = null;

            if(curSeg.getStrtPt().getX() == curSeg.getTgtPt().getX() &&
               curSeg.getStrtPt().getY() == curSeg.getTgtPt().getY())
            {
                continue;
            }

            robot.setDriveDir(curSeg.getDir());

            drvTrn.setInitValues();
            String segLogStr = String.format(Locale.US, "%s - %s H: %4.1f",
                    curSeg.getStrtPt().toString(),
                    curSeg.getTgtPt().toString(),
                    curSeg.getFieldHeading());
            drvTrn.logData(true, segName + " " + segLogStr);

            RobotLog.ii("SJH", "ENCODER TURN %s", curSeg.getName());
            doEncoderTurn(curSeg.getFieldHeading(), segName + " encoderTurn"); //quick but rough

//            if (curSeg.getAction() == Segment.Action.SHOOT)
//            {
//                robot.shotmotor1.setPower(DEF_SHT_PWR);
//                robot.shotmotor2.setPower(DEF_SHT_PWR);
//                //robot.sweepMotor.setPower(-DEF_SWP_PWR * 0.1);
//            }
            doMove(curSeg);

            Double pturn = curSeg.getPostTurn();
            if(usePostTurn && pturn != null)
            {
                RobotLog.ii("SJH", "ENCODER POST TURN %s", curSeg.getName());
                doEncoderTurn(pturn, segName + " postEncoderTurn");

//                RobotLog.ii("SJH", "GRYO POST TURN %s", curSeg.getName());
//                doGyroTurn(pturn, segName + " postGyroTurn");
            }

            if(!opModeIsActive() || isStopRequested())
            {
                drvTrn.stopMotion();
                break;
            }

            RobotLog.ii("SJH", "Planned pos: %s %s",
                    pathSegs[i].getTgtPt(),
                    pathSegs[i].getFieldHeading());


            Segment.Action act = curSeg.getAction();

            if(act != Segment.Action.NOTHING)
            {
                drvTrn.setInitValues();
                drvTrn.logData(true, segName + " action " + act.toString());
            }

            switch (act)
            {
//                case SHOOT:
//                    do_shoot();
//                    drvTrn.driveToTarget(0.13, 18);
//                    break;
//
//                case FIND_BEACON:
//                    //do_findAndPushBeacon(curSeg.getTgtPt());
//                    do_findAndPushBeacon(true, curSeg);
//
//                    if(robot.dim != null)
//                    {
//                        robot.dim.setLED(0, false); robot.dim.setLED(1, false);
//                        robot.dim.setLED(0, false); robot.dim.setLED(0, false);
//                    }
//
//                    break;
//
//                case RST_PUSHER:
//                    robot.lpusher.setPosition(L_DN_PUSH_POS);
//                    break;

                case NOTHING:
                    break;
            }
        }
    }

    private void doMove(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested()) return;

        drvTrn.setInitValues();
        RobotLog.ii("SJH", "Setting drive tuner to %4.2f", seg.getDrvTuner());
        drvTrn.logData(true, seg.getName() + " move");
        drvTrn.setDrvTuner(seg.getDrvTuner());

        drvTrn.setBusyAnd(true);
        String  snm = seg.getName();
        Point2d spt = seg.getStrtPt();
        Point2d ept = seg.getTgtPt();
        double  fhd = seg.getFieldHeading();
        ShelbyBot.DriveDir dir = seg.getDir();
        double speed = seg.getSpeed();
        double fudge = seg.getDrvTuner();
        Segment.TargetType ttype = seg.getTgtType();

        RobotLog.ii("SJH", "Drive %s %s %s %6.2f %3.2f %s tune: %4.2f %s",
                snm, spt, ept, fhd, speed, dir, fudge, ttype);

        dashboard.displayPrintf(2, "STATE: %s %s %s - %s %6.2f %3.2f %s",
                "DRIVE", snm, spt, ept, fhd, speed, dir);

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;

        timer.reset();

        boolean doCorrect = true;
        boolean singleSeg = true;
        if(robot.colorSensor != null && seg.getTgtType() == Segment.TargetType.COLOR)
        {
            colSegNum++;
            int colGyroOffset;
            if(alliance == Field.Alliance.RED)
            {
                if(colSegNum == 1)
                {
                    colGyroOffset = 60;
                }
                else
                {
                    colGyroOffset = 80;
                }
            }
            else
            {
                if(colSegNum == 1)
                {
                    colGyroOffset = 120;
                }
                else
                {
                    colGyroOffset = 120;
                }
            }

            drvTrn.setColGyroOffset(colGyroOffset);
            drvTrn.setInitValues();
            double pct = 0.90;
            double fullSegLen = seg.getLength();
            int colSegLbeg = drvTrn.curLpos;
            int colSegRbeg = drvTrn.curRpos;
            int colSegLend = colSegLbeg + drvTrn.distanceToCounts(fullSegLen);
            int colSegRend = colSegRbeg + drvTrn.distanceToCounts(fullSegLen);

            int linLpos = colSegLend;
            int linRpos = colSegRend;

            Point2d cept = new Point2d(pct * (ept.getX() - spt.getX()) + spt.getX(),
                                       pct * (ept.getY() - spt.getY()) + spt.getY());

            int targetHdg = (int) Math.round(fhd);

            double colDist = cept.distance(ept);
            double ovrDist = 2.0;
            int colCnts = drvTrn.distanceToCounts(colDist);
            int ovrCnts = drvTrn.distanceToCounts(ovrDist);

            //noinspection ConstantConditions
            if(singleSeg)
            {
                drvTrn.driveDistanceLinear(fullSegLen, speed, ddir, targetHdg, true);
            }
            else
            {
                drvTrn.driveToPointLinear(cept, speed, ddir, targetHdg);
                //drvTrn.driveToTarget(0.2, 20);
                robot.turnColorOn();

                sleep(10);

                double colSpd = 0.10;
                RobotLog.ii("SJH", "Color Driving to pt %s at speed %4.2f", ept, colSpd);
                String colDistStr = String.format(Locale.US, "%4.2f %s",
                        colDist,
                        cept.toString());
                drvTrn.logData(true, "FIND_LINE CDIST: " + colDistStr);

                drvTrn.driveDistance(colDist+ovrDist, colSpd, Drivetrain.Direction.FORWARD);

                while(opModeIsActive() && !isStopRequested())
                {
                    drvTrn.setCurValues();
                    drvTrn.logData();

                    int lTrav = Math.abs(drvTrn.curLpos  - drvTrn.initLpos);
                    int rTrav = Math.abs(drvTrn.curRpos  - drvTrn.initRpos);

                    int totColor = drvTrn.curRed + drvTrn.curGrn + drvTrn.curBlu;

                    if (totColor > COLOR_THRESH)
                    {
                        linLpos = drvTrn.curLpos;
                        linRpos = drvTrn.curRpos;
                        colGyroOffset = 50;
                        if(snm.equals("BECN2"))
                        {
                            linLpos -= colGyroOffset;
                            linRpos -= colGyroOffset;
                        }
                        drvTrn.stopMotion();
                        drvTrn.setEndValues("COLOR_FIND " + linLpos + " " + linRpos);
                        RobotLog.ii("SJH", "FOUND LINE");
                        drvTrn.trgtLpos = linLpos;
                        drvTrn.trgtRpos = linRpos;
                        drvTrn.logOverrun(0.1);
                        break;
                    }

                    if(lTrav > (colCnts + ovrCnts) ||
                       rTrav > (colCnts + ovrCnts))
                    {
                        drvTrn.stopMotion();
                        drvTrn.setEndValues("COLOR_MISS - go to" + linLpos + " " + linRpos);
                        RobotLog.ii("SJH", "REACHED OVERRUN PT - Backing up a bit");
                        drvTrn.trgtLpos = linLpos;
                        drvTrn.trgtRpos = linRpos;
                        drvTrn.logOverrun(0.1);
                        break;
                    }

                    drvTrn.makeGyroCorrections(colSpd, targetHdg, Drivetrain.Direction.FORWARD);

                    drvTrn.frame++;
                    robot.waitForTick(10);
                }
            }

            robot.turnColorOff();
        }
        else
        {
            int targetHdg = (int) Math.round(seg.getFieldHeading());
            drvTrn.driveToPointLinear(ept, speed, ddir, targetHdg);
        }

        //If segment action is shoot, force bot to turn slightly to try to get balls to fall away from beacons

        int dtlCorrect = 18;

//        if(snm.equals("PRECTR") ||
//           snm.equals("CTRPRK") ||
//           snm.equals("B_MID")  ||
//           snm.equals("DP1")    ||
//           snm.equals("ASHOOT") ||
//           snm.equals("BSHOOT") ||
//           snm.equals("DFNPRK"))
//        {
//            doCorrect = false;
//        }
        if(doCorrect) drvTrn.driveToTarget(0.13, dtlCorrect);

        drvTrn.setCurrPt(ept);

        RobotLog.ii("SJH", "Completed move %s. Time: %6.3f HDG: %4d",
                seg.getName(), timer.time(), robot.getGyroFhdg());
    }


    private void doEncoderTurn(double fHdg, int thresh, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        drvTrn.setBusyAnd(true);
        drvTrn.setInitValues();
        drvTrn.logData(true, prefix);
        int cHdg = drvTrn.curHdg;
        int tHdg = (int) Math.round(fHdg);
        double angle = tHdg - cHdg;
        RobotLog.ii("SJH", "doEncoderTurn CHDG %4d THDG %4d", cHdg, tHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 2.0) return;

        RobotLog.ii("SJH", "Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle, DEF_ENCTRN_PWR, thresh);
        cHdg = robot.getGyroFhdg();
        RobotLog.ii("SJH", "Completed turn %5.2f. Time: %6.3f CHDG: %4d",
                angle, timer.time(), cHdg);
    }

    private void doEncoderTurn(double fHdg, String prefix)
    {
        doEncoderTurn(fHdg, Drivetrain.TURN_BUSYTHRESH, prefix);
    }

    private void doGyroTurn(double fHdg, String prefix)
    {
        if(!gyroReady) return;
        if(!opModeIsActive() || isStopRequested()) return;

        drvTrn.setInitValues();
        drvTrn.logData(true, prefix);
        int cHdg = drvTrn.curHdg;
        int tHdg = (int) Math.round(fHdg);

        RobotLog.ii("SJH", "doGyroTurn CHDG %4d THDG %4d", cHdg, tHdg);

        if(Math.abs(tHdg-cHdg) < 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(tHdg, DEF_GYRTRN_PWR);

        cHdg = drvTrn.curHdg;
        RobotLog.ii("SJH", "Completed turnGyro %4d. Time: %6.3f CHDG: %4d",
                tHdg, timer.time(), cHdg);
    }

    //TODO:  ADD calls to ImageTracker - with gettin key to determine glyph box
    //       - ImageTracker should have base IF for getting pos/hdg, full img
    //       -    and cropped image (based on input paremeters with offsets)
    //       -    ImageTracker sub-class can have challenge specific - getKey
    //       -    or better - getResult - with result being enum from an IF.
    //TODO:  ADD Glyph placer
    //TODO:  ADD Glyph getter

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up;} //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    } //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    } //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }  //isMenuBackButton

    private void doMenus()
    {
        FtcChoiceMenu<Field.StartPos> startPosMenu =
                new FtcChoiceMenu<>("START:", null, this);
        FtcChoiceMenu<Field.Alliance> allianceMenu =
                new FtcChoiceMenu<>("ALLIANCE:", startPosMenu, this);
        FtcValueMenu delayMenu     = new FtcValueMenu("DELAY:", allianceMenu, this,
                0.0, 20.0, 1.0, 0.0, "%5.2f");

        startPosMenu.addChoice("Start_A", Field.StartPos.START_A_SWEEPER, startPosMenu);
        startPosMenu.addChoice("Start_B", Field.StartPos.START_B_SWEEPER, startPosMenu);
        startPosMenu.addChoice("Start_R", Field.StartPos.START_R_PUSHER,  startPosMenu);

        allianceMenu.addChoice("RED",  Field.Alliance.RED,  delayMenu);
        allianceMenu.addChoice("BLUE", Field.Alliance.BLUE, delayMenu);

        FtcMenu.walkMenuTree(startPosMenu, this);

        startPos = startPosMenu.getCurrentChoiceObject();
        alliance = allianceMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();

        int lnum = 3;
        dashboard.displayPrintf(lnum++, "START: %s", startPos);
        dashboard.displayPrintf(lnum++, "ALLIANCE: %s", alliance);
    }

    private void setupLogger()
    {
        if (logData)
        {
            dl.addField("NOTE");
            dl.addField("FRAME");
            dl.addField("Gyro");
            dl.addField("LENC");
            dl.addField("RENC");
            dl.addField("LPWR");
            dl.addField("RPWR");
            dl.addField("RED");
            dl.addField("GRN");
            dl.addField("BLU");
            dl.addField("ESTX");
            dl.addField("ESTY");
            dl.addField("ESTH");
            dl.newLine();
        }
    }

    private enum Team
    {
        SONIC,
        SNOWMAN
    }

    private final static double DEF_ENCTRN_PWR  = 0.4;
    private final static double DEF_GYRTRN_PWR = 0.4;

    private Segment[] pathSegs;

    private ShelbyBot   robot = new ShelbyBot();
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime startTimer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private BeaconFinder bf;
    private Detector bd;

    private static Point2d curPos;
    private static double  curHdg;
    private int initHdg = 0;
    private boolean gyroReady;
    private boolean usePostTurn = true;

    private static Field.StartPos startPos = Field.StartPos.START_A_SWEEPER;
    private static Field.Alliance alliance = Field.Alliance.RED;
    private static Team team = Team.SNOWMAN;

    private int RED_THRESH = 15;
    private int GRN_THRESH = 15;
    private int BLU_THRESH = 15;
    private int COLOR_THRESH = 20;

    private double delay = 0.0;

    private boolean useImageLoc  = false;

    private boolean firstInState = true;

    private int postSleep = 150;

    private int colSegNum = 0;
}

