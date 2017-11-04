package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.internal.system.SystemProperties;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Points;
import org.firstinspires.ftc.teamcode.field.RrField;
import org.firstinspires.ftc.teamcode.field.RrPoints;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.MajorColorDetector;
import org.firstinspires.ftc.teamcode.image.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerGtoBot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;


//After starting on balance stone, opmode needs to read vumark to find L, C, R
//and left ball color.
// Then
//  - move fwd/back ~3 in based on ball color and alliance
//  - move to point in front of cryptobox based on L, C, R
//  - turn 90 and move fwd towards cbox
//  - place glyph
//  - turn towards pit
//  - drive to pit
//  - grab glyphs
//  - turn toward triangle tip
//  - drive to triangle tip
//  - turn to align
//  - drive forward toward cbox
//  - place glyphs

@SuppressWarnings({"unused", "ForLoopReplaceableByForEach"})
@Autonomous(name="RrAutoShelby", group="Auton")
//@Disabled
public class RrAutoShelby extends InitLinearOpMode implements FtcMenu.MenuButtons
{
    public RrAutoShelby()
    {
        //super();
    }

    private void startMode()
    {
        dashboard.clearDisplay();
        drvTrn.start();
        do_main_loop();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, true, true, false, true);
        super.runOpMode();

        setup();
        //waitForStart();
        while(!isStarted())
        {
            double shdg = robot.getGyroHdg();
            double fhdg = robot.getGyroFhdg();
            dashboard.displayPrintf(9, "HDG %4.2f FHDG %4.2f", shdg, fhdg);
            dashboard.displayPrintf(10, "GyroReady %s", gyroReady);
            dashboard.displayPrintf(11, "RGyroReady %s", robot.gyroReady);
            dashboard.displayPrintf(12, "LENC %d", robot.leftMotor.getCurrentPosition());
            dashboard.displayPrintf(13, "RENC %d", robot.rightMotor.getCurrentPosition());
            dashboard.displayPrintf(14, "ELEV %d", robot.elevMotor.getCurrentPosition());
            sleep(10);
        }
        startMode();
        stopMode();
    }

    private void stopMode()
    {
        robot.autonEndHdg = robot.getGyroFhdg();
        if(drvTrn != null) drvTrn.cleanup();
        det.cleanupCamera();
    }

    private void setup()
    {
        dashboard.displayPrintf(0, "PLEASE WAIT - STARTING");
        logData = true;

        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        RobotLog.ii("SJH", "SETUP");

        //Since we only have 5 seconds between Auton and Teleop, automatically load
        //teleop opmode

        String teleopName = "Teleop Driver";
        RobotLog.dd(TAG, "Setting up auto tele loader : %s", teleopName);
        AutoTransitioner.transitionOnStop(this, teleopName);

        robot.init(this);

        System.out.println("Robot CPI " + robot.CPI);

        drvTrn.init(robot);
        drvTrn.setUseSpeedThreads(false);
        drvTrn.setRampUp(false);

        robot.jflicker.setPosition(JFLICKER_UP_POS);
        robot.gripper.setPosition(GRIPPER_CLOSE_POS);
        robot.gpitch.setPosition(GPITCH_UP_POS);

        det = new MajorColorDetector();
        tracker = new ImageTracker(VuforiaInitializer.Challenge.RR);

        tracker.setTrackableRelativeCropCorners(RrField.getTrackableRelativeCropCorners());

        doMenus();
        setupLogger();

        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        RobotLog.ii("SJH", "STARTPOS %s", startPos);
        RobotLog.ii("SJH", "ALLIANCE %s", alliance);
        RobotLog.ii("SJH", "DELAY    %4.2f", delay);

        Points pts = new RrPoints(startPos, alliance);
        pathSegs.addAll(Arrays.asList(pts.getSegments()));

        initHdg = pathSegs.get(0).getFieldHeading();

        ShelbyBot.DriveDir startDdir = pathSegs.get(0).getDir();
        robot.setDriveDir(startDdir);

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");

        if (robot.imu != null || robot.gyro  != null)
        {
            gyroReady = robot.calibrateGyro();
        }

        if(gyroReady)
            dashboard.displayPrintf(0, "GYRO CALIBATED!!");

        if(alliance == Field.Alliance.BLUE) drvTrn.setLFirst(false);

        RobotLog.ii("SJH", "ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs.get(0).getStrtPt();
        drvTrn.setCurrPt(currPoint);

        drvTrn.setStartHdg(initHdg);
        robot.setInitHdg(initHdg);

        RobotLog.ii("SJH", "Start %s.", currPoint);
        dashboard.displayPrintf(3, "PATH: Start at %s", currPoint);

        RobotLog.ii("SJH", "IHDG %4.2f", initHdg);

        det.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        RobotLog.ii("SJH", "STARTING AT %4.2f", timer.seconds());
        if(logData)
        {
            Point2d spt = pathSegs.get(0).getStrtPt();
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

        RobotLog.ii("SJH", "START CHDG %6.3f", robot.getGyroHdg());

        robot.resetGyro();

        boolean SkipNextSegment = false;
        for (int i = 0; i < pathSegs.size(); ++i)
        {
            if(!opModeIsActive() || isStopRequested()) break;

            String segName = pathSegs.get(i).getName();
            RobotLog.ii("SJH", "Starting segment %s at %4.2f", segName,
                    startTimer.seconds());

            //noinspection ConstantConditions
            if (SkipNextSegment)
            {
                SkipNextSegment = false;
                RobotLog.ii("SJH", "Skipping segment %s", pathSegs.get(i).getName());
                if(i < pathSegs.size() - 1)
                {
                    RobotLog.ii("SJH", "Setting segment %s start pt to %s",
                            pathSegs.get(i+1).getName(),
                            pathSegs.get(i).getStrtPt());
                    pathSegs.get(i+1).setStrtPt(pathSegs.get(i).getStrtPt());
                }
                continue;
            }

            Segment curSeg;

            if(curPos == null || !useImageLoc)
            {
                curSeg = pathSegs.get(i);
            }
            else
            {
                drvTrn.setCurrPt(curPos);
                curSeg = new Segment("CURSEG", curPos, pathSegs.get(i).getTgtPt());
            }
            curPos = null;

            robot.setDriveDir(curSeg.getDir());

            drvTrn.setInitValues();
            String segLogStr = String.format(Locale.US, "%s - %s H: %4.1f",
                    curSeg.getStrtPt().toString(),
                    curSeg.getTgtPt().toString(),
                    curSeg.getFieldHeading());
            drvTrn.logData(true, segName + " " + segLogStr);

            RobotLog.ii("SJH", "ENCODER TURN %s", curSeg.getName());

//            if (curSeg.getAction() == Segment.Action.SHOOT)
//            {
//                robot.shotmotor1.setPower(DEF_SHT_PWR);
//                robot.shotmotor2.setPower(DEF_SHT_PWR);
//                //robot.sweepMotor.setPower(-DEF_SWP_PWR * 0.1);
//            }
            if(curSeg.getStrtPt().getX() == curSeg.getTgtPt().getX() &&
               curSeg.getStrtPt().getY() == curSeg.getTgtPt().getY())
            {
                //No move needed
            }
            else
            {
                doEncoderTurn(curSeg.getFieldHeading(), segName + " encoderTurn"); //quick but rough
                doGyroTurn(curSeg.getFieldHeading(), segName + " gyroTurn");
                doMove(curSeg);
            }


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
                    pathSegs.get(i).getTgtPt(),
                    pathSegs.get(i).getFieldHeading());


            Segment.Action act = curSeg.getAction();

            if(act != Segment.Action.NOTHING)
            {
                drvTrn.setInitValues();
                drvTrn.logData(true, segName + " action " + act.toString());
            }

            switch (act)
            {
//                case RST_PUSHER:
//                    robot.lpusher.setPosition(L_DN_PUSH_POS);
//                    break;
                case SCAN_IMAGE:
                    doScanPush(pathSegs.get(i+1));
                    break;

                case SET_KEY:
                {
                    Point2d dpt = RrField.getDropPt(Field.Alliance.RED, startPos, key);
                    RobotLog.dd(TAG, "Setting drop point %s %s", key, dpt);
                    pathSegs.get(i+1).setEndPt(dpt);
                    pathSegs.get(i+2).setStrtPt(dpt);
                    break;
                }

                case SET_KEY_OLD:
                    RobotLog.dd(TAG, "In SET_KEY");
                    RobotLog.dd(TAG, "On segment %s", curSeg.toString());
                    if(i+1 < pathSegs.size())
                    {
                        Segment alignSeg = pathSegs.get(i+1);
                        Segment postDropSeg = pathSegs.get(i+2);

                        RobotLog.dd(TAG, "Orig alignSeg %s", alignSeg.toString());

                        Point2d cboxPt = new Point2d("AlnPt",
                                                     alignSeg.getTgtPt().getX(),
                                                     alignSeg.getTgtPt().getY());
                        Point2d dropPt = new Point2d("DropPt",
                                                     cboxPt.getX(),
                                                     cboxPt.getY());
                        double offset = 7.63;
                        double loc = cboxPt.getX();
                        int rbDir = 1;
                        if(alliance == Field.Alliance.BLUE) rbDir = -1;

                        if (startPos == Field.StartPos.START_1)
                        {
                            switch (key)
                            {
                                case LEFT:
                                    loc += rbDir * offset;
                                    break;
                                case RIGHT:
                                    loc -= rbDir * offset;
                                    break;
                            }
                            cboxPt.setX(loc);
                            dropPt.setX(loc);
                            dropPt.setY(-54.0);
                        }

                        if (startPos == Field.StartPos.START_2)
                        {
                            switch (key)
                            {
                                case LEFT:
                                    loc -= rbDir * offset;
                                    break;
                                case RIGHT:
                                    loc += rbDir * offset;
                                    break;
                            }
                            cboxPt.setY(loc);
                            dropPt.setY(loc);
                            dropPt.setX(54.0);
                        }

                        alignSeg.setEndPt(cboxPt);
                        postDropSeg.setStrtPt(dropPt);
                        Segment dropSeg = new Segment("DropSeg", cboxPt, dropPt);
                        dropSeg.setDir(curSeg.getDir());
                        dropSeg.setSpeed(0.2);
                        dropSeg.setAction(Segment.Action.DROP);

                        RobotLog.dd(TAG, "Post alignSeg %s", alignSeg.toString());
                        RobotLog.dd(TAG, "dropSeg  %s", dropSeg.toString());

                        pathSegs.add(i+2, dropSeg);
                    }
                    break;

                case DROP:
                    robot.gpitch.setPosition(GPITCH_DOWN_POS);
                    sleep(500);
                    robot.gripper.setPosition(GRIPPER_OPEN_POS);
                    sleep(500);
                    break;
            }
        }
    }

    private void doScanPush(Segment postPushSeg)
    {
        double jewelPushDist = 2.0;
        double jewelPushSpd  = 0.2;
        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;

        key = getKey();
        jewelColor = getJewelColor();
        RobotLog.dd(TAG, "SCAN_IMAGE KEY = %s jewelColor = %s",
                key, jewelColor);
        if(jewelColor == MajorColorDetector.Color.NONE)
            return;

        robot.jflicker.setPosition(JFLICKER_DOWN_POS);
        sleep(500);

        RobotLog.dd(TAG, "Deploy pusher");
        MajorColorDetector.Color badJewel = MajorColorDetector.Color.BLUE;
        if(alliance == Field.Alliance.BLUE) badJewel = MajorColorDetector.Color.RED;
        int pushSign = 1;
        switch (jewelColor)
        {
            case RED:
                if(alliance == Field.Alliance.RED)
                    ddir = Drivetrain.Direction.REVERSE;
                    pushSign = -1;
                break;

            case BLUE:
                if(alliance == Field.Alliance.BLUE)
                    ddir = Drivetrain.Direction.REVERSE;
                    pushSign = -1;
                break;
        }

        //move to knock off jewel
        RobotLog.dd(TAG, "Moving %s %4.2f at %4.2f to knock off jewel %s",
                ddir, jewelPushDist * pushSign, jewelPushSpd, badJewel);

        Point2d pushStart = postPushSeg.getStrtPt();
        Point2d pushEnd   = new Point2d("PUSHENDPT",
                pushStart.getX() + jewelPushDist * pushSign,
                pushStart.getY());

        ShelbyBot.DriveDir segDir = ShelbyBot.DriveDir.SWEEPER;
        if(ddir == Drivetrain.Direction.REVERSE)
        {
            if(postPushSeg.getDir() == ShelbyBot.DriveDir.SWEEPER)
                segDir = ShelbyBot.DriveDir.PUSHER;
        }
        robot.setDriveDir(segDir);

        boolean curRampDown = drvTrn.getRampDown();
        drvTrn.setRampDown(false);
        Segment pushSeg = new Segment("PushSeg", pushStart, pushEnd);
        pushSeg.setAction(Segment.Action.NOTHING);
        pushSeg.setDir(segDir);
        pushSeg.setSpeed(jewelPushSpd);
        doMove(pushSeg);
        drvTrn.setRampDown(curRampDown);

        robot.jflicker.setPosition(JFLICKER_UP_POS);
        sleep(500);

        RobotLog.dd(TAG, "Retract pusher");

        RobotLog.dd(TAG, "Orig postPushSeg %s", postPushSeg.toString());
        postPushSeg.setStrtPt(pushEnd);
        RobotLog.dd(TAG, "Post postPushSeg %s", postPushSeg.toString());
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

            double targetHdg = fhd;

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
            double targetHdg = seg.getFieldHeading();
            drvTrn.driveToPointLinear(ept, speed, ddir, targetHdg);
        }

        drvTrn.setCurrPt(ept);

        RobotLog.ii("SJH", "Completed move %s. Time: %6.3f HDG: %6.3f",
                seg.getName(), timer.time(), robot.getGyroFhdg());
    }


    private void doEncoderTurn(double fHdg, int thresh, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        drvTrn.setBusyAnd(true);
        drvTrn.setInitValues();
        drvTrn.logData(true, prefix);
        double cHdg = drvTrn.curHdg;
        double tHdg = fHdg;
        double angle = tHdg - cHdg;
        RobotLog.ii("SJH", "doEncoderTurn CHDG %6.3f THDG %6.3f", cHdg, tHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 2.0) return;

        RobotLog.ii("SJH", "Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle, DEF_ENCTRN_PWR, thresh);
        cHdg = robot.getGyroFhdg();
        RobotLog.ii("SJH", "Completed turn %5.2f. Time: %6.3f CHDG: %6.3f",
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
        double cHdg = drvTrn.curHdg;
        double tHdg = fHdg;

        RobotLog.ii("SJH", "doGyroTurn CHDG %4.2f THDG %4.2f", cHdg, tHdg);

        if(Math.abs(tHdg-cHdg) < 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(tHdg, DEF_GYRTRN_PWR);

        cHdg = drvTrn.curHdg;
        RobotLog.ii("SJH", "Completed turnGyro %4.2f. Time: %6.3f CHDG: %4.2f",
                tHdg, timer.time(), cHdg);
    }

    //TODO:  ADD Glyph placer
    //TODO:  ADD Glyph getter

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    private RelicRecoveryVuMark getKey()
    {
        RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;
        tracker.setActive(true);
        RobotLog.dd(TAG, "Finding VuMark first");

        while (opModeIsActive() && key == RelicRecoveryVuMark.UNKNOWN)
        {
            ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            Point2d sensedBotPos;
            double  sensedFldHdg = 0.0;
            int frm = 0;

            tracker.updateRobotLocationInfo();
            sensedBotPos = tracker.getSensedPosition();

            if(sensedBotPos != null)
            {
                sensedFldHdg = tracker.getSensedFldHeading();
                key = tracker.getKeyLoc();
            }
            frm++;

            if ( sensedBotPos != null )
            {
                double t = itimer.seconds();
                RobotLog.ii("SJH", "Senesed Pos: %s %5.2f %2.3f",
                        sensedBotPos, sensedFldHdg, t);
                RobotLog.ii("SJH", "IMG %s frame %d",
                        tracker.getLocString(), frm);
                dashboard.displayPrintf(1, "SLOC: %s %4.1f",
                        sensedBotPos, sensedFldHdg);
                dashboard.displayPrintf(2, "IMG", "%s  frame %d",
                        tracker.getLocString(), frm);
            }

            telemetry.update();
        }

        RobotLog.ii(TAG, "KEY : " + key);
        dashboard.displayPrintf(0, "KEY: " + key);
        RobotLog.dd(TAG, "Turn off image tracker");
        tracker.setActive(false);

        return key;
    }

    private MajorColorDetector.Color getJewelColor()
    {
        RobotLog.dd(TAG, "Set qsize to get frames");
        tracker.setFrameQueueSize(1);
        RobotLog.dd(TAG, "Start LD sensing");
        det.startSensing();

        MajorColorDetector.Color leftJewelColor = MajorColorDetector.Color.NONE;
        while(opModeIsActive() && leftJewelColor == MajorColorDetector.Color.NONE)
        {
            Bitmap rgbImage;
            rgbImage = tracker.getLastCroppedImage();

            if(rgbImage == null) continue;
            det.setBitmap(rgbImage);
            det.logDebug();
            det.logTelemetry();
            if(det instanceof MajorColorDetector)
                leftJewelColor = ((MajorColorDetector) det).getMajorColor();

            if(leftJewelColor == MajorColorDetector.Color.NONE)
                sleep(100);
        }

        det.stopSensing();
        tracker.setFrameQueueSize(0);

        return leftJewelColor;
    }

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
        boolean shortcut = false;
        if(shortcut) return;
        FtcChoiceMenu<Field.StartPos> startPosMenu =
                new FtcChoiceMenu<>("START:", null, this);
        FtcChoiceMenu<Field.Alliance> allianceMenu =
                new FtcChoiceMenu<>("ALLIANCE:", startPosMenu, this);
        FtcValueMenu delayMenu     = new FtcValueMenu("DELAY:", allianceMenu, this,
                0.0, 20.0, 1.0, 0.0, "%5.2f");

        startPosMenu.addChoice("Start_A", Field.StartPos.START_1, allianceMenu);
        startPosMenu.addChoice("Start_B", Field.StartPos.START_2, allianceMenu);

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

    public  final static double JFLICKER_UP_POS = 0.1;
    public  final static double JFLICKER_DOWN_POS = 0.6;

    public  final static double GRIPPER_CLOSE_POS = 0.83;
    public  final static double GRIPPER_PARTIAL_POS = 0.75;
    public  final static double GRIPPER_OPEN_POS = 0.6;

    public  final static double GPITCH_UP_POS = 0.9;
    public  final static double GPITCH_DOWN_POS = 0.4;
    public  final static double GPITCH_MID_POS = 0.7;

    private List<Segment> pathSegs = new ArrayList<>();

    private TilerunnerGtoBot   robot = new TilerunnerGtoBot();
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime startTimer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private Detector det;
    private static ImageTracker tracker;
    private static RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;
    private static MajorColorDetector.Color jewelColor = MajorColorDetector.Color.NONE;

    private static Point2d curPos;
    private static double  curHdg;
    private double initHdg = 0.0;
    private boolean gyroReady;
    private boolean usePostTurn = true;

    private static Field.StartPos startPos = Field.StartPos.START_1;
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

    private static final String TAG = "SJH_RRA";

    public static final void main(String args[])
    {
        //RrAutoShelby vas = new RrAutoShelby();

        Points pts = new RrPoints(startPos, alliance);
        List<Segment> pathSegs = new ArrayList<>();
        pathSegs.addAll(Arrays.asList(pts.getSegments()));

        for(Segment s : pathSegs)
        {
            System.out.println("Seg " + s.toString());
        }

        for(Segment s : pathSegs)
        {
            if(s.getName() == "RLTT")
            {
                s.getTgtPt().setX(s.getTgtPt().getX()+2.0);
            }
            System.out.println("Seg " + s.toString());
        }
        //vas.setup();
        //ShelbyBot bot = new TilerunnerGtoBot();
        //bot.init(vas);
        //RobotLog.dd(TAG, "Robot CPI %4.2f", bot.CPI);
    }
}

