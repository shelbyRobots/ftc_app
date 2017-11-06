package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
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
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
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
        if(drvTrn != null) drvTrn.cleanup();
        det.cleanupCamera();
    }

    private void setup()
    {
        dashboard.displayPrintf(0, "PLEASE WAIT - STARTING");
        logData = true;

        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        RobotLog.ii(TAG, "SETUP");

        doMenus();

        String teleopName = "TeleopDriver";

        if(robotName.equals("MEC1"))
        {
            robot = new TilerunnerMecanumBot();
            teleopName = "Mecanum";
        }
        else
        {
            robot = new TilerunnerGtoBot();
        }

        //Since we only have 5 seconds between Auton and Teleop, automatically load
        //teleop opmode
//        RobotLog.dd(TAG, "Setting up auto tele loader : %s", teleopName);
//        AutoTransitioner.transitionOnStop(this, teleopName);

        robot.init(this, robotName);

        System.out.println("Robot CPI " + robot.CPI);

        drvTrn.init(robot);
        drvTrn.setRampUp(false);

        robot.jflicker.setPosition(TilerunnerGtoBot.JFLICKER_UP_POS);
        robot.gripper.setPosition(TilerunnerGtoBot.GRIPPER_CLOSE_POS);
        robot.gpitch.setPosition(TilerunnerGtoBot.GPITCH_UP_POS);

        det = new MajorColorDetector();
        tracker = new ImageTracker(VuforiaInitializer.Challenge.RR);
        tracker.setTrackableRelativeCropCorners(RrField.getTrackableRelativeCropCorners());

        setupLogger();

        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        RobotLog.ii(TAG, "STARTPOS %s", startPos);
        RobotLog.ii(TAG, "ALLIANCE %s", alliance);
        RobotLog.ii(TAG, "DELAY    %4.2f", delay);
        RobotLog.ii(TAG, "BOT      %s", robotName);

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

        RobotLog.ii(TAG, "ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs.get(0).getStrtPt();
        drvTrn.setCurrPt(currPoint);

        drvTrn.setStartHdg(initHdg);
        robot.setInitHdg(initHdg);

        RobotLog.ii(TAG, "Start %s.", currPoint);
        dashboard.displayPrintf(3, "PATH: Start at %s", currPoint);

        RobotLog.ii(TAG, "IHDG %4.2f", initHdg);

        det.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        RobotLog.ii(TAG, "STARTING AT %4.2f", timer.seconds());
        if(logData)
        {
            Point2d spt = pathSegs.get(0).getStrtPt();
            dl.addField("START");
            dl.addField(initHdg);
            dl.addField(spt.getX());
            dl.addField(spt.getY());
            dl.newLine();
        }

        RobotLog.ii(TAG, "Delaying for %4.2f seconds", delay);
        ElapsedTime delayTimer = new ElapsedTime();
        while (opModeIsActive() && delayTimer.seconds() < delay)
        {
            idle();
        }

        RobotLog.ii(TAG, "Done delay");

        RobotLog.ii(TAG, "START CHDG %6.3f", robot.getGyroHdg());

        robot.resetGyro();

        boolean SkipNextSegment = false;
        for (int i = 0; i < pathSegs.size(); ++i)
        {
            if(!opModeIsActive() || isStopRequested()) break;

            String segName = pathSegs.get(i).getName();
            RobotLog.ii(TAG, "Starting segment %s at %4.2f", segName,
                    startTimer.seconds());

            //noinspection ConstantConditions
            if (SkipNextSegment)
            {
                SkipNextSegment = false;
                RobotLog.ii(TAG, "Skipping segment %s", pathSegs.get(i).getName());
                if(i < pathSegs.size() - 1)
                {
                    RobotLog.ii(TAG, "Setting segment %s start pt to %s",
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

            RobotLog.ii(TAG, "ENCODER TURN %s", curSeg.getName());

            if(curSeg.getLength() >= 0.01)
            {
                doEncoderTurn(curSeg.getFieldHeading(), segName + " encoderTurn"); //quick but rough
                doGyroTurn(curSeg.getFieldHeading(), segName + " gyroTurn");
                doMove(curSeg);
            }


            Double pturn = curSeg.getPostTurn();
            if(usePostTurn && pturn != null)
            {
                RobotLog.ii(TAG, "ENCODER POST TURN %s", curSeg.getName());
                doEncoderTurn(pturn, segName + " postEncoderTurn");

//                RobotLog.ii(TAG, "GRYO POST TURN %s", curSeg.getName());
//                doGyroTurn(pturn, segName + " postGyroTurn");
            }

            if(!opModeIsActive() || isStopRequested())
            {
                drvTrn.stopMotion();
                break;
            }

            RobotLog.ii(TAG, "Planned pos: %s %s",
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
                    robot.gpitch.setPosition(TilerunnerGtoBot.GPITCH_DOWN_POS);
                    sleep(500);
                    robot.gripper.setPosition(TilerunnerGtoBot.GRIPPER_PARTIAL_POS);
                    robot.gpitch.setPosition(TilerunnerGtoBot.GPITCH_UP_POS);
                    sleep(300);
                    break;

                case GRAB:
                    robot.gpitch.setPosition(TilerunnerGtoBot.GPITCH_DOWN_POS);
                    sleep(500);
                    robot.gripper.setPosition(TilerunnerGtoBot.GRIPPER_CLOSE_POS);
                    robot.gpitch.setPosition(TilerunnerGtoBot.GPITCH_UP_POS);
                    break;
            }
        }

        ShelbyBot.autonEndHdg = robot.getGyroFhdg();

        while(opModeIsActive() && !isStopRequested())
        {
            idle();
        }

    }

    private void doScanPush(Segment postPushSeg)
    {
        double jewelPushDist = 2.0;
        double jewelPushSpd  = 0.2;
        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;

        key = getKey();
        MajorColorDetector.Color jewelColor;
        jewelColor = getJewelColor();
        RobotLog.dd(TAG, "SCAN_IMAGE KEY = %s jewelColor = %s",
                key, jewelColor);
        if(jewelColor == MajorColorDetector.Color.NONE)
            return;

        robot.jflicker.setPosition(TilerunnerGtoBot.JFLICKER_DOWN_POS);
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

        ShelbyBot.DriveDir segDir = ShelbyBot.DriveDir.INTAKE;
        if(ddir == Drivetrain.Direction.REVERSE)
        {
            if(postPushSeg.getDir() == ShelbyBot.DriveDir.INTAKE)
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

        robot.jflicker.setPosition(TilerunnerGtoBot.JFLICKER_UP_POS);
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
        RobotLog.ii(TAG, "Setting drive tuner to %4.2f", seg.getDrvTuner());
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

        RobotLog.ii(TAG, "Drive %s %s %s %6.2f %3.2f %s tune: %4.2f %s",
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
            List<Integer> colSegLbegs = new ArrayList<>(robot.liftPositions.size());
            List<Integer> colSegRbegs = new ArrayList<>(robot.liftPositions.size());
            drvTrn.setPositions(colSegLbegs, drvTrn.curLpositions, 0);
            drvTrn.setPositions(colSegRbegs, drvTrn.curRpositions, 0);

            List<Integer> colSegLends = new ArrayList<>(robot.liftPositions.size());
            List<Integer> colSegRends = new ArrayList<>(robot.liftPositions.size());
            int fullSegLenCnts = drvTrn.distanceToCounts(fullSegLen);
            drvTrn.setPositions(colSegLends, colSegLbegs, fullSegLenCnts);
            drvTrn.setPositions(colSegRends, colSegRbegs, fullSegLenCnts);

            List<Integer> linLpositions = new ArrayList<>(robot.liftPositions.size());
            List<Integer> linRpositions = new ArrayList<>(robot.liftPositions.size());
            drvTrn.setPositions(linLpositions, colSegLends, 0);
            drvTrn.setPositions(linRpositions, colSegRends, 0);

            Point2d cept = new Point2d(pct * (ept.getX() - spt.getX()) + spt.getX(),
                                       pct * (ept.getY() - spt.getY()) + spt.getY());

            double colDist = cept.distance(ept);
            double ovrDist = 2.0;
            int colCnts = drvTrn.distanceToCounts(colDist);
            int ovrCnts = drvTrn.distanceToCounts(ovrDist);

            //noinspection ConstantConditions
            if(singleSeg)
            {
                drvTrn.driveDistanceLinear(fullSegLen, speed, ddir, fhd, true);
            }
            else
            {
                drvTrn.driveToPointLinear(cept, speed, ddir, fhd);
                //drvTrn.driveToTarget(0.2, 20);
                robot.turnColorOn();

                sleep(10);

                double colSpd = 0.10;
                RobotLog.ii(TAG, "Color Driving to pt %s at speed %4.2f", ept, colSpd);
                String colDistStr = String.format(Locale.US, "%4.2f %s",
                        colDist,
                        cept.toString());
                drvTrn.logData(true, "FIND_LINE CDIST: " + colDistStr);

                drvTrn.driveDistance(colDist+ovrDist, colSpd, Drivetrain.Direction.FORWARD);

                while(opModeIsActive() && !isStopRequested())
                {
                    drvTrn.setCurValues();
                    drvTrn.logData();

                    int lTrav = Math.abs(drvTrn.curLpositions.get(0) -
                                         drvTrn.begLpositions.get(0));
                    int rTrav = Math.abs(drvTrn.curRpositions.get(0) -
                                         drvTrn.begRpositions.get(0));

                    int totColor = drvTrn.curRed + drvTrn.curGrn + drvTrn.curBlu;

                    if (totColor > COLOR_THRESH)
                    {
                        drvTrn.setPositions(linLpositions, drvTrn.curLpositions, 0);
                        drvTrn.setPositions(linRpositions, drvTrn.curRpositions, 0);
                        colGyroOffset = 50;
                        if(snm.equals("BECN2"))
                        {
                            drvTrn.setPositions(linLpositions, linLpositions, -colGyroOffset);
                            drvTrn.setPositions(linRpositions, linRpositions, -colGyroOffset);
                        }
                        drvTrn.stopMotion();
                        drvTrn.setEndValues("COLOR_FIND " + linLpositions.get(0) + " " +
                                                            linRpositions.get(0));
                        RobotLog.ii(TAG, "FOUND LINE");
                        drvTrn.setPositions(drvTrn.tgtLpositions, linLpositions, 0);
                        drvTrn.setPositions(drvTrn.tgtRpositions, linRpositions, 0);
                        drvTrn.logOverrun(0.1);
                        break;
                    }

                    if(lTrav > (colCnts + ovrCnts) ||
                       rTrav > (colCnts + ovrCnts))
                    {
                        drvTrn.stopMotion();
                        drvTrn.setEndValues("COLOR_MISS - go to" + linLpositions.get(0) + " " +
                                                    linRpositions.get(0));
                        RobotLog.ii(TAG, "REACHED OVERRUN PT - Backing up a bit");
                        drvTrn.setPositions(drvTrn.tgtLpositions, linLpositions, 0);
                        drvTrn.setPositions(drvTrn.tgtRpositions, linRpositions, 0);
                        drvTrn.logOverrun(0.1);
                        break;
                    }

                    drvTrn.makeGyroCorrections(colSpd, fhd, Drivetrain.Direction.FORWARD);

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

        RobotLog.ii(TAG, "Completed move %s. Time: %6.3f HDG: %6.3f",
                seg.getName(), timer.time(), robot.getGyroFhdg());
    }


    private void doEncoderTurn(double fHdg, int thresh, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        drvTrn.setBusyAnd(true);
        drvTrn.setInitValues();
        drvTrn.logData(true, prefix);
        double cHdg = drvTrn.curHdg;
        double angle = fHdg - cHdg;
        RobotLog.ii(TAG, "doEncoderTurn CHDG %6.3f THDG %6.3f", cHdg, fHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 2.0) return;

        RobotLog.ii(TAG, "Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle, DEF_ENCTRN_PWR, thresh);
        cHdg = robot.getGyroFhdg();
        RobotLog.ii(TAG, "Completed turn %5.2f. Time: %6.3f CHDG: %6.3f",
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

        RobotLog.ii(TAG, "doGyroTurn CHDG %4.2f THDG %4.2f", cHdg, fHdg);

        if(Math.abs(fHdg-cHdg) < 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(fHdg, DEF_GYRTRN_PWR);

        cHdg = drvTrn.curHdg;
        RobotLog.ii(TAG, "Completed turnGyro %4.2f. Time: %6.3f CHDG: %4.2f",
                fHdg, timer.time(), cHdg);
    }

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    private RelicRecoveryVuMark getKey()
    {
        RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;
        tracker.setActive(true);
        RobotLog.dd(TAG, "Finding VuMark first");

        double keyTimeout = 1.0;
        ElapsedTime ktimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive()                   &&
               key == RelicRecoveryVuMark.UNKNOWN &&
               ktimer.seconds() < keyTimeout)
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
                RobotLog.ii(TAG, "Senesed Pos: %s %5.2f %2.3f",
                        sensedBotPos, sensedFldHdg, t);
                RobotLog.ii(TAG, "IMG %s frame %d",
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

        double jewelTimeout = 1.0;
        ElapsedTime jtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()                                &&
              leftJewelColor == MajorColorDetector.Color.NONE &&
              jtimer.seconds() < jewelTimeout)
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
        FtcChoiceMenu<Field.StartPos> startPosMenu =
                new FtcChoiceMenu<>("START:", null, this);
        FtcChoiceMenu<Field.Alliance> allianceMenu =
                new FtcChoiceMenu<>("ALLIANCE:", startPosMenu, this);
        FtcChoiceMenu<String> robotNameMenu =
                new FtcChoiceMenu<>("BOT_NAME:", allianceMenu, this);
        FtcValueMenu delayMenu     = new FtcValueMenu("DELAY:", robotNameMenu, this,
                0.0, 20.0, 1.0, 0.0, "%5.2f");

        startPosMenu.addChoice("Start_A", Field.StartPos.START_1, allianceMenu);
        startPosMenu.addChoice("Start_B", Field.StartPos.START_2, allianceMenu);

        allianceMenu.addChoice("RED",  Field.Alliance.RED,  robotNameMenu);
        allianceMenu.addChoice("BLUE", Field.Alliance.BLUE, robotNameMenu);

        robotNameMenu.addChoice("GTO1", "GTO1", delayMenu);
        robotNameMenu.addChoice("GTO2", "GTO2", delayMenu);
        robotNameMenu.addChoice("MEC1", "MEC1", delayMenu);

        FtcMenu.walkMenuTree(startPosMenu, this);

        startPos = startPosMenu.getCurrentChoiceObject();
        alliance = allianceMenu.getCurrentChoiceObject();
        robotName = robotNameMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();

        int lnum = 3;
        dashboard.displayPrintf(lnum++, "START: %s", startPos);
        dashboard.displayPrintf(lnum++, "ALLIANCE: %s", alliance);
        //noinspection UnusedAssignment
        dashboard.displayPrintf(lnum++, "NAME: %s", robotName);
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

    private List<Segment> pathSegs = new ArrayList<>();

    private TilerunnerGtoBot   robot;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime startTimer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private Detector det;
    private static ImageTracker tracker;
    private RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;

    private static Point2d curPos;
    private static double  curHdg;
    private double initHdg = 0.0;
    private boolean gyroReady;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean usePostTurn = true;

    private static Field.StartPos startPos = Field.StartPos.START_1;
    private static Field.Alliance alliance = Field.Alliance.RED;
    private static Team team = Team.SNOWMAN;

    private int RED_THRESH = 15;
    private int GRN_THRESH = 15;
    private int BLU_THRESH = 15;
    @SuppressWarnings("FieldCanBeLocal")
    private int COLOR_THRESH = 20;

    private double delay = 0.0;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean useImageLoc  = false;
    private boolean firstInState = true;

    private int postSleep = 150;

    private int colSegNum = 0;

    private String robotName = "";
    private static final String TAG = "SJH_RRA";
}