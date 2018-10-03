package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.RrField;
import org.firstinspires.ftc.teamcode.field.RrRoute;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.MajorColorDetector;
import org.firstinspires.ftc.teamcode.image.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerGtoBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
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
        if(robot.gpitch != null)
        {
            robot.clearGpitch();
        }
        do_main_loop();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotLog.dd(TAG, "initCommon");
        initCommon(this, true, true, false, true);
        //RobotLog.dd(TAG, "super.runOpMode");
        //super.runOpMode();

        RobotLog.dd(TAG, "getBotName");
        robotName = pmgr.getBotName();

        RobotLog.dd(TAG, "logPrefs");
        pmgr.logPrefs();

        alliance = Field.Alliance.valueOf(pmgr.getAllianceColor());
        startPos = Route.StartPos.valueOf(pmgr.getStartPosition());
        try
        {
            parkPos  = Route.ParkPos.valueOf(pmgr.getParkPosition());
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ParkPosition %s invalid.", pmgr.getParkPosition());
            parkPos = Route.ParkPos.CENTER_PARK;
        }

        delay    = pmgr.getDelay();

        dashboard.displayPrintf(2, "Pref BOT: %s", robotName);
        dashboard.displayPrintf(3, "Pref Alliance: %s", alliance);
        dashboard.displayPrintf(4, "Pref StartPos: %s %s", startPos, parkPos);
        dashboard.displayPrintf(5, "Pref Delay: %.2f", delay);

        setup();
        //waitForStart();
        int initCycle = 0;
        int initSleep = 10;
        double scanFreqSec = 3.0;
        timer.reset();
        while(!isStarted())
        {
            gpad1.update();
            double shdg = robot.getGyroHdg();
            double fhdg = robot.getGyroFhdg();
            dashboard.displayPrintf(9, "HDG %4.2f FHDG %4.2f", shdg, fhdg);
            dashboard.displayPrintf(10, "GyroReady %s", gyroReady);
            dashboard.displayPrintf(11, "RGyroReady %s", robot.gyroReady);
            if(robot.leftMotor != null)
                dashboard.displayPrintf(12, "LENC %d", robot.leftMotor.getCurrentPosition());
            if(robot.leftMotor != null)
                dashboard.displayPrintf(13, "RENC %d", robot.rightMotor.getCurrentPosition());
            if(robot.elevMotor != null)
                dashboard.displayPrintf(14, "ELEV %d", robot.elevMotor.getCurrentPosition());

            if(doScanInInit && (initCycle == 0 || timer.seconds() > scanFreqSec))
            {
                RobotLog.ii(TAG, "doScan in Init at %6.4f initCycle %d",
                        timer.seconds(), initCycle);

                tracker.resetKey();
                doScan();
                timer.reset();
            }
            initCycle++;

            double pitDlta = 1.0;
            boolean ppitChange = false;
            if(gpad1.just_pressed(ManagedGamepad.Button.D_LEFT))
            {
                ppitChange = true;
                ppitXShift += pitDlta;
            }
            if(gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT))
            {
                ppitChange = true;
                ppitXShift -= pitDlta;
            }

            if(ppitChange) RobotLog.dd(TAG, "PitPoint Offsett %s", ppitXShift);
            sleep(initSleep);
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
        dashboard.displayPrintf(0, "PLEASE WAIT - STARTING - CHECK DEFAULTS");
        logData = true;

        dashboard.displayPrintf(6, "HIT A TO ACCEPT VALUES");
        dashboard.displayPrintf(7, "HIT B FOR MENU");
        RobotLog.ii(TAG, "SETUP");

        ElapsedTime mTimer = new ElapsedTime();
        boolean doMen = false;
        while(mTimer.seconds() < 3.0)
        {
            gpad1.update();
            if(gpad1.just_pressed(ManagedGamepad.Button.A))
            {
                doMen = false;
                break;
            }
            if(gpad1.just_pressed(ManagedGamepad.Button.B))
            {
                doMen = true;
                break;
            }
        }

        if(doMen) doMenus();

        dashboard.displayPrintf(0, "INITIALIZING");

        String teleopName = "TeleopDriver";

        if(robotName.equals("MEC"))
        {
            robot = new TilerunnerMecanumBot();
            teleopName = "Mecanum";
        }
        else
        {
            robot = new TilerunnerGtoBot();
        }

        dashboard.displayPrintf(1, "HERE1");

        //Since we only have 5 seconds between Auton and Teleop, automatically load
        //teleop opmode
        RobotLog.dd(TAG, "Setting up auto tele loader : %s", teleopName);
        AutoTransitioner.transitionOnStop(this, teleopName);

        dashboard.displayPrintf(1, "HERE2");

        robot.init(this, robotName);
        robot.setAlliance(alliance);

        dashboard.displayPrintf(1, "HERE3");

        RobotLog.dd(TAG, "Robot CPI " + robot.CPI);

        drvTrn.init(robot);
        drvTrn.setRampUp(false);

        robot.stowFlicker();
        robot.deployHolder();

        robot.initElevZero();
        robot.setElevAuton();
        if(!robotName.equals("MEC"))
        {
            RobotLog.dd(TAG, "Moving elev to auton pos");
            robot.indexElev();
            robot.setElevAuton();

            sleep(1000);
        }
        RobotLog.dd(TAG, "StowGripper");
        robot.stowGripper();
        robot.retractGpitch();

        dashboard.displayPrintf(1, "HERE4");

        RobotLog.dd(TAG, "JFLICKER_UP_POS %.2f", TilerunnerGtoBot.JFLICKER_UP_POS);
        RobotLog.dd(TAG, "JFLICKER_DOWN_POS %.2f", TilerunnerGtoBot.JFLICKER_DOWN_POS);
        RobotLog.dd(TAG, "JFLICKER_STOW_POS %.2f", TilerunnerGtoBot.JFLICKER_STOW_POS);
        RobotLog.dd(TAG, "GPITCH_UP_POS %.2f", TilerunnerGtoBot.GPITCH_UP_POS);
        RobotLog.dd(TAG, "GPITCH_DOWN_POS %.2f", TilerunnerGtoBot.GPITCH_DOWN_POS);
        RobotLog.dd(TAG, "GPITCH_CLEAR_POS %.2f", TilerunnerGtoBot.GPITCH_CLEAR_POS);
        RobotLog.dd(TAG, "GRIPPER_OPEN_POS %.2f", TilerunnerGtoBot.GRIPPER_OPEN_POS);
        RobotLog.dd(TAG, "GRIPPER_PART %.2f", TilerunnerGtoBot.GRIPPER_PARTIAL_POS);
        RobotLog.dd(TAG, "GRIPPER_CLOSE_POS %.2f", TilerunnerGtoBot.GRIPPER_CLOSE_POS);

        det = new MajorColorDetector();
        tracker = new ImageTracker(VuforiaInitializer.Challenge.RR);
        tracker.setTrackableRelativeCropCorners(RrField.getTrackableRelativeCropCorners());

        setupLogger();

        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        dl.addField("Park: "     + parkPos.toString());
        RobotLog.ii(TAG, "STARTPOS %s", startPos);
        RobotLog.ii(TAG, "ALLIANCE %s", alliance);
        RobotLog.ii(TAG, "PARKPOS %s", parkPos);
        RobotLog.ii(TAG, "DELAY    %4.2f", delay);
        RobotLog.ii(TAG, "BOT      %s", robotName);

        double glyphOff = pmgr.getGlyphOffset(robotName);
        Route pts = new RrRoute(startPos, alliance, robotName, glyphOff);
        pathSegs.addAll(Arrays.asList(pts.getSegments()));

        initHdg = pathSegs.get(0).getFieldHeading();

        ShelbyBot.DriveDir startDdir = pathSegs.get(0).getDir();
        robot.setDriveDir(startDdir);

        RobotLog.dd(TAG, "START DRIVEDIR =%s", startDdir);

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
        dashboard.displayPrintf(8, "PATH: Start at %s", currPoint);

        RobotLog.ii(TAG, "IHDG %4.2f", initHdg);

        det.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        int dropCycle = 0;

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

        deployPusher();

        if(key == RelicRecoveryVuMark.UNKNOWN ||
           jewelColor == MajorColorDetector.Color.NONE)
        {
            doScan();
        }
        if(jewelColor == MajorColorDetector.Color.NONE)
        {
            retractPusher();
            sleep(200);
        }

        if(!robotName.equals("MEC"))
        {
            if(!robot.getCapability("holder"))
            {
                robot.setElevAuton();
                sleep(200);
                robot.openGripper();
                sleep(100);
                robot.setElevZero();
                sleep(1250);
                //sleep(1000);
                robot.closeGripper();
                sleep(250);
                robot.setElevAuton();
            }
            else
            {
                robot.setElevAuton();
                sleep(200);
                robot.closeGripper();
                sleep(150);
                robot.stowHolder();
            }
        }
        else if(robotName.equals("MEC"))
        {
            robot.closeGripper();
            sleep(300);
            robot.setElevAuton();
        }

        boolean SkipNextSegment = false;
        boolean breakOut = false;

        if(!robotName.equals("MEC"))
        {
            drvTrn.setRampSpdL(0.15);
            drvTrn.setRampSpdM(0.35);
            drvTrn.setRampSpdH(0.60);
            drvTrn.setRampCntH(600);
            drvTrn.setRampCntM(300);
            drvTrn.setRampCntL(100);
        }

        boolean adjustedKeyPoint = false;
        Point2d origKeyPoint = null;
        for (int i = 0; i < pathSegs.size(); ++i)
        {
            if (!opModeIsActive() || isStopRequested()) break;

            String segName = pathSegs.get(i).getName();
            RobotLog.ii(TAG, "Starting segment %s at %4.2f", segName,
                    startTimer.seconds());

            //noinspection ConstantConditions
            if (SkipNextSegment)
            {
                SkipNextSegment = false;
                RobotLog.ii(TAG, "Skipping segment %s", pathSegs.get(i).getName());
                if (i < pathSegs.size() - 1)
                {
                    RobotLog.ii(TAG, "Setting segment %s start pt to %s",
                            pathSegs.get(i + 1).getName(),
                            pathSegs.get(i).getStrtPt());
                    pathSegs.get(i + 1).setStrtPt(pathSegs.get(i).getStrtPt());
                }
                continue;
            }

            Segment curSeg;

            if (curPos == null || !useImageLoc)
            {
                curSeg = pathSegs.get(i);
            } else
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

            if (curSeg.getLength() >= 0.1)
            {
                if (curSeg.getAction() == Segment.Action.PREGRAB)
                {
                    if (startTimer.seconds() > 25.0)
                    {
                        RobotLog.ii(TAG, "SKIP TURN MOVE %s t=%6.4f", curSeg.getName(),
                                startTimer.seconds());
                        doMove(curSeg);
                        break;
                    }
                }
                RobotLog.ii(TAG, "ENCODER TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doEncoderTurn(curSeg.getFieldHeading(), segName + " encoderTurn"); //quick but rough
                RobotLog.ii(TAG, "GYRO TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doGyroTurn(curSeg.getFieldHeading(), segName + " gyroTurn");

                if (curSeg.getAction() == Segment.Action.DROP)
                {
                    if (dropCycle == 0 || dropCycle == 2)
                    {
                        robot.setElevZero();
                        robot.openGripper();
                        if (startTimer.seconds() > 28.7)
                        {
                            break;
                        }
                    }
                    else if (dropCycle == 1)
                    {
                        if (startTimer.seconds() > 28.7)
                        {
                            break;
                        }
                    }
                }
                RobotLog.ii(TAG, "MOVE %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doMove(curSeg);
            }


            Double pturn = curSeg.getPostTurn();
            if (usePostTurn && pturn != null)
            {
                RobotLog.ii(TAG, "ENCODER POST TURN %s", curSeg.getName());
                doEncoderTurn(pturn, segName + " postEncoderTurn");

//                RobotLog.ii(TAG, "GRYO POST TURN %s", curSeg.getName());
//                doGyroTurn(pturn, segName + " postGyroTurn");
            }

            if (!opModeIsActive() || isStopRequested())
            {
                drvTrn.stopMotion();
                break;
            }

            RobotLog.ii(TAG, "Planned pos: %s %s",
                    pathSegs.get(i).getTgtPt(),
                    pathSegs.get(i).getFieldHeading());


            Segment.Action act = curSeg.getAction();

            RobotLog.ii(TAG, "ACTION %s %s t=%6.4f", curSeg.getName(), act,
                    startTimer.seconds());

            if (act != Segment.Action.NOTHING)
            {
                drvTrn.setInitValues();
                drvTrn.logData(true, segName + " action " + act.toString());
            }

            RrField.Align_Type align_type = RrField.Align_Type.COMMON_ANGLE;
            switch (act)
            {
//                case RST_PUSHER:
//                    robot.lpusher.setPosition(L_DN_PUSH_POS);
//                    break;
                case SCAN_IMAGE:
                    if(key == RelicRecoveryVuMark.UNKNOWN ||
                       jewelColor == MajorColorDetector.Color.NONE)
                    {
                        deployPusher();
                        doScan();
                        sleep(200);
                    }
                    if(jewelColor == MajorColorDetector.Color.NONE)
                    {
                        retractPusher();
                        sleep(200);
                    }
                    doPush(pathSegs.get(i + 1));
                    break;

                case SET_ALIGN:
                {
                    //Note:  The segment ordering in RrRoute needs to match below
                    Point2d apt = RrField.getAlignPt(alliance, startPos, key, align_type);
                    RobotLog.dd(TAG, "Setting align point %s %s", key, apt);
                    double cfgOff = pmgr.getDropOffset(robotName, alliance.toString(),
                            startPos.toString(), key.toString());
                    RobotLog.dd(TAG, "Drop offset for %s %s %s is %.2f",
                            alliance.toString(), startPos.toString(), key.toString(), cfgOff);
                    Point2d dpt = RrField.getDropPt(alliance, startPos, key, cfgOff, align_type);
                    RobotLog.dd(TAG, "Setting drop point %s %s", key, dpt);
                    int lstSeg = pathSegs.size() - 1;
                    RobotLog.dd(TAG, "LastSeg idx = %d", lstSeg);
                    Segment seg;
                    if(lstSeg < i+1) continue;
                    seg=pathSegs.get(i + 1); seg.setEndPt(apt);  seg.setName("FPAP"); //set_key
                    if(lstSeg < i+2) continue;
                    seg=pathSegs.get(i + 2); seg.setStrtPt(apt); seg.setName("APDP");
                                             seg.setEndPt(dpt);  //drop
                    if(lstSeg < i+3) continue;
                    seg=pathSegs.get(i + 3); seg.setStrtPt(dpt); seg.setName("DPAP");
                                             seg.setEndPt(apt);  //pregrab
                    if(lstSeg < i+4) continue;
                    seg=pathSegs.get(i + 4); seg.setStrtPt(apt); seg.setName("APXP"); //none
                    //(i+5=XP->PP) grab
                    //(i+6=PP->XP) none
                    if(lstSeg < i+7) continue;
                    seg=pathSegs.get(i + 7); seg.setEndPt(apt);  seg.setName("XPAP"); //key
                    if(lstSeg < i+8) continue;
                    seg=pathSegs.get(i + 8); seg.setStrtPt(apt); seg.setName("APDP");
                                             seg.setEndPt(dpt);  //drop

                    //noinspection ConstantConditions
                    if(doMore)
                    {
                        pathSegs.add(pathSegs.get(i + 3)); //DPAP pregrab
                        pathSegs.add(pathSegs.get(i + 4)); //APXP none
                        pathSegs.add(pathSegs.get(i + 5)); //XPPP grab
                        pathSegs.add(pathSegs.get(i + 6)); //PPXP none
                        pathSegs.add(pathSegs.get(i + 7)); //XPAP key
                        pathSegs.add(pathSegs.get(i + 8)); //APDP drop
                        pathSegs.add(pathSegs.get(i + 3)); //DPAP pregra?
                    }
                    else
                    {
                        pathSegs.add(pathSegs.get(i + 3)); //DPAP
                    }

                    robot.setElevZero();
                    break;
                }

                case SET_KEY:
                {
                    //drop (key) point now set in align
                    if(robot.gpitch != null) robot.deployGpitch();

                    //robot.setElevDrop();
                    RobotLog.dd(TAG, "DropCycle = %d", dropCycle);
                    dropCycle = Math.max(0, dropCycle);
                    dropCycle = Math.min(3, dropCycle);

                    if (dropCycle == 1) robot.setElevToIdxPos(1);
                    if (dropCycle == 2) robot.setElevToIdxPos(0);

                    //SBH
                    Point2d cp = drvTrn.getCurrPt();
                    Point2d ep = drvTrn.getEstPos();
                    double xerr = cp.distance(ep);
                    adjustedKeyPoint = false;
                    if(useKeyPtAdjust && Math.abs(xerr) > 1.0 && dropCycle > 0)
                    {
                        RobotLog.dd(TAG, "Adjusting AP position from %s to %s", cp, ep);
                        adjustedKeyPoint = true;
                        Point2d dbeg = pathSegs.get(i + 1).getStrtPt();
                        origKeyPoint = dbeg;
                        Point2d modPt  = new Point2d(dbeg.getName()+"mod",
                                                    drvTrn.getEstPos().getX(),
                                                    drvTrn.getEstPos().getY());
                        Segment s;
                        int lstSeg = pathSegs.size() - 1;
                        if(lstSeg >= i + 1)
                        {
                            pathSegs.get(i + 1).setStrtPt(modPt); //APDP drop
                        }
                        if(lstSeg >= i + 2)
                        {
                            pathSegs.get(i + 2).setEndPt(modPt);  //DPAP pgrb
                        }
                        if(lstSeg >= i + 3)
                        {
                            pathSegs.get(i + 3).setStrtPt(modPt); //APXP none
                        }
                    }

                    if(doMore && dropCycle > 1 && startPos == Route.StartPos.START_1)
                    {
                        //shift to another drop location
                        Segment s;
                        s=pathSegs.get(i + 1);
                        Point2d curAlgn = s.getStrtPt();
                        double deltaX = 0.0;

                        int allInt = 0;
                        if (alliance == Field.Alliance.RED) allInt = 1;
                        int newKeyInt = 1; //Center
                        if(key == RelicRecoveryVuMark.CENTER)
                        {
                            newKeyInt = 2;
                            if(alliance == Field.Alliance.BLUE) newKeyInt = 0;
                        }
                        else if(key == RelicRecoveryVuMark.LEFT)  newKeyInt = 2;
                        else if(key == RelicRecoveryVuMark.RIGHT) newKeyInt = 0;

                        Point2d newCbox= RrField.CPTS[allInt][0][newKeyInt];

                        Point2d newDrop = RrField.calcDropPt("Drop3", curAlgn, newCbox);

                        s.setEndPt(newDrop);
                        s = pathSegs.get(i + 2);
                        s.setStrtPt(newDrop);
                    }

                    break;
                }

                case DROP:
                {
                    if(!robotName.equals("MEC"))
                    {
                        drvTrn.setRampSpdL(0.15);
                        drvTrn.setRampSpdM(0.35);
                        drvTrn.setRampSpdH(0.60);
                        drvTrn.setRampCntH(600);
                        drvTrn.setRampCntM(300);
                        drvTrn.setRampCntL(100);
                    }

                    if(robot.gpitch != null)
                    {
                        robot.deployGpitch();
                        sleep(500);
                    }

                    if(dropCycle == 0)
                    {
                        if(robot.gpitch != null)
                        {
                            robot.retractGpitch();
                        }
                    }

                    if(dropCycle == 1)
                    {
                        robot.partialGripper();
                        sleep(100);
                    }

//                    if(startTimer.seconds() < 28.5)
//                    {
//                        boolean curRampDown = drvTrn.getRampDown();
//                        drvTrn.setRampDown(false);
//                        double glyphPushDist = 3.0;
//
//                        Drivetrain.Direction dfwd = Drivetrain.Direction.FORWARD;
//                        RobotLog.dd(TAG, "PushGlyph %f at %6.4f",
//                                glyphPushDist, startTimer.seconds());
//                        //SBH drvTrn.driveDistanceLinear(glyphPushDist, 0.2, dfwd);
//                        robot.partialGripper();
//                        RobotLog.dd(TAG, "PushGlyph %f at %6.4f",
//                                -glyphPushDist, startTimer.seconds());
//                        //SBH drvTrn.driveDistanceLinear(-glyphPushDist, 0.2, dfwd);
//                        drvTrn.setRampDown(curRampDown);
//                    }

                    dropCycle++;

                    break;
                }

                case PREGRAB:
                {
                    if(robot.gpitch != null)
                    {
                        robot.downGpitch();
                    }
                    else
                    {
                        robot.setElevZero();
                    }
                    robot.openGripper();

                    if(dropCycle == 0)
                    {
                        //Shift pitpoint side to side for best glyph grab
                        Segment s = pathSegs.get(i + 1);
                        Point2d cpp = s.getTgtPt();
                        int lstSeg = pathSegs.size() - 1;
                        double cppX = cpp.getX() + ppitXShift;
                        cpp.setX(cppX);

                        if(lstSeg >= i + 1)
                        {
                            s.setEndPt(cpp); //APXP
                        }
                        if(lstSeg >= i + 2)
                        {
                            s = pathSegs.get(i + 2);
                            s.setStrtPt(cpp);
                            s.setEndPt(cpp); //XPPP
                        }
                        if(lstSeg >= i + 3)
                        {
                            s = pathSegs.get(i + 3);
                            s.setStrtPt(cpp);
                            s.setEndPt(cpp); //PPXP
                        }
                        if(lstSeg >= i + 4)
                        {
                            s = pathSegs.get(i + 4);
                            s.setStrtPt(cpp);                  //XPAP
                        }
                    }

                    if(startPos == Route.StartPos.START_1 && doMore && dropCycle != 0)
                    {
                        //Make pitpoint deeper on 2nd pass
                        Segment s;
                        double pitX = RrField.getPPx();
                        double pitY = RrField.getPPy1();
                        if (dropCycle == 2)
                        {
                            pitY = RrField.getPPy2();
                        }
                        else
                        {
                            if(parkPos  == Route.ParkPos.DEFEND_PARK)
                            {
                                pitY = RrField.getPPyD(startPos);
                            }
                        }

                        if (alliance == Field.Alliance.BLUE) pitY = -pitY;

                        Point2d ppt = null;
                        int lstSeg = pathSegs.size() - 1;
                        if(lstSeg >= i + 2)
                        {
                            ppt = pathSegs.get(i + 2).getTgtPt();
                        }

                        if(ppt != null)
                        {
                            if(parkPos  == Route.ParkPos.DEFEND_PARK)
                            {
                                ppt.setX(pitX);
                            }
                            ppt.setY(pitY);
                            RobotLog.dd(TAG, "Adjusting PitPoint to %s", ppt);
                            if (lstSeg >= i + 1)
                            {
                                s = pathSegs.get(i + 1);
                                s.setEndPt(ppt); //APXP
                            }
                            if (lstSeg >= i + 2)
                            {
                                s = pathSegs.get(i + 2);
                                s.setStrtPt(ppt);
                                s.setEndPt(ppt); //XPPP
                            }
                            if (lstSeg >= i + 3)
                            {
                                s = pathSegs.get(i + 3);
                                s.setStrtPt(ppt);
                                s.setEndPt(ppt); //PPXP
                            }
                            if (lstSeg >= i + 4)
                            {
                                s = pathSegs.get(i + 4);
                                s.setStrtPt(ppt);                  //XPAP
                            }
                        }
                    }

                    if(robot.getName().equals("MEC"))
                    {
                        robot.closeGripper();
                        breakOut = true;
                    }

                    double tooLate = 25.0;
                    if(startPos == Route.StartPos.START_2) tooLate = 22.0;
                    if(startTimer.seconds() > tooLate) breakOut = true;

                    break;
                }

                case GRAB:
                {
                    if(useKeyPtAdjust && adjustedKeyPoint)
                    {
                        RobotLog.dd(TAG, "ReAdjusting position");
                        adjustedKeyPoint = false;
                        pathSegs.get(i-3).setStrtPt(origKeyPoint); //APDP drop
                        pathSegs.get(i-2).setEndPt(origKeyPoint);  //DPAP pgrb
                        pathSegs.get(i-1).setStrtPt(origKeyPoint); //APXP none
                    }
                    robot.closeGripper();
                    sleep(230);

                    if(robot.gpitch != null)
                    {
                        sleep(200);
                        robot.retractGpitch();
                    }
                    else
                    {
                        int elvl = dropCycle;
                        if (dropCycle == 2) elvl = 0;
                        robot.setElevToIdxPos(elvl);
                    }

                    if(dropCycle == 2 && startPos == Route.StartPos.START_1)
                    {
                        Segment s;
                        int lstSeg = pathSegs.size();
                        if(lstSeg >= i + 2)
                        {
                            s = pathSegs.get(i + 2);
                            Point2d ap = s.getTgtPt();
                            double y = RrField.BSY2;
                            if(alliance == Field.Alliance.RED) y = -y;
                            ap.setY(y);
                            s.setEndPt(ap);
                            if(lstSeg >= i + 3)
                            {
                                s = pathSegs.get(i + 3);
                                s.setStrtPt(ap);
                            }
                            if(lstSeg >= i + 4)
                            {
                                s = pathSegs.get(i + 4);
                                s.setStrtPt(ap);
                            }
                        }
                    }
                    break;
                }

                case RETRACT:
                {
                    //kludge offset for BLTT being off.
                    double ttOff = 0.0;
                    if(alliance == Field.Alliance.BLUE) ttOff = 0.0;

                    if(!robotName.equals("MEC"))
                    {
                        Point2d ttpt = pathSegs.get(i + 1).getTgtPt();
                        Point2d ntt = new Point2d("KTT", ttpt.getX() + ttOff, ttpt.getY());
                        pathSegs.get(i + 1).setEndPt(ntt);
                    }

                    robot.closeGripper();
                    if(robot.gpitch != null)
                    {
                        robot.retractGpitch();
                    }
                    break;
                }

                case ESCAPE:
                {
//                    if(robot.getName().equals("MEC"))
//                    {
//                        robot.closeGripper();
//                        breakOut = true;
//                    }
                    break;
                }

                case THROW:
                {
                    if(robot.gpitch != null)
                    {
                        robot.deployGpitch();
                        sleep(100);
                        robot.openGripper();
                        sleep(200);
                        robot.retractGpitch();
                    }
                    else
                    {
                        if(startTimer.seconds() < 28)
                        {
                            drvTrn.driveDistanceLinear(8.0, 0.3, Drivetrain.Direction.FORWARD);
                            robot.openGripper();
                            sleep(100);
                            drvTrn.driveDistanceLinear(-6.0, 0.3, Drivetrain.Direction.FORWARD);
                        }
                    }

                    break;
                }
            }

            if(breakOut) break;
        }

        RobotLog.dd(TAG, "Finished auton segments");
        robot.setAutonEndHdg(robot.getGyroFhdg());
        //robot.setAutonEndPos(drvTrn.getCurrPt());
        robot.setAutonEndPos(drvTrn.getEstPos());

        while(opModeIsActive() && !isStopRequested())
        {
            idle();
        }
    }

    private void doScan()
    {
        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(true) ;
        key = getKey();
        jewelColor =  getJewelColor();
        RobotLog.dd(TAG, "SCAN_IMAGE KEY = %s jewelColor = %s",
                key, jewelColor);

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(false);
    }

    private void deployPusher()
    {
        if(robot.gpitch != null)
        {
            robot.clearGpitch();
            sleep(400);
        }
        robot.deployFlicker();
        //sleep(600);

        RobotLog.dd(TAG, "Deployed pusher");
    }

    private void retractPusher()
    {
        if(robot.gpitch != null)
        {
            robot.clearGpitch();
        }
        robot.raiseFlicker();
        //sleep(300);
        RobotLog.dd(TAG, "Retracted pusher");
    }

    private void doPush(Segment postPushSeg)
    {
        if(jewelColor == MajorColorDetector.Color.NONE)
            return;

        //deployPusher();
        //move to knock off jewel
        double jewelPushDist = 3.2;
        double jewelPushSpd  = 0.18;
        int pushSign = 1;

        ShelbyBot.DriveDir segDir = ShelbyBot.DriveDir.INTAKE;
        switch (jewelColor)
        {
            case RED:
                if(alliance == Field.Alliance.RED)
                {
                    segDir = ShelbyBot.DriveDir.PUSHER;
                    pushSign = -1;
                }
                else
                {
                    segDir = ShelbyBot.DriveDir.INTAKE;
                    pushSign = -1;
                }
                break;

            case BLUE:
                if(alliance == Field.Alliance.BLUE)
                {
                    segDir = ShelbyBot.DriveDir.PUSHER;
                    pushSign = 1;
                }
                else
                {
                    segDir = ShelbyBot.DriveDir.INTAKE;
                    pushSign = 1;
                }
                break;
        }
        MajorColorDetector.Color badJewel = MajorColorDetector.Color.BLUE;
        if(alliance == Field.Alliance.BLUE) badJewel = MajorColorDetector.Color.RED;
        RobotLog.dd(TAG, "Moving %s %4.2f at %4.2f to knock off jewel %s alliance %s",
                segDir, jewelPushDist * pushSign, jewelPushSpd, badJewel, alliance);

        Point2d pushStart = postPushSeg.getStrtPt();
        Point2d pushEnd   = new Point2d("PUSHENDPT",
                                               pushStart.getX() + jewelPushDist * pushSign,
                                               pushStart.getY());

        robot.setDriveDir(segDir);

        boolean curRampDown = drvTrn.getRampDown();
        drvTrn.setRampDown(false);
        Segment pushSeg = new Segment("PushSeg", pushStart, pushEnd);
        pushSeg.setAction(Segment.Action.NOTHING);
        pushSeg.setDir(segDir);
        pushSeg.setSpeed(jewelPushSpd);
        doMove(pushSeg);
        drvTrn.setRampDown(curRampDown);
        retractPusher();

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
        if(Math.abs(angle) <= 4.0) return;

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
        if(opModeIsActive() && key != RelicRecoveryVuMark.UNKNOWN)
            return key;
        RelicRecoveryVuMark lockey = RelicRecoveryVuMark.UNKNOWN;
        tracker.setActive(true);
        RobotLog.dd(TAG, "Finding VuMark first");

        double keyTimeout = 1.0;
        ElapsedTime ktimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive()                      &&
               lockey == RelicRecoveryVuMark.UNKNOWN &&
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
                lockey = tracker.getKeyLoc();
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

        RobotLog.ii(TAG, "KEY : " + lockey);
        dashboard.displayPrintf(0, "KEY: " + lockey);
        RobotLog.dd(TAG, "Turn off image tracker");
        tracker.setActive(false);

        return lockey;
    }

    private MajorColorDetector.Color getJewelColor()
    {
        if(opModeIsActive()  && jewelColor != MajorColorDetector.Color.NONE)
            return jewelColor;

        jewelColor = MajorColorDetector.Color.NONE;
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

        dashboard.displayPrintf(1, "JWL: " + leftJewelColor);

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
        FtcChoiceMenu<PositionOption> startPosMenu =
                new FtcChoiceMenu<>("START:", null, this);
        FtcChoiceMenu<Field.Alliance> allianceMenu =
                new FtcChoiceMenu<>("ALLIANCE:", startPosMenu, this);
        FtcChoiceMenu<String> robotNameMenu =
                new FtcChoiceMenu<>("BOT_NAME:", allianceMenu, this);
        FtcChoiceMenu<PositionOption> parkMenu
                = new FtcChoiceMenu<>("Park:",   robotNameMenu, this);
        FtcValueMenu delayMenu
                = new FtcValueMenu("DELAY:", parkMenu, this,
                0.0, 20.0, 1.0, 0.0, "%5.2f");

        startPosMenu.addChoice("Start_1", Route.StartPos.START_1, allianceMenu);
        startPosMenu.addChoice("Start_2", Route.StartPos.START_2, allianceMenu);

        allianceMenu.addChoice("RED",  Field.Alliance.RED,  parkMenu);
        allianceMenu.addChoice("BLUE", Field.Alliance.BLUE, parkMenu);

        parkMenu.addChoice("CENTER_PARK", Route.ParkPos.CENTER_PARK, delayMenu);
        parkMenu.addChoice("DEFEND_PARK", Route.ParkPos.DEFEND_PARK, delayMenu);

        robotNameMenu.addChoice("GTO1", "GTO1", delayMenu);
        robotNameMenu.addChoice("GTO2", "GTO2", delayMenu);
        robotNameMenu.addChoice("MEC", "MEC", delayMenu);

        FtcMenu.walkMenuTree(startPosMenu, this);

        startPos  = startPosMenu.getCurrentChoiceObject();
        alliance  = allianceMenu.getCurrentChoiceObject();
        robotName = robotNameMenu.getCurrentChoiceObject();
        parkPos   = parkMenu.getCurrentChoiceObject();
        delay     = delayMenu.getCurrentValue();

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

    private final static double DEF_ENCTRN_PWR  = 0.6;
    private final static double DEF_GYRTRN_PWR = 0.4;

    private List<Segment> pathSegs = new ArrayList<>();

    private TilerunnerGtoBot   robot;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime startTimer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private Detector det;
    private static ImageTracker tracker;
    private RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;
    private MajorColorDetector.Color jewelColor = MajorColorDetector.Color.NONE;

    private static Point2d curPos;
    private static double  curHdg;
    private double initHdg = 0.0;
    private boolean gyroReady;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean usePostTurn = true;

    private static PositionOption startPos = Route.StartPos.START_1;
    private static Field.Alliance alliance = Field.Alliance.RED;
    private static Team team = Team.SNOWMAN;

    private static PositionOption parkPos = Route.ParkPos.CENTER_PARK;

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

    @SuppressWarnings("FieldCanBeLocal")
    private boolean useLight = true;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean doMore = true;

    private double ppitXShift = 0.0;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean doScanInInit = false;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean useKeyPtAdjust = true;

    private String robotName = "";
    private static final String TAG = "SJH_RRA";
}