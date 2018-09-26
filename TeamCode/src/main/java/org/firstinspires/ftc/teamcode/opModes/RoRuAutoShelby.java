package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.field.RoRuRoute;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.RrField;
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

//After starting on latch, opmode needs to lower/detach
// Then
//  - align (possibly with tape
//  - find gold mineral
//  - move gold mineral
//  - optionally knock partners gold mineral
//  - place marker in depot
//  - park at pit

@SuppressWarnings({"unused", "ForLoopReplaceableByForEach"})
@Autonomous(name="RoRuAutoShelby", group="Auton")
//@Disabled
public class RoRuAutoShelby extends InitLinearOpMode implements FtcMenu.MenuButtons
{
    public RoRuAutoShelby()
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
        RobotLog.dd(TAG, "initCommon");
        initCommon(this, true, true, false, true);

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

            initCycle++;

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

        dashboard.displayPrintf(1, "HERE4");

        //RobotLog.dd(TAG, "JFLICKER_UP_POS %.2f", TilerunnerGtoBot.JFLICKER_UP_POS);

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

        Route pts = new RoRuRoute(startPos, alliance, robotName);
        pathSegs.addAll(Arrays.asList(pts.getSegments()));
//TODO Figure out how to initialize heading
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

        //robot.resetGyro();

        //TODO: Change to RoverRuckus
//        if(key == RelicRecoveryVuMark.UNKNOWN ||
//           mineralColor == MajorColorDetector.Color.NONE)
//        {
//            doScan();
//        }

        //doLower();
        //doRelease();

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
                RobotLog.ii(TAG, "ENCODER TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doEncoderTurn(curSeg.getFieldHeading(), segName + " encoderTurn"); //quick but rough
                RobotLog.ii(TAG, "GYRO TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doGyroTurn(curSeg.getFieldHeading(), segName + " gyroTurn");


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

            switch (act)
            {
                case SCAN_IMAGE:
                    if(key == RelicRecoveryVuMark.UNKNOWN ||
                       mineralColor == MajorColorDetector.Color.NONE)
                    {
                        doScan();
                        sleep(200);
                    }
                    break;

                case SET_ALIGN:
                {
                    break;
                }

                case SET_KEY:
                {
                    break;
                }

                case DROP:
                {
                    break;
                }
            }
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
//        if(useLight)
//            CameraDevice.getInstance().setFlashTorchMode(true) ;
//        mineralColor =  getMineralColor();
//        RobotLog.dd(TAG, "SCAN_IMAGE KEY = %s mineralColor = %s",
//                key, mineralColor);
//
//        if(useLight)
//            CameraDevice.getInstance().setFlashTorchMode(false);
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
    private MajorColorDetector.Color getMineralColor()
    {
//        if(opModeIsActive()  && mineralColor != MajorColorDetector.Color.NONE)
//            return mineralColor;
//
//        mineralColor = MajorColorDetector.Color.NONE;
//        RobotLog.dd(TAG, "Set qsize to get frames");
//        tracker.setFrameQueueSize(1);
//        RobotLog.dd(TAG, "Start LD sensing");
//        det.startSensing();
//
//        MajorColorDetector.Color leftJewelColor = MajorColorDetector.Color.NONE;
//
//        double jewelTimeout = 1.0;
//        ElapsedTime jtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        while(opModeIsActive()                                &&
//              leftJewelColor == MajorColorDetector.Color.NONE &&
//              jtimer.seconds() < jewelTimeout)
//        {
//            Bitmap rgbImage;
//            rgbImage = tracker.getLastCroppedImage();
//
//            if(rgbImage == null) continue;
//            det.setBitmap(rgbImage);
//            det.logDebug();
//            det.logTelemetry();
//            if(det instanceof MajorColorDetector)
//                leftJewelColor = ((MajorColorDetector) det).getMajorColor();
//
//            if(leftJewelColor == MajorColorDetector.Color.NONE)
//                sleep(100);
//        }
//
//        det.stopSensing();
//        tracker.setFrameQueueSize(0);
//
//        dashboard.displayPrintf(1, "JWL: " + leftJewelColor);
//
//        return leftJewelColor;
        return MajorColorDetector.Color.BLUE;
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
    private MajorColorDetector.Color mineralColor = MajorColorDetector.Color.NONE;

    private static Point2d curPos;
    private static double  curHdg;
    private double initHdg = 0.0;
    private boolean gyroReady;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean usePostTurn = true;

    private static PositionOption startPos = Route.StartPos.START_1;
    private static Field.Alliance alliance = Field.Alliance.RED;

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