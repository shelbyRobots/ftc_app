package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;

@SuppressWarnings("unused")
public class VvRoute extends Route
{
    @SuppressWarnings("ConstantConditions")
    @Override
    protected Vector<Point2d> initPoints()
    {
        Point2d start_pt = ASTART_PT;
        Point2d shoot_pt = ASHOOT_PT;

        if(startPos == StartPos.START_B_SWEEPER)
        {
            start_pt = BSTART_PT;
            shoot_pt = BSHOOT_PT;
        }
        else if(startPos == StartPos.START_R_PUSHER)
        {
            start_pt = RSTART_PT;
            shoot_pt = RSHOOT_PT;
        }

        Point2d park_pt = CTRPRKPT;
        if(parkChoice == ParkPos.CENTER_PARK &&
           startPos == StartPos.START_B_SWEEPER)
        {
            park_pt = BCTPRKPT;
        }
        if(parkChoice == ParkPos.CORNER_PARK)
        {
            park_pt = CRNPRKPT;
        }
        else if(parkChoice == ParkPos.DEFEND_PARK)
        {
            park_pt = DFNPRKPT;
        }

        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        ShelbyBot.DriveDir fwd = ShelbyBot.DriveDir.PUSHER;
        ShelbyBot.DriveDir rev = ShelbyBot.DriveDir.INTAKE;
        Segment.Action none   = Segment.Action.NOTHING;
        Segment.Action shoot  = Segment.Action.SHOOT;
        Segment.Action beacon = Segment.Action.FIND_BEACON;

        //SHOOT PTS
        points.add(start_pt);

        if(startPos == StartPos.START_B_SWEEPER)
        {
            addPoint(points, rev, 0.3, 1.00, Segment.TargetType.ENCODER, none, BPRSHT_PT);
        }

        if(startPos != StartPos.START_R_PUSHER)
        {
            addPoint(points, rev, 0.45, 1.00, Segment.TargetType.ENCODER, shoot, shoot_pt);
            //addPoint(points, fwd, 0.5, 1.00, Segment.TargetType.ENCODER, none, TMP_PT);
        }

        Segment.TargetType becnSegType = Segment.TargetType.COLOR;
        if(useFly2Light)
        {
            becnSegType = Segment.TargetType.ENCODER;
        }

        if(pushChoice == BeaconChoice.NEAR ||
           pushChoice == BeaconChoice.BOTH)
        {

            addPoint(points, fwd, 0.45,  1.00, becnSegType, beacon, BECN1_PT);

            if(startPos == StartPos.START_R_PUSHER)
            {
                addPoint(points, rev, 0.4, 1.00, Segment.TargetType.ENCODER, shoot, shoot_pt);
            }
        }

//        if(pushChoice == Field.BeaconChoice.FAR && startPos == Field.StartPos.START_B_SWEEPER)
//        {
//            addPoint(points, fwd, 0.5, 1.00, Segment.TargetType.ENCODER, none, B_MID_PT);
//        }

        if(pushChoice == BeaconChoice.FAR ||
           pushChoice == BeaconChoice.BOTH)
        {
            addPoint(points, fwd, 0.45, 1.00, becnSegType, beacon, BECN2_PT);
        }

        //PARK PTS
        ShelbyBot.DriveDir parkDir = rev;
        boolean parkFull = true;
        if(parkChoice == ParkPos.DEFEND_PARK)
        {
            addPoint(points, fwd, 0.6, 1.00, Segment.TargetType.ENCODER, none, DP1);
            //addPoint(points, fwd, 0.6, 1.00, Segment.TargetType.ENCODER, none, DP2);
        }
        else if(parkChoice == ParkPos.CENTER_PARK && parkFull)
        {
            if(startPos == StartPos.START_B_SWEEPER)
                addPoint(points, rev, 0.6, 1.00, Segment.TargetType.ENCODER, none, BPRCTRPT);
//            else
//                addPoint(points, rev, 0.6, 1.00, Segment.TargetType.ENCODER, none, PRECTRPT);
            parkDir = fwd;
        }

        addPoint(points, parkDir, 0.75, 1.00, Segment.TargetType.ENCODER, none, park_pt);

        return points;
    }

    public VvRoute(PositionOption startPos,
                   Field.Alliance alliance,
                   GoalOption pushChoice,
                   ParkPos parkChoice,
                   boolean useFly2Light)
    {
        super(startPos, alliance);
        this.parkChoice = parkChoice;
        this.pushChoice = pushChoice;
        this.useFly2Light = useFly2Light;
    }

    private Vector<Point2d> initBluePoints(Vector<Point2d> inpts)
    {
        Vector<Point2d> bpts = new Vector<>(inpts.size());

        for(Point2d rpt : inpts)
        {
            //Adjust distance to wall at BECN or SCAN Pts for BLUE
            //Note:  This is done in "red space", so adding a positive value to X
            //       will move farther from the wall
            if(rpt.getName().equals("BECN1") || rpt.getName().equals("BECN2") ||
               rpt.getName().equals("SCAN1") || rpt.getName().equals("SCAN2"))
            {
                rpt.setX(rpt.getX() + blueBecnScanAdjust);
            }

            if(rpt.getName().equals("DFNPRK"))
            {
                rpt.setX(-11.0);
            }

            bpts.add(convertRtoB(rpt));
        }
        return bpts;
    }

    public Segment[] initSegments(Vector<Point2d> pts)
    {
        Segment[] pathSegs = super.initSegments(pts);

        for(int s = 0; s < pathSegs.length - 1 ; s++)
        {
            Segment curSeg = pathSegs[s];
            String sname = curSeg.getName();

            if(sname.equals("SCAN1") || sname.equals("SCAN2") ||
               sname.equals("BECN1") || sname.equals("BECN2"))
            {
                double nfhdg = 180.0;
                if (alliance == Field.Alliance.BLUE) nfhdg = 90.0;
                curSeg.setPostTurn(nfhdg);
                RobotLog.ii("SJH", "Segment %s setting postTurn %4.2f", sname, nfhdg);
            }

            if(sname.equals("BSHOOT"))
            {
                if (alliance == Field.Alliance.BLUE )
                {
                    BASKET_PT = convertRtoB(BASKET_PT);
                }
                Segment aim = new Segment("AIM", curSeg.getTgtPt(), BASKET_PT);
                double nfhdg = aim.getFieldHeading();
                curSeg.setPostTurn(nfhdg);
                RobotLog.ii("SJH", "Segment %s setting postTurn %4.2f", sname, nfhdg);
            }
        }

        return pathSegs;
    }

    @SuppressWarnings("unused")
    private Segment getSegment(String name, Segment[] segs)
    {
        for (Segment pathSeg : segs)
        {
            String n = pathSeg.getName();
            if (n.equals(name)) return pathSeg;
        }
        return null;
    }

    protected Point2d convertRtoB(Point2d rpt)
    {
        double bx = -rpt.getY();
        double by = -rpt.getX();
        rpt.setX(bx);
        rpt.setY(by);
        return rpt;
    }
    
    public String toString()
    {
        StringBuilder sbldr = new StringBuilder();
        for (Segment segment : segments)
        {
            sbldr.append(segment.toString()).append("\n");
        }
        return sbldr.toString();
    }

    private static final double BECN1_Y = -12.0;
    private static final double FUDGE   =  0.0;
    private static final double BECN2_Y =  36.0 - FUDGE;

    //Note - these should really come from the robot.
    private static final double REAR_OFFSET = 9.0;
    private static final double FRNT_OFFSET = 9.0;

    private static final double ASTARTX =  -8.0;
    private static final double ASTARTY =  S_WALL + REAR_OFFSET + 0.5;
    private static final double AIMERY  =  ASTARTY + 3.0;
    private static final double ASHOOTY =  -27.0 - FRNT_OFFSET;
    private static final double AIMTOX  =  -10.5;
    private static final double AIMTOY  =  -10.5;

    private static final double BSTARTX =  12.0;
    private static final double BSHOOTX =    1.0;
    private static final double BSHOOTY =  -33.5;

    private static final double RSTARTX =  -24.0;
    private static final double RSTARTY =  ASTARTY;
    private static final double RSHOOTX = -25.0 - FRNT_OFFSET;
    private static final double RSHOOTY = BECN1_Y;

    private static final double blueBecnScanAdjust = -3.0;

    private static final double BPRCTRX =   1.0;
    private static final double BPRCTRY = -25.0;
    private static final double CTRPRKX = -10.0;
    private static final double CTRPRKY =   0.0;
    private static final double CRNPRKX = -45.0;
    private static final double CRNPRKY = -48.0;
    private static final double DFNPRKX =  -9.0;
    private static final double DFNPRKY =  48.0;
    private static final double BCTPRKX =   0.9;
    private static final double BCTPRKY =  -8.0;

    private static final double BECN_X  = -50.0;
    private static final double BECN2X  = -51.0;

    private static final double DFNPTHX =  -34; //24;
    private static final double DFNPTHY =  -30; //-24;

    private Point2d ASTART_PT = new Point2d("ASTART", ASTARTX, ASTARTY);
    private Point2d ASHOOT_PT = new Point2d("ASHOOT", ASTARTX, ASHOOTY);

    private Point2d BASKET_PT = new Point2d("BASKET", AIMTOX, AIMTOY);

    private Point2d BSTART_PT = new Point2d("BSTART", BSTARTX, ASTARTY);
    private Point2d BPRSHT_PT = new Point2d("BPRSHT", BSTARTX, AIMERY);
    private Point2d BSHOOT_PT = new Point2d("BSHOOT", BSHOOTX, BSHOOTY);

    private Point2d RSTART_PT = new Point2d("RSTART", RSTARTX, RSTARTY);
    private Point2d RSHOOT_PT = new Point2d("RSHOOT", RSHOOTX, RSHOOTY);

    private Point2d BECN1_PT = new Point2d("BECN1", BECN_X, BECN1_Y);
    private Point2d BECN2_PT = new Point2d("BECN2", BECN2X, BECN2_Y);

    private Point2d BPRCTRPT = new Point2d("BPRCTR", BPRCTRX, BPRCTRY);
    private Point2d CTRPRKPT = new Point2d("CTRPRK", CTRPRKX, CTRPRKY);
    private Point2d CRNPRKPT = new Point2d("CRNPRK", CRNPRKX, CRNPRKY);
    private Point2d BCTPRKPT = new Point2d("BCTPRK", BCTPRKX, BCTPRKY);
    private Point2d DFNPRKPT = new Point2d("DFNPRK", DFNPRKX, DFNPRKY);

    private Point2d DP1 = new Point2d("DP1", DFNPTHX, DFNPTHY);

    private boolean            useFly2Light;

    private GoalOption pushChoice;

    public enum StartPos implements PositionOption
    {
        START_1,
        START_2,
        START_B_SWEEPER,
        START_R_PUSHER
    }

    public enum ParkPos implements PositionOption
    {
        CENTER_PARK,
        CORNER_PARK,
        DEFEND_PARK
    }

    public enum BeaconChoice implements GoalOption
    {
        BOTH,
        NEAR,
        FAR,
        NONE
    }
}


