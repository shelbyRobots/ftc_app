package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;

@SuppressWarnings("unused")
public class RrPoints extends Points
{
    @SuppressWarnings("ConstantConditions")
    private Vector<Point2d> initPoints()
    {
        //AVA - add points to list here
        //Point2d start_pt = ASTART_PT;

        if(startPos == Field.StartPos.START_R_PUSHER)
        {
            //start_pt = RSTART_PT;
        }

        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        ShelbyBot.DriveDir fwd = ShelbyBot.DriveDir.PUSHER;
        ShelbyBot.DriveDir rev = ShelbyBot.DriveDir.SWEEPER;
        Segment.Action none   = Segment.Action.NOTHING;
        //Segment.Action beacon = Segment.Action.FIND_BEACON;

        //SHOOT PTS
        //points.add(start_pt);

        Segment.TargetType becnSegType = Segment.TargetType.COLOR;
        //if(useFly2Light)
        {
            becnSegType = Segment.TargetType.ENCODER;
        }

        //addPoint(points, fwd, 0.45,  1.00, becnSegType, beacon, BECN1_PT);
        //addPoint(points, fwd, 0.45, 1.00, becnSegType, beacon, BECN2_PT);

        ShelbyBot.DriveDir parkDir = rev;
        //addPoint(points, parkDir, 0.75, 1.00, Segment.TargetType.ENCODER, none, park_pt);

        return points;
    }

    public RrPoints(Field.StartPos startPos,
                    Field.Alliance alliance)
    {
        super(startPos, alliance);
    }

    private Vector<Point2d> initBluePoints(Vector<Point2d> inpts)
    {
        Vector<Point2d> bpts = new Vector<>(inpts.size());

        for(Point2d rpt : inpts)
        {
            bpts.add(convertRtoB(rpt));
        }
        return bpts;
    }

    Segment[] initSegments(Vector<Point2d> pts)
    {
        Segment[] pathSegs = super.initSegments(pts);

//        for(int s = 0; s < pathSegs.length - 1 ; s++)
//        {
//            Segment curSeg = pathSegs[s];
//            String sname = curSeg.getName();
//        }

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
        double bx =  rpt.getX();
        double by = -rpt.getY();
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

    //AVA - CREATE "helper" constants here, or in VV FIELDS if
    //they represent field points (like Red balance A)
    //Most should be in RrField - better way unless they
    //don't involve the field
//    private static final double BECN1_Y = -12.0;
//    private static final double FUDGE   =  0.0;
//    private static final double BECN2_Y =  36.0 - FUDGE;
//
//    private static final double ASTARTX =  -8.0;
//    private static final double ASTARTY =  S_WALL + REAR_OFFSET + 0.5;
//    private static final double AIMERY  =  ASTARTY + 3.0;
//    private static final double ASHOOTY =  -27.0 - FRNT_OFFSET;
//    private static final double AIMTOX  =  -10.5;
//    private static final double AIMTOY  =  -10.5;
//
//    private static final double BSTARTX =  12.0;
//    private static final double BSHOOTX =    1.0;
//    private static final double BSHOOTY =  -33.5;
//
//    private static final double RSTARTX =  -24.0;
//    private static final double RSTARTY =  ASTARTY;
//    private static final double RSHOOTX = -25.0 - FRNT_OFFSET;
//    private static final double RSHOOTY = BECN1_Y;

//    private static final double BPRCTRX =   1.0;
//    private static final double BPRCTRY = -25.0;
//    private static final double CTRPRKX = -10.0;
//    private static final double CTRPRKY =   0.0;
//    private static final double CRNPRKX = -45.0;
//    private static final double CRNPRKY = -48.0;
//    private static final double DFNPRKX =  -9.0;
//    private static final double DFNPRKY =  48.0;
//    private static final double BCTPRKX =   0.9;
//    private static final double BCTPRKY =  -8.0;
//
//    private static final double BECN_X  = -50.0;
//    private static final double BECN2X  = -51.0;
//
//    private static final double DFNPTHX =  -34; //24;
//    private static final double DFNPTHY =  -30; //-24;

    //AVA - Create points using either points, or X, Y constants
    //Give each point a good short name - like RBSA (Red Balance Stone A)
//    private Point2d ASTART_PT = new Point2d("ASTART", ASTARTX, ASTARTY);
//    private Point2d ASHOOT_PT = new Point2d("ASHOOT", ASTARTX, ASHOOTY);
//
//    private Point2d BASKET_PT = new Point2d("BASKET", AIMTOX, AIMTOY);
//
//    private Point2d BSTART_PT = new Point2d("BSTART", BSTARTX, ASTARTY);
//    private Point2d BPRSHT_PT = new Point2d("BPRSHT", BSTARTX, AIMERY);
//    private Point2d BSHOOT_PT = new Point2d("BSHOOT", BSHOOTX, BSHOOTY);
//
//    private Point2d RSTART_PT = new Point2d("RSTART", RSTARTX, RSTARTY);
//    private Point2d RSHOOT_PT = new Point2d("RSHOOT", RSHOOTX, RSHOOTY);
//
//    private Point2d BECN1_PT = new Point2d("BECN1", BECN_X, BECN1_Y);
//    private Point2d BECN2_PT = new Point2d("BECN2", BECN2X, BECN2_Y);
//
//    private Point2d BPRCTRPT = new Point2d("BPRCTR", BPRCTRX, BPRCTRY);
//    private Point2d CTRPRKPT = new Point2d("CTRPRK", CTRPRKX, CTRPRKY);
//    private Point2d CRNPRKPT = new Point2d("CRNPRK", CRNPRKX, CRNPRKY);
//    private Point2d BCTPRKPT = new Point2d("BCTPRK", BCTPRKX, BCTPRKY);
//    private Point2d DFNPRKPT = new Point2d("DFNPRK", DFNPRKX, DFNPRKY);
//
//    private Point2d DP1 = new Point2d("DP1", DFNPTHX, DFNPTHY);

}


