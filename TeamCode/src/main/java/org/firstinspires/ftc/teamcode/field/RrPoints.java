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
    @Override
    protected Vector<Point2d> initPoints()
    {
        Point2d start_pt = RrField.RLBS;

        if(startPos == Field.StartPos.START_2)
        {
            start_pt = RrField.RRBS;
        }

        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        ShelbyBot.DriveDir fwd = ShelbyBot.DriveDir.PUSHER;
        ShelbyBot.DriveDir rev = ShelbyBot.DriveDir.SWEEPER;
        Segment.Action none    = Segment.Action.NOTHING;
        Segment.Action scan    = Segment.Action.SCAN_IMAGE;
        Segment.TargetType defSegType = Segment.TargetType.ENCODER;

        points.add(start_pt);

        addPoint(points, fwd, 0.45,  1.00, defSegType, scan, RrField.RLBS);
        addPoint(points, fwd, 0.45,  1.00, defSegType, none, RrField.RLTT);
        addPoint(points, fwd, 0.45,  1.00, defSegType, none, RrField.RLPP);
        addPoint(points, fwd, 0.45,  1.00, defSegType, none, RrField.RLTT);

        ShelbyBot.DriveDir parkDir = fwd;
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
        //This override will only be needed if we need to do something special to blue points
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
}


