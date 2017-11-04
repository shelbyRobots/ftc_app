package org.firstinspires.ftc.teamcode.field;

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
        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        ShelbyBot.DriveDir fwd = ShelbyBot.DriveDir.SWEEPER;
        ShelbyBot.DriveDir rev = ShelbyBot.DriveDir.PUSHER;
        if(alliance == Field.Alliance.BLUE)
        {
            fwd = ShelbyBot.DriveDir.PUSHER;
            rev = ShelbyBot.DriveDir.SWEEPER;
        }
        Segment.Action none    = Segment.Action.NOTHING;
        Segment.Action scan    = Segment.Action.SCAN_IMAGE;
        Segment.Action key     = Segment.Action.SET_KEY;
        Segment.Action drop    = Segment.Action.DROP;
        Segment.TargetType encType = Segment.TargetType.ENCODER;

        if(startPos == Field.StartPos.START_1)
        {
            points.add(RrField.RLBS);
            addPoint(points, fwd, 0.45, 1.00, encType, scan, RrField.RLBS);
            addPoint(points, fwd, 0.30, 1.00, encType, key,  RrField.RLFP);
            addPoint(points, fwd, 0.45, 1.00, encType, drop, RrField.RLDC);
            addPoint(points, rev, 0.40, 1.00, encType, none, RrField.RLFP);
            addPoint(points, fwd, 0.60, 1.00, encType, none, RrField.RLPP);
            addPoint(points, fwd, 0.60, 1.00, encType, none, RrField.RLTT);
        }
        else if(startPos == Field.StartPos.START_2)
        {
            points.add(RrField.RRBS);
            addPoint(points, fwd, 0.45, 1.00, encType, scan, RrField.RRBS);
            addPoint(points, fwd, 0.30, 1.00, encType, key,  RrField.RRFP);
            addPoint(points, fwd, 0.30, 1.00, encType, drop, RrField.RRDC);
            addPoint(points, rev, 0.40, 1.00, encType, none, RrField.RRFP);
            addPoint(points, fwd, 0.60, 1.00, encType, none, RrField.RRPP);
            addPoint(points, fwd, 0.60, 1.00, encType, none, RrField.RRTT);
        }

        ShelbyBot.DriveDir parkDir = fwd;
        //addPoint(points, parkDir, 0.75, 1.00, Segment.TargetType.ENCODER, none, park_pt);

        return points;
    }

    public RrPoints(Field.StartPos startPos,
                    Field.Alliance alliance)
    {
        super(startPos, alliance);
    }

    protected Point2d convertRtoB(Point2d rpt)
    {
        double bx =  rpt.getX();
        double by = -rpt.getY();
        rpt.setX(bx);
        rpt.setY(by);
        return rpt;
    }
}


