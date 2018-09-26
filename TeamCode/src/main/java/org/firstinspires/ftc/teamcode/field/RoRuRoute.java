package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;

@SuppressWarnings("unused")
public class RoRuRoute extends Route
{
    private static final String TAG = "SJH_RRP";

    @SuppressWarnings("ConstantConditions")
    @Override
    protected Vector<Point2d> initPoints()
    {
        RobotLog.dd(TAG, "In RrRoute initPoints alliance=%s startPos=%s",
                alliance, startPos);
        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        ShelbyBot.DriveDir fwd = ShelbyBot.DriveDir.INTAKE;
        ShelbyBot.DriveDir rev = ShelbyBot.DriveDir.PUSHER;

        Segment.Action none    = Segment.Action.NOTHING;
        Segment.Action scan    = Segment.Action.SCAN_IMAGE;
        Segment.Action align   = Segment.Action.SET_ALIGN;
        Segment.Action drop    = Segment.Action.DROP;
        Segment.Action push    = Segment.Action.PUSH;
        Segment.Action park    = Segment.Action.PARK;
        Segment.TargetType encType = Segment.TargetType.ENCODER;

        double fakeDist = 0.05;
        if(alliance == Field.Alliance.BLUE) fakeDist *= -1;
        //"Fake" points to help set initial orientation for zero length seg
        Point2d RLBs = new Point2d("RLBs", RrField.RLBS.getX() + fakeDist,
                                           RrField.RLBS.getY());

        Point2d RRBs = new Point2d("RRBs", RrField.RRBS.getX() + fakeDist,
                                           RrField.RRBS.getY());

        if(startPos == Route.StartPos.START_1) {
            points.add(RoRuField.RLLP);
            addPoint(points, fwd, 0.45, 1.00, encType, align, RoRuField.RLTP); //BS-BS(JP)
            addPoint(points, fwd, 0.35, 1.00, encType, scan,  RoRuField.RLTP); //BS-FP
            addPoint(points, fwd, 0.45, 1.00, encType, push,  RoRuField.RLM2); //FP-AP
            addPoint(points, rev, 0.40, 1.00, encType, none,  RoRuField.RLTP); //AP-DP
            addPoint(points, fwd, 0.60, 1.00, encType, none,  RoRuField.RLDT); //DP-AP
            addPoint(points, fwd, 0.70, 1.00, encType, drop,  RoRuField.RLDP); //AP-XP
            addPoint(points, rev, 0.60, 1.00, encType, park,  RoRuField.RLPP); //XP-PP
        }
        else if(startPos == Route.StartPos.START_2)
        {
            points.add(RoRuField.RRLP);
            addPoint(points, fwd, 0.45, 1.00, encType, align, RoRuField.RRTP); //BS-BS(JP)
            addPoint(points, fwd, 0.35, 1.00, encType, scan,  RoRuField.RRTP); //BS-FP
            addPoint(points, fwd, 0.45, 1.00, encType, push,  RoRuField.RRM2); //FP-AP
            addPoint(points, rev, 0.40, 1.00, encType, none,  RoRuField.RRTP); //AP-DP
            addPoint(points, fwd, 0.60, 1.00, encType, none,  RoRuField.RRDT); //DP-AP
            addPoint(points, fwd, 0.70, 1.00, encType, drop,  RoRuField.RRDP); //AP-XP
            addPoint(points, rev, 0.60, 1.00, encType, park,  RoRuField.RRPP); //XP-PP
        }

        return points;
    }

    public RoRuRoute(PositionOption startPos,
                     Field.Alliance alliance,
                     String robotName)
    {
        super(startPos, alliance);
        RrField.initField(robotName, 0);
    }

    protected Point2d convertRtoB(Point2d rpt)
    {
        double bx = -rpt.getX();
        double by = -rpt.getY();
        String nm = "B" + rpt.getName().substring(1);

        RobotLog.dd(TAG, "convertRtoB %s to %s", rpt.getName(), nm);

        return new Point2d(nm, bx, by);
    }
}