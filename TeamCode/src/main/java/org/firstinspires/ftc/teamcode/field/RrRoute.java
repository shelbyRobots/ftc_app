package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;

@SuppressWarnings("unused")
public class RrRoute extends Route
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
        ShelbyBot.DriveDir itk = ShelbyBot.DriveDir.INTAKE;
        ShelbyBot.DriveDir psr = ShelbyBot.DriveDir.PUSHER;
        if(alliance == Field.Alliance.BLUE)
        {
            fwd = ShelbyBot.DriveDir.PUSHER;
            //noinspection UnusedAssignment
            rev = ShelbyBot.DriveDir.INTAKE;
        }
        Segment.Action none    = Segment.Action.NOTHING;
        Segment.Action scan    = Segment.Action.SCAN_IMAGE;
        Segment.Action align   = Segment.Action.SET_ALIGN;
        Segment.Action key     = Segment.Action.SET_KEY;
        Segment.Action drop    = Segment.Action.DROP;
        Segment.Action grab    = Segment.Action.GRAB;
        Segment.Action pgrb    = Segment.Action.PREGRAB;
        Segment.Action rtct    = Segment.Action.RETRACT;
        Segment.Action escp    = Segment.Action.ESCAPE;
        Segment.Action thrw    = Segment.Action.THROW;
        Segment.TargetType encType = Segment.TargetType.ENCODER;

        double fakeDist = 0.05;
        if(alliance == Field.Alliance.BLUE) fakeDist *= -1;
        //"Fake" points to help set initial orientation for zero length seg
        Point2d RLBs = new Point2d("RLBs", RrField.RLBS.getX() + fakeDist,
                                           RrField.RLBS.getY());

        Point2d RRBs = new Point2d("RRBs", RrField.RRBS.getX() + fakeDist,
                                           RrField.RRBS.getY());

        if(startPos == Field.StartPos.START_1)
        {
            points.add(RrField.RLBS);
            addPoint(points, itk, 0.45, 1.00, encType, scan,  RLBs);         //BS-BS(JP)
            addPoint(points, fwd, 0.35, 1.00, encType, align, RrField.RLFP); //BS-FP
            addPoint(points, fwd, 0.45, 1.00, encType, key,   RrField.RLFP); //FP-AP
            addPoint(points, itk, 0.40, 1.00, encType, drop,  RrField.RLDC); //AP-DP
            addPoint(points, psr, 0.60, 1.00, encType, pgrb,  RrField.RLFP); //DP-AP
            addPoint(points, itk, 0.70, 1.00, encType, none,  RrField.RLXP); //AP-XP
            addPoint(points, itk, 0.60, 1.00, encType, grab,  RrField.RLPP); //XP-PP
            addPoint(points, psr, 0.70, 1.00, encType, none,  RrField.RLXP); //PP-XP
            addPoint(points, psr, 0.70, 1.00, encType, key,   RrField.RLFP); //XP-AP
            addPoint(points, itk, 0.40, 1.00, encType, drop,  RrField.RLTT); //AP-DP
        }
        else if(startPos == Field.StartPos.START_2)
        {
            points.add(RrField.RRBS);
            addPoint(points, itk, 0.45, 1.00, encType, scan,  RRBs);         //BS-BS(JP)
            addPoint(points, fwd, 0.35, 1.00, encType, align, RrField.RRFP); //BS-FP
            addPoint(points, itk, 0.45, 1.00, encType, key,   RrField.RRFP); //FP-AP
            addPoint(points, itk, 0.40, 1.00, encType, drop,  RrField.RRDC); //AP-DP
            addPoint(points, psr, 0.60, 1.00, encType, pgrb,  RrField.RRFP); //DP-AP
            addPoint(points, itk, 0.70, 1.00, encType, none,  RrField.RRXP); //AP-XP
            addPoint(points, itk, 0.60, 1.00, encType, grab,  RrField.RRPP); //XP-PP
            addPoint(points, psr, 0.70, 1.00, encType, none,  RrField.RRXP); //PP-XP
            addPoint(points, psr, 0.70, 1.00, encType, key,   RrField.RRFP); //XP-AP
            addPoint(points, itk, 0.40, 1.00, encType, drop,  RrField.RRTT); //AP-DP
        }

        ShelbyBot.DriveDir parkDir = fwd;
        //addPoint(points, parkDir, 0.75, 1.00, Segment.TargetType.ENCODER, none, park_pt);

        return points;
    }

    public RrRoute(Field.StartPos startPos,
                   Field.Alliance alliance,
                   String robotName,
                   double glyphOff)
    {
        super(startPos, alliance);
        RrField.initField(robotName, glyphOff);
    }

    protected Point2d convertRtoB(Point2d rpt)
    {
        double bx =  rpt.getX();
        double by = Math.abs(rpt.getY());
        String nm = "B" + rpt.getName().substring(1);

        RobotLog.dd(TAG, "convertRtoB %s to %s", rpt.getName(), nm);

        return new Point2d(nm, bx, by);
    }
}