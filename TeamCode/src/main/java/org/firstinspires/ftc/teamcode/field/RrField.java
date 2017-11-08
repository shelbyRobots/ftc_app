package org.firstinspires.ftc.teamcode.field;


import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Units;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

/*
 * Created by Ryan on 9/27/2017.
 */

@SuppressWarnings("unused")
public class RrField extends Field
{
    //Point naming key:
    //1st char: R=Red, B=Blue
    //2nd char: L=Left start, R=Right start (viewed from red side - along field X)
    //3rd-4th chars:
    //          BS=Balance Stone (starting points)
    //          JB=JewelBox
    //          CL=Crypto Left
    //          CC=Crypto Center
    //          CR=Crypto Right
    //          DL=Drop Left
    //          DC=Drop Center
    //          DR=Drop Right
    //          FP=Floor Point (point on floor just clear of balance stone)
    //          TT=TriangleTip
    //          PP=PitPoint
    //          RR=RelicRecovery
    //          RZ=RelicZone

    static final double BOT_2_GLYPH = 14.0;
    static Point2d calcDropPt(String dName, Point2d strt, Point2d end)
    {
        Point2d dropPt = new Point2d(dName, 0.0, 0.0);
        double dSegLen = strt.distance(end);
        double pct = (dSegLen - BOT_2_GLYPH)/dSegLen;
        double sX = strt.getX();
        double sY = strt.getY();
        double eX = end.getX();
        double eY = end.getY();

        dropPt.setX(sX + (eX-sX)*pct);
        dropPt.setY(sY + (eY-sY)*pct);
        return  dropPt;
    }

    static final double CBOX_GAP  = 7.625;
    static final double CBOX_LCTR = -12.0;
    static final double CBOX_RCTR = -36.0;
    //Red Points > Left
    static final Point2d RLBS = new Point2d("RLBS", -48.0,  -48.0);
    static final Point2d RLJB = new Point2d("RLJB", -48.0,  -68.0);
    static final Point2d RLFP = new Point2d("RLFP", -24.0, -48.0);
    static final Point2d RLCL = new Point2d("RLCL", CBOX_LCTR + CBOX_GAP, -67.0);
    static final Point2d RLCC = new Point2d("RLCC", CBOX_LCTR,            -67.0);
    static final Point2d RLCR = new Point2d("RLCR", CBOX_LCTR - CBOX_GAP, -67.0);
    static final Point2d RLDL = calcDropPt("RLDL", RLFP, RLCL);
    static final Point2d RLDC = calcDropPt("RLDC", RLFP, RLCC);
    static final Point2d RLDR = calcDropPt("RLDR", RLFP, RLCR);
    static final Point2d RLRR = new Point2d("RLRR", -68.0,  -68.0);

    static final Point2d RLTT = new Point2d("RLTT", -12.0,  -48.0);
    static final Point2d RLPP = new Point2d("RLPP", -12.0,  -12.0);

    //Red Points > Right
    static final Point2d RRBS = new Point2d("RRBS",  24.0,  -48.0);
    static final Point2d RRJB = new Point2d("RRJB",  24.0,  -68.0);
    static final Point2d RRFP = new Point2d("RRFP",  48.0,  -48.0);
    static final Point2d RRCL = new Point2d("RRCL",  67.0,  CBOX_RCTR + CBOX_GAP);
    static final Point2d RRCC = new Point2d("RRCC",  67.0,  CBOX_RCTR);
    static final Point2d RRCR = new Point2d("RRCR",  67.0,  CBOX_RCTR - CBOX_GAP);
    static final Point2d RRDL = calcDropPt("RRDL", RRFP, RRCL);
    static final Point2d RRDC = calcDropPt("RRDC", RRFP, RRCC);
    static final Point2d RRDR = calcDropPt("RRDR", RRFP, RRCR);
    static final Point2d RRRR = new Point2d("RRRR",  68.0,  -68.0);

    static final Point2d RRTT = new Point2d("RRTT",  48.0,  -36.0);
    static final Point2d RRPP = new Point2d("RRPP",   0.0,  -12.0);
    static final Point2d RRXP = new Point2d("RRXP",  48.0,  -24.0);

    static final Point2d RARZ = new Point2d("RARZ", -72.0,  -12.0);

    public static Point2d getDropPt(Alliance alliance, StartPos startPos, RelicRecoveryVuMark key)
    {
        Point2d retPt = null;
        switch (startPos)
        {
            case START_1:
                switch (key)
                {
                    case LEFT:   retPt = RLDL; break;
                    case CENTER: retPt = RLDC; break;
                    case RIGHT:  retPt = RLDR; break;
                    case UNKNOWN: retPt = RLDC; break;
                }
                break;
            case START_2:
                switch (key)
                {
                    case LEFT:   retPt = RRDL; break;
                    case CENTER: retPt = RRDC; break;
                    case RIGHT:  retPt = RRDR; break;
                    case UNKNOWN: retPt = RRDC; break;
                }
        }

        if(alliance == Alliance.BLUE)
        {
            retPt.setY(-retPt.getY());
        }

        return retPt;
    }

    //The jewels will lie roughly along the line of the tracker
    //picture bottom.  The center of the left jewel
    //will be ~ at the lower right corner.
    //The left edge of the left jewel will be one jewel radius
    //"left" of the corner.  The right edge of the right jewel
    //will be 1 jewel radius + 6" beyond the end of the line.
    //The jewels will be +/- radius perpindicular to the line.
    //According to RR manual part 2, the bottom of the image is
    //1.5" above the floor, and tiles are 5/8" thick.
    //So, bottom of jewel sitting on tiles will be 5/8"
    //above floor.  1.5" trackable bottom - 5/8" jewel bottom
    //= -7/8"

    private static final boolean bothJewels = false;
    private static final double tileToTrackableBottom = (1.5 - 0.625) * Units.MM_PER_INCH;
    private static final double jewelRadius   = (3.75/2.0) * Units.MM_PER_INCH;
    private static final double jewelCtrToCtr = 6.0        * Units.MM_PER_INCH;
    private static final double jewelBottom   = -Field.target_height /2 - tileToTrackableBottom ;
    private static final double jewelTop      =  jewelBottom + 2*jewelRadius;
    private static final double jewelLeft     =  Field.target_width /2 - jewelRadius;
    private static double jewelRight    =  jewelLeft + 2*jewelRadius;

    public static List<Point2d> getTrackableRelativeCropCorners()
    {
        if(bothJewels) jewelRight += jewelCtrToCtr;

        List<Point2d> jewelCorners = new ArrayList<>(4);
        jewelCorners.add(new Point2d(jewelLeft,  jewelTop));
        jewelCorners.add(new Point2d(jewelRight, jewelTop));
        jewelCorners.add(new Point2d(jewelRight, jewelBottom));
        jewelCorners.add(new Point2d(jewelLeft,  jewelBottom));

        return jewelCorners;
    }
}