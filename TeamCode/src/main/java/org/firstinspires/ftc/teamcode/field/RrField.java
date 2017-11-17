package org.firstinspires.ftc.teamcode.field;


import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Units;

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

    private static final String TAG = " SJH_RFD";

    private static final double BOT_2_GLYPH = 12.0; //GTO2
    private static Point2d calcDropPt(String dName, Point2d strt, Point2d end)
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

        RobotLog.dd(TAG, "OrigLen %.2f BOT_2_GLYPH %.2f DropPt %s",
                dSegLen, BOT_2_GLYPH, dropPt);
        return  dropPt;
    }

    private static final int BLUE  = 0;
    private static final int RED   = 1;
    private static final int STRT1 = 0;
    private static final int STRT2 = 1;
    private static final int LEFT  = 0;
    private static final int CNTR  = 1;
    private static final int RGHT  = 2;

    private static final double CXY = 65.0;
    private static final double BSY = 47.5;

    private static final double CBOX_GAP  = 7.5;
    private static final double CBOX_LCTR = -12.0;
    private static final double CBOX_RCTR = -36.0;

    //3-dimensional array of crypto box points [alliance][start][key]
    //allows inidividual adjustment
    private static final Point2d CPTS[][][] =
    {
        {
            {
                new Point2d("BLCL", CBOX_LCTR - CBOX_GAP,  CXY), //BLUE-LEFT-LEFT
                new Point2d("BLCC", CBOX_LCTR,             CXY), //BLUE-LEFT-CENTER
                new Point2d("BLCR", CBOX_LCTR + CBOX_GAP,  CXY)  //BLUE-LEFT-RIGHT
            },
            {
                new Point2d("BRCL", CXY,  -CBOX_RCTR + CBOX_GAP), //BLUE-RIGHT-LEFT
                new Point2d("BRCC", CXY,  -CBOX_RCTR),            //BLUE-RIGHT-CENTER
                new Point2d("BRCR", CXY,  (-CBOX_RCTR - CBOX_GAP) -2)  //BLUE-RIGHT-RIGHT
            },
        },
        {
            {
                new Point2d("RLCL", CBOX_LCTR + CBOX_GAP, -CXY), //RED-LEFT-LEFT
                new Point2d("RLCC", CBOX_LCTR           , -CXY), //RED-LEFT-CENTER
                new Point2d("RLCR", CBOX_LCTR - CBOX_GAP, -CXY)  //RED-LEFT-RIGHT
            },
            {
                new Point2d("RRCL", CXY, CBOX_RCTR + CBOX_GAP + 2.5), //RED-RIGHT-LEFT
                new Point2d("RRCC", CXY, CBOX_RCTR + 3),            //RED-RIGHT-CENTER
                new Point2d("RRCR", CXY, (CBOX_RCTR - CBOX_GAP) + 2)  //RED-RIGHT-RIGHT
            },
        }
    };

    //3-dimensional array of align points [alliance][start][key]
    //allows inidividual adjustment
    private static final Point2d APTS[][][] =
    {
        {
            {
                new Point2d("BLAL", CBOX_LCTR - CBOX_GAP,  BSY), //BLUE-LEFT-LEFT
                new Point2d("BLAC", CBOX_LCTR,             BSY), //BLUE-LEFT-CENTER
                new Point2d("BLAR", CBOX_LCTR + CBOX_GAP,  BSY)  //BLUE-LEFT-RIGHT
            },
            {
                new Point2d("BRAL",  49.0,  -CBOX_RCTR + CBOX_GAP), //BLUE-RIGHT-LEFT
                new Point2d("BRAC",  49.0,  -CBOX_RCTR),            //BLUE-RIGHT-CENTER
                new Point2d("BRAR",  49.0,  -CBOX_RCTR - CBOX_GAP)  //BLUE-RIGHT-RIGHT
            },
        },
        {
            {
                new Point2d("RLAL", CBOX_LCTR + CBOX_GAP, -BSY), //RED-LEFT-LEFT
                new Point2d("RLAC", CBOX_LCTR           , -BSY), //RED-LEFT-CENTER
                new Point2d("RLAR", CBOX_LCTR - CBOX_GAP, -BSY)  //RED-LEFT-RIGHT
            },
            {
                new Point2d("RRAL",  49.0, CBOX_RCTR + CBOX_GAP), //RED-RIGHT-LEFT
                new Point2d("RRAC",  49.0, CBOX_RCTR),            //RED-RIGHT-CENTER
                new Point2d("RRAR",  49.0, CBOX_RCTR - CBOX_GAP)  //RED-RIGHT-RIGHT
            },
        }
    };

    //2-dimensional array of floor points [alliance][start]
    //allows inidividual adjustment
    private static final Point2d FPTS[][] =
    {
        {
            new Point2d("BLFP", -23.0,  BSY),
            new Point2d("BRFP",  49.0,  BSY),
        },
        {
            new Point2d("RLFP", -23.0, -BSY),
            new Point2d("RRFP",  49.0, -BSY),
        }
    };

    //Red Points > Left
    static final Point2d RLBS = new Point2d("RLBS", -BSY,  -BSY);
    static final Point2d RLJB = new Point2d("RLJB", -BSY,  -68.0);
    static final Point2d RLFP = new Point2d("RLFP", -23.0, -BSY);
    static final Point2d RLDC = calcDropPt("RLDC", RLFP, CPTS[RED][STRT1][CNTR]);
    static final Point2d RLRR = new Point2d("RLRR", -68.0,  -68.0);

    static final Point2d RLTT = new Point2d("RLTT", -12.0,  -BSY);
    static final Point2d RLPP = new Point2d("RLPP", -12.0,  -30.0);
    static final Point2d RLXP = new Point2d("RLXP", -12.0,  -37.0);

    //Red Points > Right
    static final Point2d RRBS = new Point2d("RRBS",  24.0,  -BSY);
    static final Point2d RRJB = new Point2d("RRJB",  24.0,  -68.0);
    static final Point2d RRFP = new Point2d("RRFP",  49.0,  -BSY);
    static final Point2d RRDC = calcDropPt("RRDC", RRFP, CPTS[RED][STRT2][CNTR]);
    static final Point2d RRRR = new Point2d("RRRR",  68.0,  -68.0);

    static final Point2d RRTT = new Point2d("RRTT",  49.0,  -36.0);
    static final Point2d RRPP = new Point2d("RRPP",  14.0,  -14.0);
    static final Point2d RRXP = new Point2d("RRXP",  49.0,  -24.0);

    static final Point2d RARZ = new Point2d("RARZ", -72.0,  -12.0);


    public static Point2d getAlignPt(Alliance alliance,
                                     StartPos startPos,
                                     RelicRecoveryVuMark key,
                                     boolean useFP)
    {
        int alnc = (alliance == Alliance.RED) ? RED : BLUE;
        int strt = (startPos == StartPos.START_1) ? STRT1 : STRT2;
        int ckey = CNTR;

        switch (key)
        {
            case LEFT:    ckey = LEFT; break;
            case CENTER:  ckey = CNTR; break;
            case UNKNOWN: ckey = CNTR; break;
            case RIGHT:   ckey = RGHT; break;
        }

        Point2d apt = APTS[alnc][strt][ckey];
        if(useFP) apt = FPTS[alnc][strt];

        return apt;
    }

    public static Point2d getDropPt(Alliance alliance,
                                    StartPos startPos,
                                    RelicRecoveryVuMark key,
                                    boolean useFP)
    {
        int alnc = (alliance == Alliance.RED) ? RED : BLUE;
        int strt = (startPos == StartPos.START_1) ? STRT1 : STRT2;
        int ckey = CNTR;

        switch (key)
        {
            case LEFT:    ckey = LEFT; break;
            case CENTER:  ckey = CNTR; break;
            case UNKNOWN: ckey = CNTR; break;
            case RIGHT:   ckey = RGHT; break;
        }

        Point2d spt = getAlignPt(alliance, startPos, key, useFP);
        Point2d ept = CPTS[alnc][strt][ckey];

        String pName = ept.getName().substring(0,2) + "D" + ept.getName().substring(3);

        RobotLog.dd(TAG, "getDropPt %s start %s end %s", pName, spt, ept);
        return calcDropPt(pName, spt, ept);
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