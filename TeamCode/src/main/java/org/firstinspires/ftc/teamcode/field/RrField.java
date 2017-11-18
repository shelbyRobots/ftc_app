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

@SuppressWarnings({"unused", "WeakerAccess"})
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

    private static double BOT_2_GLYPH = 12.0; //GTO2
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

    public static final double CXY = 65.0;
    public static final double BSY = 47.5;
    private static final double BS1X = -BSY;
    public static final double BS2X = 24.0;

    public static final double BS_CLEAR = 25.0;

    private static final double CBOX_GAP  = 7.5;
    private static final double CBOX_LCTR = -12.0;
    private static final double CBOX_RCTR = -36.0;

    private static double BLCLXT = 0.0;
    private static double BLCCXT = 0.0;
    private static double BLCRXT = 0.0;
    private static double BRCLXT = 0.0;
    private static double BRCCXT = 0.0;
    private static double BRCRXT = 0.0;
    private static double RLCLXT = 0.0;
    private static double RLCCXT = 0.0;
    private static double RLCRXT = 0.0;
    private static double RRCLXT = 0.0;
    private static double RRCCXT = 0.0;
    private static double RRCRXT = 0.0;

    private static double BLCLYT = 0.0;
    private static double BLCCYT = 0.0;
    private static double BLCRYT = 0.0;
    private static double BRCLYT = 0.0;
    private static double BRCCYT = 0.0;
    private static double BRCRYT = -2.0;
    private static double RLCLYT = 0.0;
    private static double RLCCYT = 0.0;
    private static double RLCRYT = 0.0;
    private static double RRCLYT = 1.5;
    private static double RRCCYT = 1.0;
    private static double RRCRYT = 2.5;

    static void initField(String robotName)
    {
        if (robotName.equals("GTO1"))
        {
            BOT_2_GLYPH = 12.0;

            BLCLXT = 0.0;
            BLCCXT = 0.0;
            BLCRXT = 0.0;
            BRCLXT = 0.0;
            BRCCXT = 0.0;
            BRCRXT = 0.0;
            RLCLXT = 0.0;
            RLCCXT = 0.0;
            RLCRXT = 0.0;
            RRCLXT = 0.0;
            RRCCXT = 0.0;
            RRCRXT = 0.0;

            BLCLYT = 0.0;
            BLCCYT = 0.0;
            BLCRYT = 0.0;
            BRCLYT = 0.0;
            BRCCYT = 0.0;
            BRCRYT = -2.0;
            RLCLYT = 0.0;
            RLCCYT = 0.0;
            RLCRYT = 0.0;
            RRCLYT = 1.5;
            RRCCYT = 1.5;
            RRCRYT = 2.5;
        }
        else if (robotName.equals("GTO2"))
        {
            BOT_2_GLYPH = 12.0;

            BLCLXT = 0.0;
            BLCCXT = 0.0;
            BLCRXT = 0.0;
            BRCLXT = 0.0;
            BRCCXT = 0.0;
            BRCRXT = 0.0;
            RLCLXT = 0.0;
            RLCCXT = 0.0;
            RLCRXT = 0.0;
            RRCLXT = 0.0;
            RRCCXT = 0.0;
            RRCRXT = 0.0;

            BLCLYT = 0.0;
            BLCCYT = 0.0;
            BLCRYT = 0.0;
            BRCLYT = 0.0;
            BRCCYT = 0.0;
            BRCRYT = -2.0;
            RLCLYT = 0.0;
            RLCCYT = 0.0;
            RLCRYT = 0.0;
            RRCLYT = 1.5;
            RRCCYT = 1.0;
            RRCRYT = 2.5;
        }
        else if (robotName.equals("MEC"))
        {
            BOT_2_GLYPH = 12.0;

            BLCLXT = 0.0;
            BLCCXT = 0.0;
            BLCRXT = 0.0;
            BRCLXT = 0.0;
            BRCCXT = 0.0;
            BRCRXT = 0.0;
            RLCLXT = 0.0;
            RLCCXT = 0.0;
            RLCRXT = 0.0;
            RRCLXT = 0.0;
            RRCCXT = 0.0;
            RRCRXT = 0.0;

            BLCLYT = 0.0;
            BLCCYT = 0.0;
            BLCRYT = 0.0;
            BRCLYT = 0.0;
            BRCCYT = 0.0;
            BRCRYT = -2.0;
            RLCLYT = 0.0;
            RLCCYT = 0.0;
            RLCRYT = 0.0;
            RRCLYT = 1.5;
            RRCCYT = 1.0;
            RRCRYT = 2.5;
        }
    }

    //3-dimensional array of crypto box points [alliance][start][key]
    //allows inidividual adjustment
    private static final Point2d CPTS[][][] =
    {
        {
            {
                new Point2d("BLCL", BLCLXT + CBOX_LCTR - CBOX_GAP,  BLCLYT + CXY), //BLUE-LEFT-LEFT
                new Point2d("BLCC", BLCCXT + CBOX_LCTR,             BLCCYT + CXY), //BLUE-LEFT-CENTER
                new Point2d("BLCR", BLCRXT + CBOX_LCTR + CBOX_GAP,  BLCRYT + CXY)  //BLUE-LEFT-RIGHT
            },
            {
                new Point2d("BRCL", BRCLXT + CXY,  BRCLYT + -CBOX_RCTR + CBOX_GAP), //BLUE-RIGHT-LEFT
                new Point2d("BRCC", BRCCXT + CXY,  BRCCYT + -CBOX_RCTR),            //BLUE-RIGHT-CENTER
                new Point2d("BRCR", BRCRXT + CXY,  BRCRYT + (-CBOX_RCTR - CBOX_GAP))  //BLUE-RIGHT-RIGHT
            },
        },
        {
            {
                new Point2d("RLCL", RLCLXT + CBOX_LCTR + CBOX_GAP, RLCLYT + -CXY), //RED-LEFT-LEFT
                new Point2d("RLCC", RLCCXT + CBOX_LCTR           , RLCCYT + -CXY), //RED-LEFT-CENTER
                new Point2d("RLCR", RLCRXT + CBOX_LCTR - CBOX_GAP, RLCRYT + -CXY)  //RED-LEFT-RIGHT
            },
            {
                new Point2d("RRCL", RRCLXT + CXY, RRCLYT + CBOX_RCTR + CBOX_GAP), //RED-RIGHT-LEFT
                new Point2d("RRCC", RRCCXT + CXY, RRCCYT + CBOX_RCTR),            //RED-RIGHT-CENTER
                new Point2d("RRCR", RRCRXT + CXY, RRCRYT + (CBOX_RCTR - CBOX_GAP))  //RED-RIGHT-RIGHT
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
                new Point2d("BRAL",  BS2X + BS_CLEAR,  -CBOX_RCTR + CBOX_GAP), //BLUE-RIGHT-LEFT
                new Point2d("BRAC",  BS2X + BS_CLEAR,  -CBOX_RCTR),            //BLUE-RIGHT-CENTER
                new Point2d("BRAR",  BS2X + BS_CLEAR,  -CBOX_RCTR - CBOX_GAP)  //BLUE-RIGHT-RIGHT
            },
        },
        {
            {
                new Point2d("RLAL", CBOX_LCTR + CBOX_GAP, -BSY), //RED-LEFT-LEFT
                new Point2d("RLAC", CBOX_LCTR           , -BSY), //RED-LEFT-CENTER
                new Point2d("RLAR", CBOX_LCTR - CBOX_GAP, -BSY)  //RED-LEFT-RIGHT
            },
            {
                new Point2d("RRAL",  BS2X + BS_CLEAR, CBOX_RCTR + CBOX_GAP), //RED-RIGHT-LEFT
                new Point2d("RRAC",  BS2X + BS_CLEAR, CBOX_RCTR),            //RED-RIGHT-CENTER
                new Point2d("RRAR",  BS2X + BS_CLEAR, CBOX_RCTR - CBOX_GAP)  //RED-RIGHT-RIGHT
            },
        }
    };

    //2-dimensional array of floor points [alliance][start]
    //allows inidividual adjustment
    private static final Point2d FPTS[][] =
    {
        {
            new Point2d("BLFP",  BS1X + BS_CLEAR,  BSY),
            new Point2d("BRFP",  BS2X + BS_CLEAR,  BSY),
        },
        {
            new Point2d("RLFP",  BS1X + BS_CLEAR, -BSY),
            new Point2d("RRFP",  BS2X + BS_CLEAR, -BSY),
        }
    };

    //Red Points > Left
    static final Point2d RLBS = new Point2d("RLBS", BS1X,  -BSY);
    static final Point2d RLJB = new Point2d("RLJB", -BSY,  -68.0);
    static final Point2d RLFP = new Point2d("RLFP", BS1X + BS_CLEAR, -BSY);
    static final Point2d RLDC = calcDropPt("RLDC", RLFP, CPTS[RED][STRT1][CNTR]);
    static final Point2d RLRR = new Point2d("RLRR", -68.0,  -68.0);

    static final Point2d RLTT = new Point2d("RLTT", CBOX_LCTR,  -BSY);
    static final Point2d RLPP = new Point2d("RLPP", CBOX_LCTR,  -30.0);
    static final Point2d RLXP = new Point2d("RLXP", CBOX_LCTR,  -37.0);

    //Red Points > Right
    static final Point2d RRBS = new Point2d("RRBS",  BS2X,  -BSY);
    static final Point2d RRJB = new Point2d("RRJB",  BS2X,  -68.0);
    static final Point2d RRFP = new Point2d("RRFP",  BS2X + BS_CLEAR,  -BSY);
    static final Point2d RRDC = calcDropPt("RRDC", RRFP, CPTS[RED][STRT2][CNTR]);
    static final Point2d RRRR = new Point2d("RRRR",  68.0,  -68.0);

    static final Point2d RRTT = new Point2d("RRTT",  BS2X + BS_CLEAR,  -36.0);
    static final Point2d RRPP = new Point2d("RRPP",  14.0,  -14.0);
    static final Point2d RRXP = new Point2d("RRXP",  BS2X + BS_CLEAR,  -24.0);

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
