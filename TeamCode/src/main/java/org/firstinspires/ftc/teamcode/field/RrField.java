package org.firstinspires.ftc.teamcode.field;


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
    //Red Points > Left
    static final Point2d RLBS = new Point2d("RLBS", -48.0,  -48.0);
    static final Point2d RLJB = new Point2d("RLJB", -48.0,  -68.0);
    static final Point2d RLCL = new Point2d("RLCL", -4.37,  -70.0);
    static final Point2d RLCC = new Point2d("RLCC", -12.0,  -70.0);
    static final Point2d RLCR = new Point2d("RLCR", -19.63, -70.0);
    static final Point2d RLRR = new Point2d("RLRR", -68.0,  -68.0);

    static final Point2d RLTT = new Point2d("RLTT", -12.0,  -48.0);

    static final Point2d RLPP = new Point2d("RLPP", -12.0,  -12.0);

    //Red Points > Right
    static final Point2d RRBS = new Point2d("RRBS",  24.0,  -48.0);
    static final Point2d RRJB = new Point2d("RRJB",  24.0,  -68.0);
    static final Point2d RRCL = new Point2d("RRCL",  70.0,  -28.37);
    static final Point2d RRCC = new Point2d("RRCC",  70.0,  -36.0);
    static final Point2d RRCR = new Point2d("RRCR",  70.0,  -43.63);
    static final Point2d RRRR = new Point2d("RRRR",  68.0,  -68.0);
    static final Point2d RARZ = new Point2d("RARZ", -72.0,  -12.0);

    static final Point2d RRTT = new Point2d("RRTT",  48.0,  -36.0);

    static final Point2d RRPP = new Point2d("RRPP",   0.0,  -12.0);

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