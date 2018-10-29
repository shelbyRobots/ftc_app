package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.image.MineralDetector;
import org.firstinspires.ftc.teamcode.util.Point2d;

@SuppressWarnings({"unused", "WeakerAccess"})
public class RoRuField extends Field
{
    public RoRuField()
    {
        super("RoverRuckus", TRACKABLE_NAMES, LOCATIONS_ON_FIELD);
    }

    //Point naming key:
    //1st char: R=Red, B=Blue
    //2nd char: L=Left start, R=Right start (viewed from red side - along field X)
    //3rd-4th chars: Pt description

    private static final String TAG = " SJH_RFD";

    private static final int BLUE  = 0;
    private static final int RED   = 1;
    private static final int STRT1 = 0;
    private static final int STRT2 = 1;
    private static final int LEFT  = 0;
    private static final int CNTR  = 1;
    private static final int RGHT  = 2;

    //Red Route > Left
    static final Point2d RLLP = new Point2d("RLLP", -12.5,  -12.5);
    static final Point2d RLTP = new Point2d("RLTP", -16.0,  -16.0);
    //includes offset for veh center. actual
    // RLM1 -46, -24; RLM2 -36, -36; RLM3 -25, -46
    static final Point2d RLM1 = new Point2d("RLM1", -45.0,  -21.0);
    static final Point2d RLM2 = new Point2d("RLM2", -33.0,  -33.0);
    static final Point2d RLM3 = new Point2d("RLM3", -21.0,  -45.0);
//    static final Point2d RLR1 = new Point2d("RLR1", -16.0,  -16.0);
//    static final Point2d RLR2 = new Point2d("RLR2",  -6.0,  -56.0);
    static final Point2d RLR1 = new Point2d("RLR1", -17.0,  -17.0);
    static final Point2d RLR2 = new Point2d("RLR2",  -7.0,  -56.0);
    static final Point2d RLDT = new Point2d("RLDT",  24.0,  -58.5);
    static final Point2d RLDP = new Point2d("RLDP",  56.0,  -56.0);
    static final Point2d RLPG = new Point2d("RLPG",  33.0,  -33.0); //prntr gold
    static final Point2d RLPP = new Point2d("RLPP", -17.0,  -59.0);

    static final Point2d BLM1 = new Point2d("BLM1",  45.0,  21.0);
    static final Point2d BLM2 = new Point2d("BLM2",  33.0,  33.0);
    static final Point2d BLM3 = new Point2d("BLM3",  21.0,  45.0);

    static final Point2d RRLP = new Point2d("RRLP",  12.5, -12.5);
    static final Point2d RRTP = new Point2d("RRTP",  16.0, -16.0);
    // RLM3 46, -24; RRM2 36, -36; RRM1 25, -46
    static final Point2d RRM1 = new Point2d("RRM1",  25.0, -57.0);
    static final Point2d RRM2 = new Point2d("RRM2",  41.0, -41.0);
    static final Point2d RRM3 = new Point2d("RRM3",  57.0, -25.0);

    static final Point2d RRR1 = new Point2d("RRR1",  59.0, -24.0);
    //static final Point2d RRR2 = new Point2d("RRR2",  57.0,  -7.0);
    //static final Point2d RRDT = new Point2d("RRDT",  59.0, -24.0);
    static final Point2d RRDP = new Point2d("RRDP",  56.0, -56.0);
    static final Point2d RRPP = new Point2d("RRPP",  60.0,  17.0);

    static final Point2d BRM3 = new Point2d("BRM3", -45.0,  21.0);
    static final Point2d BRM2 = new Point2d("BRM2", -33.0,  33.0);
    static final Point2d BRM1 = new Point2d("BRM1", -21.0,  45.0);

    public static Point2d getMineralPt(Alliance alnc,
                                       PositionOption startPos,
                                       MineralDetector.Position minPos)
    {
        Route.StartPos spos = Route.StartPos.START_1;
        if(startPos instanceof Route.StartPos) spos = (Route.StartPos) startPos;
        Point2d minPt = null;
        switch(alnc)
        {
            case RED: {
                switch (spos) {
                    case START_1: {
                        switch (minPos) {
                            case LEFT:    minPt = RLM3; break;
                            case RIGHT:   minPt = RLM1; break;
                            case CENTER:  minPt = RLM2; break;
                            case NONE: minPt = RLM2; break;
                        }
                        break;
                    }
                    case START_2: {
                        switch (minPos) {
                            case LEFT:    minPt = RRM3; break;
                            case RIGHT:   minPt = RRM1; break;
                            case CENTER:  minPt = RRM2; break;
                            case NONE: minPt = RRM2; break;
                        }
                        break;
                    }
                }
                break;
            }
            case BLUE: {
                switch (spos) {
                    case START_1: {
                        switch (minPos) {
                            case LEFT:    minPt = BLM3; break;
                            case RIGHT:   minPt = BLM1; break;
                            case CENTER:  minPt = BLM2; break;
                            case NONE: minPt = BLM2; break;
                        }
                        break;
                    }
                    case START_2: {
                        switch (minPos) {
                            case LEFT:    minPt = BRM3; break;
                            case RIGHT:   minPt = BRM1; break;
                            case CENTER:  minPt = BRM2; break;
                            case NONE: minPt = BRM2; break;
                        }
                        break;
                    }
                }
                break;
            }
        }
        return minPt;
    }

    private static final String ASSET_NAME = "RoverRuckus";

    public static final String[] TRACKABLE_NAMES =
            {
                    "BluePerimeter_Rover",
                    "RedPerimeter_Footprint",
                    "FrontPerimeter_Craters",
                    "BackPerimeter_Space"
            };

    private static final float[] TRACKABLE_POS[] = {
            scaleArr(new float[]{0.0f,     N_WALL_Y,     IMAGE_Z}, scale),
            scaleArr(new float[]{0.0f,     S_WALL_Y,     IMAGE_Z}, scale),
            scaleArr(new float[]{E_WALL_X, 0.0f,         IMAGE_Z}, scale),
            scaleArr(new float[]{W_WALL_X, 0.0f,         IMAGE_Z}, scale)
            };

    private static final OpenGLMatrix[] LOCATIONS_ON_FIELD =
            {
                    genMatrix(TRACKABLE_POS[0], new float[]{90.0f,   0.0f,   0.0f}),
                    genMatrix(TRACKABLE_POS[1], new float[]{90.0f,   0.0f, 180.0f}),
                    genMatrix(TRACKABLE_POS[2], new float[]{90.0f,   0.0f,  90.0f}),
                    genMatrix(TRACKABLE_POS[3], new float[]{90.0f,   0.0f, 270.0f})
            };
}
