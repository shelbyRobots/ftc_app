package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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


    static void initField(String robotName, double gOffset)
    {

    }

    //Red Route > Left
    static final Point2d RLLP = new Point2d("RLLP", -8.25,  -8.25);
    static final Point2d RLTP = new Point2d("RLTP", -17.25, -17.25);
    static final Point2d RLM1 = new Point2d("RLM1", -48.0,  -24.0);
    static final Point2d RLM2 = new Point2d("RLM2", -36.0,  -36.0);
    static final Point2d RLM3 = new Point2d("RLM3", -24.0,  -48.0);
    static final Point2d RLDT = new Point2d("RLDT",   0.0,  -52.0);
    static final Point2d RLDP = new Point2d("RLDP",  42.0,  -55.0);
    static final Point2d RLPP = new Point2d("RLPP", -24.0,  -55.0);

    static final Point2d BLLP = new Point2d("BLLP",   8.25,  8.25);
    static final Point2d BLTP = new Point2d("BLTP",  17.25, 17.25);
    static final Point2d BLM1 = new Point2d("BLM1",  48.0,  24.0);
    static final Point2d BLM2 = new Point2d("BLM2",  36.0,  36.0);
    static final Point2d BLM3 = new Point2d("BLM3",  24.0,  48.0);
    static final Point2d BLDT = new Point2d("BLDT",   0.0,  52.0);
    static final Point2d BLDP = new Point2d("BLDP", -42.0,  55.0);
    static final Point2d BLPP = new Point2d("BLPP",  24.0,  55.0);

    static final Point2d RRLP = new Point2d("RRLP",   8.25, -8.25);
    static final Point2d RRTP = new Point2d("RRTP",  17.25,-17.25);
    static final Point2d RRM3 = new Point2d("RRM3",  48.0, -24.0);
    static final Point2d RRM2 = new Point2d("RRM2",  36.0, -36.0);
    static final Point2d RRM1 = new Point2d("RRM1",  24.0, -48.0);
    static final Point2d RRDT = new Point2d("RRDT",  52.0,   0.0);
    static final Point2d RRDP = new Point2d("RRDP",  55.0, -42.0);
    static final Point2d RRPP = new Point2d("RRPP",  55.0,  24.0);

    static final Point2d BRLP = new Point2d("BRLP",  -8.25,  8.25);
    static final Point2d BRTP = new Point2d("BRTP", -17.25, 17.25);
    static final Point2d BRM3 = new Point2d("BRM3", -48.0,  24.0);
    static final Point2d BRM2 = new Point2d("BRM2", -36.0,  36.0);
    static final Point2d BRM1 = new Point2d("BRM1", -24.0,  48.0);
    static final Point2d BRDT = new Point2d("BRDT", -52.0,   0.0);
    static final Point2d BRDP = new Point2d("BRDP", -55.0,  42.0);
    static final Point2d BRPP = new Point2d("BRPP", -55.0, -24.0);

    private static final String ASSET_NAME = "RoverRuckus";

    public static final String[] TRACKABLE_NAMES =
    {
        "BluePerimeter",
        "RedPerimeter",
        "FrontPerimeter",
        "BackPerimeter",
    };

    private static final float[] TRACKABLE_POS[] = {
            scaleArr(new float[]{0.0f,     E_WALL_X, IMAGE_Z}, scale),
            scaleArr(new float[]{0.0f,     W_WALL_X, IMAGE_Z}, scale),
            scaleArr(new float[]{S_WALL_Y, 0.0f,     IMAGE_Z}, scale),
            scaleArr(new float[]{N_WALL_Y, 0.0f,     IMAGE_Z}, scale)};

    private static final OpenGLMatrix[] LOCATIONS_ON_FIELD =
     {
         genMatrix(TRACKABLE_POS[0], new float[]{90.0f,   0.0f, 0.0f}),
         genMatrix(TRACKABLE_POS[1], new float[]{90.0f, 180.0f, 0.0f}),
         genMatrix(TRACKABLE_POS[2], new float[]{90.0f,  90.0f, 0.0f}),
         genMatrix(TRACKABLE_POS[3], new float[]{90.0f, 270.0f, 0.0f})
     };
}
