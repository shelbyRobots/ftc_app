package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.HashMap;
import java.util.Map;


@SuppressWarnings("unused")
public abstract class Field
{
    public Field(String assetName, String[] imageNames, OpenGLMatrix[] imageTransforms)
    {
        this.assetName = assetName;
        this.trackableNames = imageNames;
        this.locationsOnField = imageTransforms;
        imgTransformMap = createPosMap(this.trackableNames, this.locationsOnField);
    }

    private final String         assetName ;
    private final String[]       trackableNames;
    private final OpenGLMatrix[] locationsOnField;

    public enum Alliance {BLUE, RED}

    //  X axis parallel to red  alliance wall point toward    blue alliance
    //  Y axis parallel to blue alliance wall point away from red  alliance

    //The descriptions say the field is 12'x12', but our
    //practice field is actually slightly smaller at 141"
    private static final float X_WIDTH = 141.0f;
    private static final float Y_WIDTH = 141.0f;
    /* package */ static final float N_WALL_Y = Y_WIDTH/2.0f;
    /* package */ static final float E_WALL_X = X_WIDTH/2.0f;
    public static final float S_WALL_Y = -N_WALL_Y;
    public static final float W_WALL_X = -E_WALL_X;

    static final float IMAGE_Z = 6.50f;

    /* protected */ static final float scale = (float) Units.MM_PER_INCH;

    //Note: asset file has 304mm x 224mm (12"x8.8")for RR !?!?
    //Need to figure out what xml coordinates really mean
//    public static int target_width  = 127 * 2;
//    public static int target_height = 92 * 2;
    //It turns out that we really need the trackable page dimensions in this case
    //So use 8.5"x11" paper in landscape =
    public static int target_width  = 280;
    public static int target_height = 216;

    static float[] scaleArr(float[] inArr, float scale)
    {
        float[] outArr = {inArr[0], inArr[1], inArr[2]};
        for (int i =0; i<inArr.length; ++i)
        {
            outArr[i]*=scale;
        }
        return outArr;
    }

    static OpenGLMatrix genMatrix(float[] pos, float[] rot)
    {
        return OpenGLMatrix
                .translation(pos[0], pos[1], pos[2])
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                        rot[0], rot[1], rot[2]));
    }

    public String getAssetName()    { return assetName; }
    public String[] getImageNames() { return trackableNames; }
    public OpenGLMatrix[] getImageTransforms() { return locationsOnField; }

    OpenGLMatrix getAssetTransform(String name)
    {
        return imgTransformMap.get(name);
    }

    private static Map<String, OpenGLMatrix> imgTransformMap;

    private static Map<String, OpenGLMatrix> createPosMap(String[] names,
                                                           OpenGLMatrix[] transforms)
    {
        Map<String, OpenGLMatrix> map = new HashMap<>();
        for (int i = 0; i < names.length && i < transforms.length; ++i)
        {
            map.put(names[i], transforms[i]);
        }
        return map;
    }
}
