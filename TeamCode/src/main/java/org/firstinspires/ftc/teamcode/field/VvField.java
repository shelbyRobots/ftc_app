package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Units;


@SuppressWarnings("unused")
public class VvField extends Field
{
    private static final float[] toolsPos  = {W_WALL_X,  3.0f*12, IMAGE_Z};
    private static final float[] gearsPos  = {W_WALL_X, -1.0f*12, IMAGE_Z};
    private static final float[] wheelsPos = { 1.0f*12, N_WALL_Y, IMAGE_Z};
    private static final float[] legosPos  = {-3.0f*12, N_WALL_Y, IMAGE_Z};
    //static final float[] legosPos  = {0.0f*12, 0.0f, 0.0f};

    private static final float[] toolsRot  = {90.0f, 0.0f, 90.0f};
    private static final float[] gearsRot  = {90.0f, 0.0f, 90.0f};
    private static final float[] wheelsRot = {90.0f, 0.0f,  0.0f};
    private static final float[] legosRot  = {90.0f, 0.0f,  0.0f};

    private static final float[] toolsPosMm  = scaleArr(toolsPos,  (float)Units.MM_PER_INCH);
    private static final float[] gearsPosMm  = scaleArr(gearsPos,  (float)Units.MM_PER_INCH);
    private static final float[] wheelsPosMm = scaleArr(wheelsPos, (float)Units.MM_PER_INCH);
    private static final float[] legosPosMm  = scaleArr(legosPos,  (float)Units.MM_PER_INCH);

    public static final OpenGLMatrix redToolsLocationOnField   =
            genMatrix(toolsPosMm,  toolsRot);
    public static final OpenGLMatrix redGearsLocationOnField   =
            genMatrix(gearsPosMm,  gearsRot);
    public static final OpenGLMatrix blueWheelsLocationOnField =
            genMatrix(wheelsPosMm, wheelsRot);
    public static final OpenGLMatrix blueLegosLocationOnField  =
            genMatrix(legosPosMm,  legosRot);
}
