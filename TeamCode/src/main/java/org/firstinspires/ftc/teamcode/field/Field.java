package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@SuppressWarnings("unused")
public class Field
{
    public enum Alliance {BLUE, RED}

    public enum AutoStrategy
    {
        PUSH_STACK_PARK,
        SHOOT_PUSH_PARK,
        SHOOT_PARK
    }

    public enum StartPos
    {
        START_1,
        START_2,
        START_A_SWEEPER,
        START_B_SWEEPER,
        START_R_PUSHER,
        START_TEST
    }

    public enum BeaconChoice
    {
        BOTH,
        NEAR,
        FAR,
        NONE
    }

    public enum ParkChoice
    {
        CENTER_PARK,
        CORNER_PARK,
        DEFEND_PARK,
        SAFE_ZONE_1,
        SAFE_ZONE_2
    }

    //  X axis parallel to red  alliance wall point toward    blue alliance
    //  Y axis parallel to blue alliance wall point away from red  alliance

    //The descriptions say the field is 12'x12', but our
    //practice field is actually slightly smaller at 141"
    private static final float X_WIDTH = 141.0f;
    private static final float Y_WIDTH = 141.0f;
    public static final float N_WALL_Y = Y_WIDTH/2.0f;
    public static final float E_WALL_X = X_WIDTH/2.0f;
    public static final float S_WALL_Y = -N_WALL_Y;
    public static final float W_WALL_X = -E_WALL_X;

    static final float MM_PER_INCH = 25.4f;
    static final float IMAGE_Z = 6.50f;

    static float[] scaleArr(float[] inArr, float scale)
    {
        float[] outArr = {0.0f, 0.0f, 0.0f};
        for (int i =0; i<inArr.length; ++i)
        {
            outArr[i] = inArr[i] * scale;
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
}
