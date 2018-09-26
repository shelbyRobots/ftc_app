package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.util.LinkedHashSet;
import java.util.Set;


@SuppressWarnings("unused")
public class VvField extends Field
{
    public VvField()
    {
        super("RoverRuckus", TRACKABLE_NAMES, LOCATIONS_ON_FIELD);
    }

    private static final String[] TRACKABLE_NAMES = {
            "BlueWheels",
            "BlueLegos",
            "RedTools",
            "RedGears"};

    private static final float[] TRACKABLE_POS[] = {
            scaleArr(new float[] {W_WALL_X,  3.0f*12, IMAGE_Z}, scale),
            scaleArr(new float[] {W_WALL_X, -1.0f*12, IMAGE_Z}, scale),
            scaleArr(new float[] { 1.0f*12, N_WALL_Y, IMAGE_Z}, scale),
            scaleArr(new float[] {-3.0f*12, N_WALL_Y, IMAGE_Z}, scale)};

    private static final OpenGLMatrix[] LOCATIONS_ON_FIELD = {
            genMatrix(TRACKABLE_POS[0], new float[] {90.0f, 0.0f, 90.0f}),
            genMatrix(TRACKABLE_POS[1], new float[] {90.0f, 0.0f, 90.0f}),
            genMatrix(TRACKABLE_POS[2], new float[] {90.0f, 0.0f,  0.0f}),
            genMatrix(TRACKABLE_POS[3], new float[] {90.0f, 0.0f,  0.0f})};

    public static final OpenGLMatrix redToolsLocationOnField   = LOCATIONS_ON_FIELD[0];
    public static final OpenGLMatrix redGearsLocationOnField   = LOCATIONS_ON_FIELD[1];
    public static final OpenGLMatrix blueWheelsLocationOnField = LOCATIONS_ON_FIELD[2];
    public static final OpenGLMatrix blueLegosLocationOnField  = LOCATIONS_ON_FIELD[3];
}
