package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


@SuppressWarnings("unused")
public abstract class Field
{
    public Field(String assetName)
    {
        this.assetName = assetName;
        setImageNames();
        setImageLocations();
        imgTransformMap = createPosMap(this.trackableNames, locationsOnField);
    }

    private final String         assetName;
            List<String>         trackableNames   = new ArrayList<>();
            List<OpenGLMatrix>   locationsOnField = new ArrayList<>();

    public enum Alliance {BLUE, RED}

    //  X axis parallel to red  alliance wall point toward    blue alliance
    //  Y axis parallel to blue alliance wall point away from red  alliance

    //The descriptions say the field is 12'x12', but our
    //practice field is actually slightly smaller at 141"
    private static final float X_WIDTH = 141.0f;
    private static final float Y_WIDTH = 141.0f;
    /* package */ static final float N_WALL_Y = Y_WIDTH/2.0f;
    /* package */ static final float E_WALL_X = X_WIDTH/2.0f;
    /* package */ static final float S_WALL_Y = -N_WALL_Y;
    /* package */ static final float W_WALL_X = -E_WALL_X;

    public static final float tileWidth        = 23.5f;
    public static final float oneTile          = tileWidth;
    public static final float halfTile         = tileWidth/2.0f;
    public static final float oneAndHalfTile   = 3.0f*tileWidth/2.0f;
    public static final float halfField        = X_WIDTH/2.0f;
    public static final float quadField        = X_WIDTH/4.0f;
    public static final float mmTargetHeight   = 6.0f;

    public static final float IMAGE_Z = mmTargetHeight;

    private static final float scale = (float) Units.MM_PER_INCH;

    //Note: asset file has 304mm x 224mm (12"x8.8")for RR !?!?
    //Need to figure out what xml coordinates really mean
//    public static int target_width  = 127 * 2;
//    public static int target_height = 92 * 2;
    //It turns out that we really need the trackable page dimensions in this case
    //So use 8.5"x11" paper in landscape =
    public static int target_width  = 280;
    public static int target_height = 216;

    static float[] scaleArr(float[] inArr)
    {
        float[] outArr = {inArr[0], inArr[1], inArr[2]};
        for (int i =0; i<inArr.length; ++i)
        {
            outArr[i]*= Field.scale;
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
    public List<String> getImageNames() { return trackableNames; }
    public List<OpenGLMatrix> getImageTransforms() { return locationsOnField; }
    OpenGLMatrix getAssetTransform(String name)
    {
        return imgTransformMap.get(name);
    }

    void setImageNames() {}
    void setImageLocations() {}

    @SuppressWarnings("SameParameterValue")
    void setHasVuMarks(boolean hasVuMarks)
    {
        this.hasVuMarks = hasVuMarks;
    }
    public boolean hasVuMarks() {return hasVuMarks;}
    private boolean hasVuMarks = false;

    private static Map<String, OpenGLMatrix> imgTransformMap;

    private static Map<String, OpenGLMatrix> createPosMap(List<String> names,
                                                          List<OpenGLMatrix> transforms)
    {
        Map<String, OpenGLMatrix> map = new HashMap<>();
        for (int i = 0; i < names.size() && i < transforms.size(); ++i)
        {
            map.put(names.get(i), transforms.get(i));
        }
        return map;
    }
	
	public enum PathWayToFrontStage implements PositionOption
    {
        CENTER_PATH,
        WALLSIDE_PATH
    }

    public enum PathWayToBackStage implements PositionOption
    {
        WALLSIDE_PATH,
        CENTER_PATH,
        DOOR_PATH
    }
    /* Selects which pixel stack to pick up from */
    public enum PixelStackLoc implements PositionOption
    {
        WALLSIDE_PIXELSTACK,
        CENTER_PIXELSTACK,
        DOOR_PIXELSTACK
    }

    public enum Route implements PositionOption
    {
        QUALIFIER_ROUTE,
        STATES_ROUTE
    }

    public enum StartPos implements PositionOption
    {
        START_BACKDROP,
        START_STACKS
    }

    public enum AutonDebug implements PositionOption
    {
        ENABLE,
        DISABLE
    }

    public enum ParkLocation implements PositionOption
    {
        WALLSIDE_PARK,
        CENTER_CANVAS_PARK,
        DOORSIDE_PARK
    }

    public enum stacksSideExtraPixelGrab implements PositionOption
    {
        NO_EXTRA_PIXEL,
        GRAB_EXTRA_PIXEL
    }

}
