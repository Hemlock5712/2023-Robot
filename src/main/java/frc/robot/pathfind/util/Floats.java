package frc.robot.pathfind.util;

import java.util.List;

public class Floats {
    public static float[] toArray(List<Float> arr) {
        float[] floats = new float[arr.size()];
        for(int i = 0; i < arr.size(); i++) {
            floats[i] = arr.get(i);
        }
        return floats;
    }
}