package frc.robot.pathfind.util;

import java.util.List;

public class Doubles {
    public static double[] toArray(List<Double> arr) {
        double[] doubles = new double[arr.size()];
        for(int i = 0; i < arr.size(); i++) {
            doubles[i] = arr.get(i);
        }
        return doubles;
    }
}