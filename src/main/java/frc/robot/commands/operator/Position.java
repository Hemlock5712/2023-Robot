package frc.robot.commands.operator;

import frc.robot.util.PlacementCalculator;
import frc.robot.util.PlacementPosition;

public class Position {
  private static PlacementPosition position = PlacementCalculator.order.get(0);

  public static PlacementPosition getPlacementPosition() {
    return position;
  }

  public static void setPlacementPosition(PlacementPosition newPosition) {
    position = newPosition;
  }

}
