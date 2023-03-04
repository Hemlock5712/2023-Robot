package frc.robot.commands.operator;

import frc.robot.util.PlacementPosition;
import frc.robot.util.TargetLevel;
import frc.robot.util.TargetPosition;

public class Position {
  private static PlacementPosition position = new PlacementPosition(TargetPosition.Position9, TargetLevel.Top);

  public static PlacementPosition getPlacementPosition() {
    return position;
  }

  public static void setPlacementPosition(PlacementPosition newPosition) {
    position = newPosition;
  }

}
