package frc.robot.util;

import java.util.List;

public class PlacementCalculator {

  public static List<PlacementPosition> order = List.of(
      new PlacementPosition(TargetPosition.Position9, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position8, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position7, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position6, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position5, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position4, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position3, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position2, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position1, TargetLevel.Top),
      new PlacementPosition(TargetPosition.Position9, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position8, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position7, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position6, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position5, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position4, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position3, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position2, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position1, TargetLevel.Mid),
      new PlacementPosition(TargetPosition.Position9, TargetLevel.Low),
      new PlacementPosition(TargetPosition.Position8, TargetLevel.Low),
      new PlacementPosition(TargetPosition.Position7, TargetLevel.Low),
      new PlacementPosition(TargetPosition.Position6, TargetLevel.Low),
      new PlacementPosition(TargetPosition.Position5, TargetLevel.Low),
      new PlacementPosition(TargetPosition.Position4, TargetLevel.Low),
      new PlacementPosition(TargetPosition.Position3, TargetLevel.Low),
      new PlacementPosition(TargetPosition.Position2, TargetLevel.Low),
      new PlacementPosition(TargetPosition.Position1, TargetLevel.Low));

  /**
   * Determine the next position to drive the robot to
   * 
   * @param position Position where robot just placed game piece
   * @param level    Level where robot just placed game piece
   * @return Next position
   */
  public static PlacementPosition getNextPlacementPosition(TargetPosition position, TargetLevel level) {
    int i = order.indexOf(new PlacementPosition(position, level));

    if (i < order.size()) {
      return order.get((i + 1) % order.size());
    }
    return order.get(0);
  }

  /**
   * Determine the next position to drive the robot to
   * 
   * @param position Position where robot just placed game piece
   * @param level    Level where robot just placed game piece
   * @return Next position
   */
  public static PlacementPosition getNextPlacementPosition(PlacementPosition currentPosition) {
    int i = order.indexOf(new PlacementPosition(currentPosition.getPosition(), currentPosition.getLevel()));

    if (i < order.size()) {
      return order.get((i + 1) % order.size());
    }
    return order.get(0);
  }

  /**
   * Determine the next position to drive the robot to
   * 
   * @param currentPosition Position where robot just placed game piece
   * @return Prev position
   */
  public static PlacementPosition getPreviousPlacementPosition(PlacementPosition currentPosition) {
    int i = order.indexOf(new PlacementPosition(currentPosition.getPosition(), currentPosition.getLevel()));
    System.out.println(i);
    if (i < 0) {
      return order.get((order.size() - 1) % order.size());
    }
    return order.get((i - 1) % order.size());
  }
}
