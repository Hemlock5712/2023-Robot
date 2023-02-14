package frc.robot.util;

public class PlacementPosition {

  private TargetPosition position;

  public TargetPosition getPosition() {
    return position;
  }

  private TargetLevel level;

  public TargetLevel getLevel() {
    return level;
  }

  public PlacementPosition(TargetPosition position, TargetLevel level) {
    this.position = position;
    this.level = level;
  }

  public boolean equals(Object other) {
    if (other instanceof PlacementPosition) {
      return ((PlacementPosition) other).level == level && ((PlacementPosition) other).position == position;
    }
    return false;
  }
}
