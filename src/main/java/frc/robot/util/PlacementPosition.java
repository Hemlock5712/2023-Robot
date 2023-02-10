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
}
