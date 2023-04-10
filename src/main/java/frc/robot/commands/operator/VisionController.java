package frc.robot.commands.operator;

import frc.robot.Constants;

public class VisionController {
  public static void setVisionOn() {
    Constants.VisionConstants.USE_VISION = true;
  }

  public static void setVisionOff() {
    Constants.VisionConstants.USE_VISION = false;
  }

}
