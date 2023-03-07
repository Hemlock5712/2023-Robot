// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class PiecePicker {
  static boolean gamePiecePicked = true;

  public static void toggle(boolean i) {
    if (i) {
      gamePiecePicked = true;
    } else {
      gamePiecePicked = false;
    }
  }

  public static boolean getPiecePicker() {
    return gamePiecePicked;
  }

}
