// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class XYACalulator {

  public static ArmSetpoint Calulator(double x,double y ,double angle){
    double ratio = 37.45/41;
    double dis = Math.hypot(Units.inchesToMeters(x), Units.inchesToMeters(y));
    double angleElvator = new Rotation2d(Units.inchesToMeters(x+33.5), Units.inchesToMeters(y)).getDegrees();
    return new ArmSetpoint(angleElvator, dis*ratio, angle-angleElvator);
  }
}
