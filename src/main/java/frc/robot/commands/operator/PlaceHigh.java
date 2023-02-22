// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.util.PlacementCalculator;
import frc.robot.util.PlacementPosition;
import frc.robot.util.TargetLevel;

public class PlaceHigh extends CommandBase {
  DrivetrainSubsystem drivetrain;

  /** Creates a new PlaceHigh. */
  public PlaceHigh(DrivetrainSubsystem drivetrainSubsystem, ) {
    this.drivetrain = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("Placing high");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PlacementPosition nextPosition = PlacementCalculator.getNextPlacementPosition(drivetrain.getTargetPosition(),
        TargetLevel.Top);
    drivetrain.setTargetPosition(nextPosition.getPosition());
    SmartDashboard.putNumber("NextPosition", nextPosition.getPosition().ordinal());
    SmartDashboard.putNumber("NextLevel", 2 - nextPosition.getLevel().ordinal());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
