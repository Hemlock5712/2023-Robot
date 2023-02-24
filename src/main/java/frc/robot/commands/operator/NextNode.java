// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.PlacementCalculator;
import frc.robot.util.PlacementPosition;

public class NextNode extends CommandBase {
  DrivetrainSubsystem drivetrain;
  int movement;

  /** Creates a new NextNode. */
  public NextNode(DrivetrainSubsystem drivetrainSubsystem, int movement) {
    this.drivetrain = drivetrainSubsystem;
    this.movement = movement;
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
    int direction = movement;
    PlacementPosition nextPosition;
    if(Constants.DrivetrainConstants.alliance == Alliance.Red){
      direction = 1 - direction;
    }
    if(direction == 1){
       nextPosition = PlacementCalculator.getNextPlacementPosition(drivetrain.getPlacementPosition());
    }
    else{
      nextPosition = PlacementCalculator.getPreviousPlacementPosition(drivetrain.getPlacementPosition());
    }
  
    drivetrain.setTargetPosition(nextPosition);
    SmartDashboard.putNumber("NextPosition", nextPosition.getPosition().ordinal());
    SmartDashboard.putNumber("NextLevel", 2 - nextPosition.getLevel().ordinal());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
