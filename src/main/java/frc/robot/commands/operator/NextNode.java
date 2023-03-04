// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Direction;
import frc.robot.util.PlacementPosition;
import frc.robot.util.TargetLevel;
import frc.robot.util.TargetPosition;

public class NextNode extends CommandBase {
  DrivetrainSubsystem drivetrain;
  Direction direction;

  /** Creates a new NextNode. */
  public NextNode(Direction direction) {
    this.direction = direction;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.print("Placing high");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PlacementPosition current = Position.getPlacementPosition();
   
  
    int changeX = 0;
    int changeY = 0;


    switch (direction) {
      case Right:
        changeX = 1;
        break;
      case Left:
        changeX = -1;
        break;
      case Up:
        changeY = 1;
        break;
      case Down:
        changeY = -1;
        break;
      default:
        break;
    }

    if(Constants.DrivetrainConstants.alliance == Alliance.Blue){
      changeX *= -1;
    }

    TargetPosition newTargetPosition = TargetPosition.values()[((current.getPosition().ordinal() + changeX + 9) % 9)];

    TargetLevel newTargetLevel = TargetLevel.values()[(current.getLevel().ordinal()+changeY+3)%3];

    Position.setPlacementPosition(new PlacementPosition(newTargetPosition, newTargetLevel));

    SmartDashboard.putNumber("NextPosition", newTargetPosition.ordinal());
    SmartDashboard.putNumber("NextLevel", 2 - newTargetLevel.ordinal());
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
