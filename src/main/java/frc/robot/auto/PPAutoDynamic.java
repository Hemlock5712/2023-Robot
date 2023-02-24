// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.pathfind.AutoClass;
import frc.robot.pathfind.FindTrajectory;
import frc.robot.pathfind.GamePiece;
import frc.robot.pathfind.Node;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.FieldConstants;


public class PPAutoDynamic extends CommandBase {
  /** Creates a new PPautoPath. */

  double Blue[][]= 
  { 
    {FieldConstants.Grids.lowTranslations[0].getY() ,2 }, 
    {FieldConstants.Grids.lowTranslations[1].getY() ,2 }, 
    {FieldConstants.Grids.lowTranslations[2].getY() ,2 }, 
    {FieldConstants.Grids.lowTranslations[3].getY() ,2 }, 
    {FieldConstants.Grids.lowTranslations[4].getY() ,2 }, 
    {FieldConstants.Grids.lowTranslations[5].getY() ,2 }, 
    {FieldConstants.Grids.lowTranslations[6].getY() ,2 }, 
    {FieldConstants.Grids.lowTranslations[7].getY() ,2 }, 
    {FieldConstants.Grids.lowTranslations[8].getY() ,2 } 
  };
  double preplaced[][] = 
  {
    {FieldConstants.StagingLocations.translations[0].getY(),FieldConstants.StagingLocations.translations[0].getX()},
    {FieldConstants.StagingLocations.translations[1].getY(),FieldConstants.StagingLocations.translations[1].getX()},
    {FieldConstants.StagingLocations.translations[2].getY(),FieldConstants.StagingLocations.translations[2].getX()},
    {FieldConstants.StagingLocations.translations[3].getY(),FieldConstants.StagingLocations.translations[3].getX()}
  };

  private final DrivetrainSubsystem Drivetrain;
  private final PoseEstimatorSubsystem PoseEstimator;
  private final VisGraph AStarMap;
  private final List<Obstacle> obstacles;
  private final PathConstraints constraints;
  private PPSwerveControllerCommand pathDrivingCommand;



  public PPAutoDynamic(DrivetrainSubsystem d, PoseEstimatorSubsystem p, VisGraph AStarMap, List<Obstacle> obstacles, PathConstraints constraints) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Drivetrain = d;
    this.PoseEstimator = p;
    this.AStarMap = AStarMap;
    this.obstacles = obstacles;
    this.constraints = constraints;

    addRequirements(Drivetrain, PoseEstimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    NetworkTableEntry pointTable = NetworkTableInstance.getDefault().getTable("/Auto").getEntry("Points");
    
    NetworkTable autoTable = NetworkTableInstance.getDefault().getTable("/Auto/Grid");
    NetworkTable preplaced = NetworkTableInstance.getDefault().getTable("/Auto/Staged");
    String preload = NetworkTableInstance.getDefault().getTable("/Auto").getEntry("Preload").getString("empty");
    List<NetworkTableEntry> pinListNetworkTableEntries = new ArrayList<>();
    List<NetworkTableEntry> preplacedListNetworkTableEntries = new ArrayList<>();
    List<Double> XYList = new ArrayList<>();
 
    List<AutoClass> preplacedList = new ArrayList<>();
    List<AutoClass> pinList = new ArrayList<>();
    List<PathPlannerTrajectory> trajectories = new ArrayList<>();

    double startTime = System.currentTimeMillis();

    for (int i = 0; i < 9; i++) {
      for (int j = 0; j < 3; j++) {
        String string = autoTable.getEntry(i+"-"+j).getString("empty");
        if(!string.equals("empty")){
          pinListNetworkTableEntries.add(autoTable.getEntry(i+"-"+j));
        }
      }
    }

    System.out.println(System.currentTimeMillis()-startTime);


    
    for (int i = 0; i < 4; i++) {
      String string = preplaced.getEntry(i+"").getString("empty");
      if(!string.equals("empty")){
        preplacedListNetworkTableEntries.add(preplaced.getEntry(i+""));
      }
    }



    for (NetworkTableEntry i : pinListNetworkTableEntries) {
      Node node;
      GamePiece gamePiece = GamePiece.Empty;
      int index = Integer.parseInt(i.getName().split("/")[3].split("-")[0]);
      double[] cords = Blue[index];
      node = new Node(cords[1], cords[0],new Rotation2d(180));
      int level = Integer.parseInt(i.getName().split("-", 2)[1]);

      if(i.getValue().getString().equals("cone")) {
        gamePiece = GamePiece.Cone;
      }
      if(i.getValue().getString().equals("cube")) {
        gamePiece = GamePiece.Cube;
      }
      
      
      AutoClass c = new AutoClass(node, level, gamePiece);
      pinList.add(c);
    }




    for (NetworkTableEntry i : preplacedListNetworkTableEntries) {
      int index = Integer.parseInt(i.getName().split("/")[3]);
      Translation2d cords = FieldConstants.StagingLocations.translations[index];
      Node node = new Node(cords.getX(), cords.getY());
      GamePiece gamePiece = GamePiece.Empty;
      if(i.getValue().toString().equals("cone")){
        gamePiece = GamePiece.Cone;
      }
      if(i.getValue().toString().equals("cube")){
        gamePiece = GamePiece.Cube;
      }

      AutoClass c = new AutoClass(node, -1, gamePiece);
      preplacedList.add(c);
    }

    if((preplacedList.size()+1)-pinList.size() >= 0){

      int closeIndex = 0;
      //PoseEstimator.getCurrentPose()
      Node startingPoint = new Node(PoseEstimator.getCurrentPose().getX(),PoseEstimator.getCurrentPose().getY(),PoseEstimator.getCurrentPose().getRotation());

      double BestDis = 10000;
      double currentDis = 10000;

      for (int i = 0; i < pinList.size(); i++) {
        
        if(preload.equals("cone") && pinList.get(i).getGameObject() == GamePiece.Cone){
          currentDis = Math.hypot(pinList.get(i).getNode().getX()-startingPoint.getX(), pinList.get(i).getNode().getY()-startingPoint.getY());
          if(BestDis>currentDis){
            BestDis = currentDis;
            closeIndex = i;
          }
        }
        if(preload.equals("cube") && pinList.get(i).getGameObject() == GamePiece.Cube){
          currentDis = Math.hypot(pinList.get(i).getNode().getX()-startingPoint.getX(), pinList.get(i).getNode().getY()-startingPoint.getY());
          if(BestDis>currentDis){
            BestDis = currentDis;
            closeIndex = i;
          }
        }
      }


      Node finalPosition;
      finalPosition = pinList.get(closeIndex).getNode();

    
      if (DriverStation.getAlliance() == Alliance.Red) {
        Pose2d flippedY = new Pose2d(startingPoint.getX(),
            FieldConstants.FIELD_WIDTH_METERS - startingPoint.getY(),
            startingPoint.getHolRot());
        startingPoint = new Node(flippedY);
      }

      PathPlannerTrajectory StartPath = FindTrajectory.getPathTrajectory(Drivetrain, constraints ,obstacles,AStarMap,startingPoint,finalPosition);

      // PoseEstimator.addTrajectory(StartPath);

      // System.out.println(StartPath.getTotalTimeSeconds()+"________________________");

      finalPosition.setHolRot(Rotation2d.fromDegrees(180));

      

      XYList = addNode(XYList, startingPoint);
      

      trajectories.add(StartPath);
      pinList.remove(closeIndex);
      closeIndex = 0;



      PathPlannerTrajectory currentPath;
      PathPlannerTrajectory BestPath;
      int preplacedListSize = (preplacedList.size()*2);

      for (int i = 0; i < (preplacedListSize); i++) {
        closeIndex = 0;
        startingPoint = finalPosition;
        if(i%2 != 1){
          
          
          BestPath = FindTrajectory.getPathTrajectory(Drivetrain, constraints,obstacles,AStarMap,startingPoint,preplacedList.get(0).getNode());

          for (int j = 0; j < preplacedList.size(); j++) {
            
            currentPath = FindTrajectory.getPathTrajectory(Drivetrain, constraints,obstacles,AStarMap,startingPoint,preplacedList.get(j).getNode());
            if(BestPath.getTotalTimeSeconds()>currentPath.getTotalTimeSeconds()){
              BestPath = currentPath;
              closeIndex = j;
              
            }
          }
          
          // PoseEstimator.addTrajectory(BestPath);
          finalPosition = preplacedList.get(closeIndex).getNode();
          finalPosition.setHolRot(Rotation2d.fromDegrees(180));


          XYList = addNode(XYList, startingPoint);

          trajectories.add(BestPath);
          preplacedList.remove(closeIndex);

        }
        else{

          if(pinList.size() != 0){
            

            BestPath = FindTrajectory.getPathTrajectory(Drivetrain, constraints,obstacles,AStarMap,startingPoint,pinList.get(0).getNode());
              for (int j = 0; j < pinList.size(); j++) {
                currentPath = FindTrajectory.getPathTrajectory(Drivetrain, constraints,obstacles,AStarMap,startingPoint,pinList.get(j).getNode());
                if(BestPath.getTotalTimeSeconds()>currentPath.getTotalTimeSeconds()){
                  BestPath = currentPath;
                  closeIndex = j;
                }
              }

            finalPosition = pinList.get(closeIndex).getNode();
            finalPosition.setHolRot(Rotation2d.fromDegrees(180));


            XYList = addNode(XYList, startingPoint);

            trajectories.add(BestPath);
            pinList.remove(closeIndex);

          }
          else{
            break;
          }
        }
      
      }
    }
    else{

    }

    double[] doubleArr = new double[XYList.size()];

    for (int i = 0; i < XYList.size(); i++) {
      doubleArr[i] = XYList.get(i);
    }
    pointTable.setDoubleArray(doubleArr);

    Trajectory currentPath = trajectories.get(0);
    for (int i = 0; i < trajectories.size()-1; i++) {
      currentPath = currentPath.concatenate(trajectories.get(i+1));
    }
    PoseEstimator.addTrajectory(currentPath);

    Command[] followPathCommands = new Command[trajectories.size()];

    // pathDrivingCommand = DrivetrainSubsystem.followTrajectory(Drivetrain, PoseEstimator, trajectories.get(0));
    for (int i = 0; i < trajectories.size(); i++) {
      
      // pathDrivingCommand.andThen(DrivetrainSubsystem.followTrajectory(Drivetrain, PoseEstimator, trajectories.get(i)));
      followPathCommands[i] = (DrivetrainSubsystem.followTrajectory(Drivetrain, PoseEstimator, trajectories.get(i)));
    }
    // pathDrivingCommand.schedule();

    Command command = new SequentialCommandGroup(followPathCommands);
    command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
    
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private List<Double> addNode(List<Double> XYList, Node node){
    XYList.add(node.getX());
    XYList.add(node.getY());
    return XYList;
  }

}
