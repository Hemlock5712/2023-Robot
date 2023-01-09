package frc.robot.pathfind;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

public class Node {
    double x, y;
    Rotation2d holonomicRotation;
    List < Node > neighbors;
  
    public Node(double x, double y) {
        this.x = x;
        this.y = y;
        holonomicRotation = Rotation2d.fromDegrees(0);
        this.neighbors = new ArrayList < > ();
    }
  
    public Node(double x, double y, Rotation2d holonomicRotation) {
        this.x = x;
        this.y = y;
        this.holonomicRotation = holonomicRotation;
        this.neighbors = new ArrayList < > ();
    }
  
    public void addNeighbor(Node neighbor) {
        this.neighbors.add(neighbor);
    }
    public double getX(){
      return x;
    }
    public double getY(){
      return y;
    }
    public Rotation2d getHolRot(){
      return holonomicRotation;
    }
  }
