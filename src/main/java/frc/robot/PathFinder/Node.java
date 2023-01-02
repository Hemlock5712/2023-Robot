package frc.robot.PathFinder;

import java.util.ArrayList;
import java.util.List;

// A class representing a node in the navigation mesh
public class Node {
  double x, y, holonomicRotation;
  List < Node > neighbors;

  public Node(double x, double y) {
      this.x = x;
      this.y = y;
      holonomicRotation = -1;
      this.neighbors = new ArrayList < > ();
  }

  public Node(double x, double y, double holonomicRotation) {
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
  public double getHolRot(){
    return holonomicRotation;
  }
}
