package frc.robot.PathFinder;

import java.util.ArrayList;
import java.util.List;

public class Obstacle {
  PolygonFloat polygon;
  public Obstacle(float[] xPoints, float[] yPoints, int nPoints){
    this.polygon = new PolygonFloat(xPoints, yPoints, nPoints);
  }

  public List<Node> getNodes(double robotSize) {
    List<Node> nodes = new ArrayList<Node>();
    for (int i = 0; i < this.polygon.npoints; i++) {
        if (this.polygon.xpoints[i] != 0) {
            // create a node at a distance from the obstacle equal to the size of the robot
            double buffer = robotSize / 2;
            nodes.add(new Node(this.polygon.xpoints[i] + buffer, this.polygon.ypoints[i] + buffer));
            nodes.add(new Node(this.polygon.xpoints[i] - buffer, this.polygon.ypoints[i] - buffer));
            nodes.add(new Node(this.polygon.xpoints[i] + buffer, this.polygon.ypoints[i] - buffer));
            nodes.add(new Node(this.polygon.xpoints[i] - buffer, this.polygon.ypoints[i] + buffer));
        }
    }
    return nodes;
  }

}
