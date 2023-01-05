package frc.robot.PathFinder;

import java.util.ArrayList;
import java.util.List;

public class Obstacle {
  PolygonFloat polygon;
  public Obstacle(float[] xPoints, float[] yPoints, int nPoints){
    this.polygon = new PolygonFloat(xPoints, yPoints, nPoints);
  }

  public List<Node> getNodes() {
    List<Node> nodes = new ArrayList<Node>();
    for(int i = 0; i < this.polygon.npoints; i++) {
      if(this.polygon.xpoints[i]!=0){
        nodes.add(new Node(this.polygon.xpoints[i], this.polygon.ypoints[i]));
      } 
    }
    return nodes;
  }
}
