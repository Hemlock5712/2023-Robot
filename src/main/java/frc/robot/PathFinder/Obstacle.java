package frc.robot.PathFinder;

public class Obstacle {
  PolygonFloat polygon;
  public Obstacle(float[] xPoints, float[] yPoints, int nPoints){
    this.polygon = new PolygonFloat(xPoints, yPoints, nPoints);
  }
}
