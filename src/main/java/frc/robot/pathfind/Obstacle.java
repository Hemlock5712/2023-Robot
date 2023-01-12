package frc.robot.pathfind;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.pathfind.util.Doubles;

public class Obstacle {
    PolygonDouble polygon;

    public Obstacle(double[] xPoints, double[] yPoints) {
        this.polygon = new PolygonDouble(xPoints, yPoints);
    }

    public void addNodes(VisGraph nodes) {
        for(int i = 0; i < polygon.npoints; i++) {
            nodes.addNode(new Node(polygon.xpoints[i], polygon.ypoints[i]));
        }
    }

    /**
     * Creates a polygon that's offset by the distance passed in.
     *
     * Makes a copy of each point of the polygon, offset by a vector
     * at 90deg from the angle of the edges, then connects them together.
     *
     * Has functionality in place to prevent issues with concave shapes
     * having overlapping lines and other weirdness.
     *
     * @param distance Distance to expand the shape outwards by.
     * @return New obstacle, which has the distance passed in added to all sides.
     */
    public Obstacle offset(double distance) {
        // Get a list of all edges with the offsets added onto them
        List<ObstacleEdge> offsetEdges = new ArrayList<>();
        for(int i = 0; i < polygon.npoints; i++) {
            offsetEdges.add(
                    new ObstacleEdge(
                    polygon.xpoints[i],
                    polygon.ypoints[i],
                    polygon.xpoints[(i + 1) % polygon.npoints],
                    polygon.ypoints[(i + 1) % polygon.npoints]
                ).offset(distance)
            );
        }
        List<Double> xPoints = new ArrayList<>();
        List<Double> yPoints = new ArrayList<>();

        // Loop through all edges, checking if there's an intersection between any of them.
        // If an intersection does occur, cut the point off at that intersection,
        // Creating a new point at the intersection, which should be the correct distance away.
        for(int i = 0; i < offsetEdges.size(); i++) {
            ObstacleEdge edge = offsetEdges.get(i);
            // Check against every other edge, including last -> first
            for(int j = i + 1; j <= offsetEdges.size(); j++) {
                // Wrap edges back to beginning so the last edge can be checked against the first one
                ObstacleEdge otherEdge = offsetEdges.get(j % offsetEdges.size());
                Translation2d intersectionPoint = edge.findIntersectionPoint(otherEdge);
                if(intersectionPoint == null && !edge.hasBeenPlotted()) {
                    // If lines don't intersect, and the edge hasn't been plotted out already
                    if(!edge.hasBeenCleaned()) {
                        // If edge was cleaned from a previous loop, don't add the first point
                        xPoints.add( offsetEdges.get(i).point1.getX());
                        yPoints.add( offsetEdges.get(i).point1.getY());
                    }
                    xPoints.add(offsetEdges.get(i).point2.getX());
                    yPoints.add(offsetEdges.get(i).point2.getY());
                    edge.setHasBeenPlotted(true);
                }
                else {
                    // If lines do intersect
                    if (!edge.hasBeenPlotted()) {
                        // Don't duplicate points
                        if (!edge.hasBeenCleaned()) {
                            // If edge was cleaned from a previous loop, don't add the first point
                            xPoints.add( offsetEdges.get(i).point1.getX());
                            yPoints.add( offsetEdges.get(i).point1.getY());
                        }
                        xPoints.add( intersectionPoint.getX());
                        yPoints.add( intersectionPoint.getY());
                        otherEdge.setHasBeenCleaned(true);
                        edge.setHasBeenPlotted(true);
                    }
                }
            }
        }
        return new Obstacle(Doubles.toArray(xPoints), Doubles.toArray(yPoints));
    }

    public String toString() {
        String output = "Polygon(\n";
        for(int i = 0; i < polygon.npoints; i++) {
            output += Double.toString(Math.round(polygon.xpoints[i] * 100) / 100.0) + ", ";
            output += Double.toString(Math.round(polygon.ypoints[i] * 100) / 100.0) + "\n";
        }
        return output + ")";
    }

    private class ObstacleEdge {
        private Translation2d point1;
        private Translation2d point2;
        private boolean hasBeenCleaned = false;
        private boolean hasBeenPlotted = false;

        public ObstacleEdge(double x1, double y1, double x2, double y2) {
            point1 = new Translation2d(x1, y1);
            point2 = new Translation2d(x2, y2);
        }

        public ObstacleEdge(Translation2d point1, Translation2d point2) {
            this.point1 = point1;
            this.point2 = point2;
        }

        public ObstacleEdge offset(double distance) {
            // Calculate angle of edge
            Rotation2d angle = point2.minus(point1).getAngle();
            // Calculate perpendicular angle
            Rotation2d transformAngle = angle.plus(Rotation2d.fromDegrees(90));
            // Create offset vector using distance and angles
            Translation2d offset = new Translation2d(distance, 0).rotateBy(transformAngle);
            // Add offset to points
            return new ObstacleEdge(
                    point1.plus(offset),
                    point2.plus(offset)
            );
        }

        /**
         * This is heavily based on an algorithm in the
         * "Tricks of the Windows Game Programming Gurus" book by Andre LeMothe
         *
         * @param other
         * @return
         */
        public Translation2d findIntersectionPoint(ObstacleEdge other) {
            // Calculate x distance of line 1
            double s1_x = this.point2.getX() - this.point1.getX();
            // Calculate x distance of line 2
            double s2_x = other.point2.getX() - other.point1.getX();
            // Calculate y distance of line 1
            double s1_y = this.point2.getY() - this.point1.getY();
            // Calculate y distance of line 2
            double s2_y = other.point2.getY() - other.point1.getY();

            double s, t;
            // Denominator portion of below equations, split into variable because it's the same between the two
            double d = -s2_x * s1_y + s1_x * s2_y;

            // Magical math that I need to look into how it works more
            s = (-s1_y * (this.point1.getX() - other.point1.getX()) + s1_x * (this.point1.getY() - other.point1.getY())) / d;
            t = ( s2_x * (this.point1.getY() - other.point1.getY()) - s2_y * (this.point1.getX() - other.point1.getX())) / d;

            double i_x, i_y;
            if(s >= 0 && s <= 1 && t >= 0 && t <= 1) {
                // Intersection found
                i_x = this.point1.getX() + (t * s1_x);
                i_y = this.point1.getY() + (t * s1_y);
                return new Translation2d(i_x, i_y);
            }
            return null;

        }

        public String toString() {
            return point1.toString() + "," + point2.toString();
        }

        public boolean hasBeenCleaned() {
            return hasBeenCleaned;
        }

        public void setHasBeenCleaned(boolean hasBeenCleaned) {
            this.hasBeenCleaned = hasBeenCleaned;
        }

        public boolean hasBeenPlotted() {
            return hasBeenPlotted;
        }

        public void setHasBeenPlotted(boolean hasBeenPlotted) {
            this.hasBeenPlotted = hasBeenPlotted;
        }
    }
}
