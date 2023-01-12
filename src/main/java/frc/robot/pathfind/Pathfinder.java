package frc.robot.pathfind;

import java.util.ArrayList;
import java.util.List;

public class Pathfinder {
    List<Obstacle> obstacles = new ArrayList<>();
    List<Obstacle> obstaclesWithOffsets = new ArrayList<>();
    VisGraph navMesh = new VisGraph();
    double obstacleOffsetDistance;

    public Pathfinder(double obstacleOffsetDistance) {
        this.obstacleOffsetDistance = obstacleOffsetDistance;
    }

    public Pathfinder(double obstacleOffsetDistance, List<Obstacle> obstacles) {
        this(obstacleOffsetDistance);
        this.obstacles = obstacles;
        obstaclesWithOffsets = obstacles.stream().map(o -> o.offset(obstacleOffsetDistance)).toList();
        obstaclesWithOffsets.forEach(this::addObstacleNodes);
    }

    public void addObstacle(Obstacle obstacle) {
        this.obstacles.add(obstacle);
        this.obstaclesWithOffsets.add(obstacle.offset(obstacleOffsetDistance));
        addObstacleNodes(obstacle);
    }

    public void addObstacleNodes(Obstacle obstacle) {
        obstacle.addNodes(navMesh);
    }

    /**
     * Finds a path between the start point and end point.
     * End point should have a node created and edges added for it earlier on, probably
     * at robot startup, since we'll know all the points we have presets for
     *
     * @param startPoint Current robot position
     * @param endPoint Target position to find a path to
     * @return List of nodes to create a trajectory through, or null if no path is found
     */
    public List<Node> findPath(Node startPoint, Node endPoint) {
        // Add edges from current position to all other positions
        addNode(startPoint);

        return navMesh.findPath(startPoint, endPoint);
    }

    public void addNode(Node node) {
        navMesh.addNode(node);
        for(int i = 0; i < navMesh.getNodeSize(); i++) {
            Node endNode = navMesh.getNode(i);
            navMesh.addEdge(new Edge(node, endNode), obstacles);
        }
    }

    /**
     * Add edges between all nodes. Do this after all obstacles are added to the field
     */
    public void generateNodeEdges() {
        for(int i = 0; i < navMesh.getNodeSize(); i++) {
            Node startNode = navMesh.getNode(i);
            for(int j = i + 1; j < navMesh.getNodeSize(); j++) {
                Node endNode = navMesh.getNode(j);
                navMesh.addEdge(new Edge(startNode, endNode), obstacles);
            }
        }
    }
}
