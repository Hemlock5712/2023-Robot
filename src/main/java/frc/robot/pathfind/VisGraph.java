package frc.robot.pathfind;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class VisGraph {

    // A class representing the navigation mesh
    private final List<Node> nodes;
    private final List<Edge> edges;

    public VisGraph() {
        this.nodes = new ArrayList<>();
        this.edges = new ArrayList<>();
    }

    // Add a node to the navigation mesh
    public void addNode(Node node) {
        this.nodes.add(node);
    }

    public int getNodeSize() {
        return nodes.size();
    }

    public Node getNode(int index) {
        return nodes.get(index);
    }

    // Add an edge to the navigation mesh
    public boolean addEdge(Edge edge, List<Obstacle> obstacles) {
        // Why not use the Line2D class' static method of .linesIntersect() ? I am just hold on
        for (Obstacle obstacle : obstacles) {
            PolygonDouble polygon = obstacle.polygon;
            for (int i = 0; i < polygon.npoints; i++) {
                int j = (i + 1) % polygon.npoints;
                // Couldn't the mod be eliminated by starting the index i at 1 and manually checking the segment between the last vertex and first one before this? Well
                double x1 = polygon.xpoints[i];
                double y1 = polygon.ypoints[i];
                double x2 = polygon.xpoints[j];
                double y2 = polygon.ypoints[j];
                if (Line2D.linesIntersect(x1, y1, x2, y2, edge.start.x, edge.start.y, edge.end.x, edge.end.y)) {
                    return false;
                }
            }
        }
        this.edges.add(edge);
        edge.start.addNeighbor(edge.end);
        edge.end.addNeighbor(edge.start);
        return true;
    }


    // Find a path through the navigation mesh from the start node to the goal node
    public List<Node> findPath(Node start, Node goal) {
        // Use A* search to find the shortest path through the navigation mesh
        Set<Node> closedSet = new HashSet<>();
        Set<Node> openSet = new HashSet<>();
        Map<Node, Double> gScore = new HashMap<>();
        Map<Node, Double> fScore = new HashMap<>();
        Map<Node, Node> cameFrom = new HashMap<>();
        gScore.put(start, 0.0);
        fScore.put(start, distance(start, goal));
        openSet.add(start);

        while (!openSet.isEmpty()) {

            Node current = getLowestFScore(openSet, fScore);
            if (current.equals(goal)) {
                return reconstructPath(cameFrom, current);
            }
            openSet.remove(current);
            closedSet.add(current);
            for (Node neighbor : current.neighbors) {
                if (!closedSet.contains(neighbor)) {
                    double tentativeGScore = gScore.get(current) + distance(current, neighbor);
                    if (!openSet.contains(neighbor) || tentativeGScore < gScore.get(neighbor)) {
                        cameFrom.put(neighbor, current);
                        gScore.put(neighbor, tentativeGScore);
                        fScore.put(neighbor, gScore.get(neighbor) + distance(neighbor, goal));
                        // No check needed, .add is a noop if it contains it. Sets don't allow duplicates
                        openSet.add(neighbor);
                    }
                }
            }
        }

        // If we get here, then no path was found
        return null;
    }

    // Calculate the distance between two nodes
    private static double distance(Node n1, Node n2) {
        double dx = n1.x - n2.x;
        double dy = n1.y - n2.y;
        return Math.hypot(dx, dy);
    }

    // Get the node in the open set with the lowest f score
    private Node getLowestFScore(Set<Node> openSet, Map<Node, Double> fScore) {
        Node lowestFScoreNode = null;
        double lowestFScore = Double.MAX_VALUE;
        for (Node node : openSet) {
            double f = fScore.get(node);
            if (f < lowestFScore) {
                lowestFScore = f;
                lowestFScoreNode = node;
            }
        }
        return lowestFScoreNode;
    }

    // Reconstruct the path from the start node to the goal node
    private List<Node> reconstructPath(Map<Node, Node> cameFrom, Node current) {
        List<Node> path = new ArrayList<>();
        path.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            path.add(current);
        }
        Collections.reverse(path);
        return path;
    }
}
