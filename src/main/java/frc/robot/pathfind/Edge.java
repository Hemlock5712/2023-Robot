package frc.robot.pathfind;

public class Edge {
    Node start, end;
    double cost;

    public Edge(Node start, Node end, double cost) {
        this.start = start;
        this.end = end;
        this.cost = cost;
    }

    public Edge(Node start, Node end) {
        this.start = start;
        this.end = end;
        this.cost = distance(start, end);
    }
    // Calculate the distance between two nodes
    private static double distance(Node n1, Node n2) {
        double dx = n1.x - n2.x;
        double dy = n1.y - n2.y;
        return Math.hypot(dx, dy);
    }
}
