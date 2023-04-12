package frc.robot.pathfind;

import java.util.List;

public class MapCreator {
  public void createGraph(VisGraph AStarMap, List<Obstacle> obstacles) {
    AStarMap.addNode(new Node(2.40 - 0.1, 4.75));
    AStarMap.addNode(new Node(5.40 + 0.1, 4.75));
    AStarMap.addNode(new Node(5.40 + 0.1, 0.75));
    AStarMap.addNode(new Node(2.40 - 0.1, 0.75));
    // Divider
    //AStarMap.addNode(new Node(3.8 + 0.1, 4.75));

    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
      Node startNode = AStarMap.getNode(i);
      for (int j = i + 1; j < AStarMap.getNodeSize(); j++) {
        AStarMap.addEdge(new Edge(startNode, AStarMap.getNode(j)), obstacles);
      }
    }
  }
}
