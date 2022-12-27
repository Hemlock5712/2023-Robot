package frc.robot.commands;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

class Node {

  private int g;
  private int f;
  private int h;
  private int row;
  private int col;
  private boolean isBlock;
  private Node parent;

  public Node(int row, int col) {
    super();
    this.row = row;
    this.col = col;
  }

  public void calculateHeuristic(Node finalNode) {
    this.h = Math.abs(finalNode.getRow() - getRow()) + Math.abs(finalNode.getCol() - getCol());
  }

  public void setNodeData(Node currentNode, int cost) {
    int gCost = currentNode.getG() + cost;
    setParent(currentNode);
    setG(gCost);
    calculateFinalCost();
  }

  public boolean checkBetterPath(Node currentNode, int cost) {
    int gCost = currentNode.getG() + cost;
    if (gCost < getG()) {
      setNodeData(currentNode, cost);
      return true;
    }
    return false;
  }

  private void calculateFinalCost() {
    int finalCost = getG() + getH();
    setF(finalCost);
  }

  @Override
  public boolean equals(Object arg0) {
    Node other = (Node) arg0;
    return this.getRow() == other.getRow() && this.getCol() == other.getCol();
  }

  @Override
  public String toString() {
    return "Node [row=" + row + ", col=" + col + "]";
  }

  public int getH() {
    return h;
  }

  public void setH(int h) {
    this.h = h;
  }

  public int getG() {
    return g;
  }

  public void setG(int g) {
    this.g = g;
  }

  public int getF() {
    return f;
  }

  public void setF(int f) {
    this.f = f;
  }

  public Node getParent() {
    return parent;
  }

  public void setParent(Node parent) {
    this.parent = parent;
  }

  public boolean isBlock() {
    return isBlock;
  }

  public void setBlock(boolean isBlock) {
    this.isBlock = isBlock;
  }

  public int getRow() {
    return row;
  }

  public void setRow(int row) {
    this.row = row;
  }

  public int getCol() {
    return col;
  }

  public void setCol(int col) {
    this.col = col;
  }
}

class AStar {
  private static int DEFAULT_HV_COST = 100; // Horizontal - Vertical Cost
  private static int DEFAULT_DIAGONAL_COST = 141;
  private int hvCost;
  private int diagonalCost;
  private Node[][] searchArea;
  private PriorityQueue<Node> openList;
  private Set<Node> closedSet;
  private Node initialNode;
  private Node finalNode;

  public AStar(int rows, int cols, Node initialNode, Node finalNode, int hvCost, int diagonalCost) {
    setHvCost(hvCost);
    setDiagonalCost(diagonalCost);
    setInitialNode(initialNode);
    setFinalNode(finalNode);
    this.searchArea = new Node[rows][cols];
    this.openList = new PriorityQueue<Node>(new Comparator<Node>() {
      @Override
      public int compare(Node node0, Node node1) {
        return Integer.compare(node0.getF(), node1.getF());
      }
    });
    setNodes();
    this.closedSet = new HashSet<>();
  }

  public AStar(int rows, int cols, Node initialNode, Node finalNode) {
    this(rows, cols, initialNode, finalNode, DEFAULT_HV_COST, DEFAULT_DIAGONAL_COST);
  }

  private void setNodes() {
    for (int i = 0; i < searchArea.length; i++) {
      for (int j = 0; j < searchArea[0].length; j++) {
        Node node = new Node(i, j);
        node.calculateHeuristic(getFinalNode());
        this.searchArea[i][j] = node;
      }
    }
  }

  public void setBlocks(int[][] blocksArray) {
    for (int i = 0; i < blocksArray.length; i++) {
      int row = blocksArray[i][0];
      int col = blocksArray[i][1];
      setBlock(row, col);
    }
  }

  public List<Node> findPath() {
    openList.add(initialNode);
    while (!isEmpty(openList)) {
      Node currentNode = openList.poll();
      closedSet.add(currentNode);
      if (isFinalNode(currentNode)) {
        return getPath(currentNode);
      } else {
        addAdjacentNodes(currentNode);
      }
    }
    return new ArrayList<Node>();
  }

  private List<Node> getPath(Node currentNode) {
    List<Node> path = new ArrayList<Node>();
    path.add(currentNode);
    Node parent;
    while ((parent = currentNode.getParent()) != null) {
      path.add(0, parent);
      currentNode = parent;
    }
    return path;
  }

  private void addAdjacentNodes(Node currentNode) {
    addAdjacentUpperRow(currentNode);
    addAdjacentMiddleRow(currentNode);
    addAdjacentLowerRow(currentNode);
  }

  private void addAdjacentLowerRow(Node currentNode) {
    int row = currentNode.getRow();
    int col = currentNode.getCol();
    int lowerRow = row + 1;
    if (lowerRow < getSearchArea().length) {
      if (col - 1 >= 0) {
        checkNode(currentNode, col - 1, lowerRow, getDiagonalCost()); // Comment this line if diagonal movements are not
                                                                      // allowed
      }
      if (col + 1 < getSearchArea()[0].length) {
        checkNode(currentNode, col + 1, lowerRow, getDiagonalCost()); // Comment this line if diagonal movements are not
                                                                      // allowed
      }
      checkNode(currentNode, col, lowerRow, getHvCost());
    }
  }

  private void addAdjacentMiddleRow(Node currentNode) {
    int row = currentNode.getRow();
    int col = currentNode.getCol();
    int middleRow = row;
    if (col - 1 >= 0) {
      checkNode(currentNode, col - 1, middleRow, getHvCost());
    }
    if (col + 1 < getSearchArea()[0].length) {
      checkNode(currentNode, col + 1, middleRow, getHvCost());
    }
  }

  private void addAdjacentUpperRow(Node currentNode) {
    int row = currentNode.getRow();
    int col = currentNode.getCol();
    int upperRow = row - 1;
    if (upperRow >= 0) {
      if (col - 1 >= 0) {
        checkNode(currentNode, col - 1, upperRow, getDiagonalCost()); // Comment this if diagonal movements are not
                                                                      // allowed
      }
      if (col + 1 < getSearchArea()[0].length) {
        checkNode(currentNode, col + 1, upperRow, getDiagonalCost()); // Comment this if diagonal movements are not
                                                                      // allowed
      }
      checkNode(currentNode, col, upperRow, getHvCost());
    }
  }

  private void checkNode(Node currentNode, int col, int row, int cost) {
    Node adjacentNode = getSearchArea()[row][col];
    if (!adjacentNode.isBlock() && !getClosedSet().contains(adjacentNode)) {
      if (!getOpenList().contains(adjacentNode)) {
        adjacentNode.setNodeData(currentNode, cost);
        getOpenList().add(adjacentNode);
      } else {
        boolean changed = adjacentNode.checkBetterPath(currentNode, cost);
        if (changed) {
          // Remove and Add the changed node, so that the PriorityQueue can sort again its
          // contents with the modified "finalCost" value of the modified node
          getOpenList().remove(adjacentNode);
          getOpenList().add(adjacentNode);
        }
      }
    }
  }

  private boolean isFinalNode(Node currentNode) {
    return currentNode.equals(finalNode);
  }

  private boolean isEmpty(PriorityQueue<Node> openList) {
    return openList.size() == 0;
  }

  private void setBlock(int row, int col) {
    this.searchArea[row][col].setBlock(true);
  }

  public Node getInitialNode() {
    return initialNode;
  }

  public void setInitialNode(Node initialNode) {
    this.initialNode = initialNode;
  }

  public Node getFinalNode() {
    return finalNode;
  }

  public void setFinalNode(Node finalNode) {
    this.finalNode = finalNode;
  }

  public Node[][] getSearchArea() {
    return searchArea;
  }

  public void setSearchArea(Node[][] searchArea) {
    this.searchArea = searchArea;
  }

  public PriorityQueue<Node> getOpenList() {
    return openList;
  }

  public void setOpenList(PriorityQueue<Node> openList) {
    this.openList = openList;
  }

  public Set<Node> getClosedSet() {
    return closedSet;
  }

  public void setClosedSet(Set<Node> closedSet) {
    this.closedSet = closedSet;
  }

  public int getHvCost() {
    return hvCost;
  }

  public void setHvCost(int hvCost) {
    this.hvCost = hvCost;
  }

  private void setDiagonalCost(int diagonalCost) {
    this.diagonalCost = diagonalCost;
  }

  private int getDiagonalCost() {
    return diagonalCost;
  }


}

public class PathTesting {
  public static List<Translation2d> generatePath(double startX, double startY, double endX, double endY) {
    Node initialNode = new Node((int) (Units.metersToInches(startX) / 10), (int) (Units.metersToInches(startY) / 10));
    Node finalNode = new Node((int) (Units.metersToInches(endX) / 10), (int) (Units.metersToInches(endY) / 10));
    int rows = 32;
    int cols = 65;
    AStar aStar = new AStar(cols, rows, initialNode, finalNode);

    // Put in blocks Array to block cells {0, 1}, {1, 1}
    int[][] blocksArray = new int[][] {};

    aStar.setBlocks(blocksArray);
    List<Node> path = aStar.findPath();
    List<Node> midPoints = path.subList(2, path.size() - 1);
    List<Translation2d> points = midPoints.stream()
        .map((p) -> new Translation2d(Units.inchesToMeters(p.getCol() * 10), Units.inchesToMeters(p.getRow() * 10)))
        .collect(Collectors.toList());
    return points;
  }


  public static PathPlannerTrajectory PPPath(List<Translation2d> internalPoints, PathConstraints constraints, double startX, double startY, double startAngle, double endX, double endY, double endAngle){
    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    Rotation2d endRotationObj = Rotation2d.fromDegrees(endAngle);
    Translation2d startPoint = new Translation2d(startX, startY);
    Rotation2d holoRotaion =  Rotation2d.fromDegrees(startAngle);

    if (internalPoints.size() > 0)
    {
      PathPoint[] restOfPoints = new PathPoint[internalPoints.size() - 1];
      

      PathPoint secondPoint = new PathPoint(internalPoints.get(0), internalPoints.get(1).minus(internalPoints.get(0)).getAngle(), endRotationObj);

      for (int i = 0; i < internalPoints.size() - 1; i++)
      {
        double y1 = internalPoints.get(i+1).getY();
        double x1 = internalPoints.get(i+1).getX();
        double y0 = internalPoints.get(i).getY();
        double x0 = internalPoints.get(i).getX();
        double hypotenuse = Math.hypot(y1 - y0, x1 - x0);
        Rotation2d angleOfTangentLineToXAxis = new Translation2d(x0 / hypotenuse, y0 / hypotenuse).getAngle();
        restOfPoints[i] = new PathPoint(internalPoints.get(i + 1), angleOfTangentLineToXAxis, endRotationObj);
      }
      restOfPoints[restOfPoints.length - 1] = new PathPoint(new Translation2d(endX, endY), endRotationObj, endRotationObj);

      return PathPlanner.generatePath(constraints,
          new PathPoint(startPoint, internalPoints.get(0).minus(startPoint).getAngle(), holoRotaion),
          secondPoint,
          restOfPoints);
    } else {
          return PathPlanner.generatePath(constraints,
          new PathPoint(startPoint, internalPoints.get(0).minus(startPoint).getAngle(), holoRotaion),
          new PathPoint(new Translation2d(endX, endY), endRotationObj, endRotationObj));
    }
  }
  public static void main(String[] args) {
    double startX = 0;
    double startY = 0;
    double endX = 3;
    double endY = 3;
    double startAngle = 0;
    double endAngle = 0;
    PathConstraints constraints = new PathConstraints(2, 2);
    List<Translation2d> path = generatePath(startX, startY, endX, endY);
    PathPlannerTrajectory trajectory = PPPath(path, constraints, startX, startY, startAngle, endX, endY, endAngle);
    for(int i = 0; i<trajectory.getStates().size();i++){
      System.out.print("x: "+trajectory.getState(i).poseMeters.getX()+" y: "+trajectory.getState(i).poseMeters.getY()+",");
    }
  }
}