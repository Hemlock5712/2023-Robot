package frc.robot.aStar;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

class Node implements Comparable<Node> 
{
  private int g;
  private int f;
  private int h;
  private int row;
  private int col;
  private boolean isBlock;
  private Node parent;

  public Node(int row, int col) {
    this.row = row;
    this.col = col;
  }

  @Override
  public int compareTo(Node other)
  {
    return Integer.compare(f, other.f);
  }

  public void calculateHeuristic(Node finalNode) {
    h = Math.abs(finalNode.row - row) + Math.abs(finalNode.col - col);
  }

  public void setNodeData(Node currentNode, int cost) {
    parent = currentNode;
    g = currentNode.g + cost;
    f = g + h;
  }

  public boolean checkBetterPath(Node currentNode, int cost) {
    int gCost = currentNode.g + cost;
    if (gCost < this.g) {
      setNodeData(currentNode, cost);
      return true;
    }
    return false;
  }

  @Override
  public boolean equals(Object arg0) {
    if (arg0 == this)
      return true;

    if (arg0 == null || arg0.getClass() != getClass())
      return false;

    Node other = (Node) arg0;
    return row == other.row && col == other.col;
  }

  @Override
  public int hashCode()
  {
    return 31 * row + col;
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
  private static final int DEFAULT_HV_COST = 100; // Horizontal - Vertical Cost
  private static final int DEFAULT_DIAGONAL_COST = 141;
  private int hvCost;
  private int diagonalCost;
  private Node[][] searchArea;
  private PriorityQueue<Node> openList;
  private Set<Node> closedSet;
  private Node initialNode;
  private Node finalNode;

  public AStar(int rows, int cols, Node initialNode, Node finalNode, int hvCost, int diagonalCost) {
    this.hvCost = hvCost;
    this.diagonalCost = diagonalCost;
    this.initialNode = initialNode;
    this.finalNode = finalNode;

    searchArea = new Node[rows][cols];
    openList = new PriorityQueue<>();
    closedSet = new HashSet<>();
    setNodes();
  }

  public AStar(int rows, int cols, Node initialNode, Node finalNode) {
    this(rows, cols, initialNode, finalNode, DEFAULT_HV_COST, DEFAULT_DIAGONAL_COST);
  }

  private void setNodes() {
    for (int i = 0; i < searchArea.length; i++) {
      for (int j = 0; j < searchArea[0].length; j++) {
        Node node = new Node(i, j);
        node.calculateHeuristic(finalNode);
        searchArea[i][j] = node;
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
    while (!openList.isEmpty()) {
      Node currentNode = openList.poll();
      closedSet.add(currentNode);

      if (isFinalNode(currentNode))
        return getPath(currentNode);
      else
        addAdjacentNodes(currentNode);
    }
    return new ArrayList<>();
  }

  private List<Node> getPath(Node currentNode) {
    List<Node> path = new ArrayList<>();
    path.add(currentNode);

    Node parentOfTopNode = currentNode.getParent();
    while (parentOfTopNode != null) {
      path.add(0, parentOfTopNode);
      parentOfTopNode = parentOfTopNode.getParent();
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
    if (lowerRow < searchArea.length) {
      if (col - 1 >= 0) {
        checkNode(currentNode, col - 1, lowerRow, diagonalCost); // Comment this line if diagonal movements are not
                                                                      // allowed
      }
      if (col + 1 < searchArea[0].length) {
        checkNode(currentNode, col + 1, lowerRow, diagonalCost); // Comment this line if diagonal movements are not
                                                                      // allowed
      }
      checkNode(currentNode, col, lowerRow, hvCost);
    }
  }

  private void addAdjacentMiddleRow(Node currentNode) {
    int row = currentNode.getRow();
    int col = currentNode.getCol();
    int middleRow = row;
    if (col - 1 >= 0) {
      checkNode(currentNode, col - 1, middleRow, hvCost);
    }
    if (col + 1 < searchArea[0].length) {
      checkNode(currentNode, col + 1, middleRow, hvCost);
    }
  }

  private void addAdjacentUpperRow(Node currentNode) {
    int row = currentNode.getRow();
    int col = currentNode.getCol();
    int upperRow = row - 1;
    if (upperRow >= 0) {
      if (col - 1 >= 0) {
        checkNode(currentNode, col - 1, upperRow, diagonalCost); // Comment this if diagonal movements are not
                                                                      // allowed
      }
      if (col + 1 < searchArea[0].length) {
        checkNode(currentNode, col + 1, upperRow, diagonalCost); // Comment this if diagonal movements are not
                                                                      // allowed
      }
      checkNode(currentNode, col, upperRow, hvCost);
    }
  }

  private void checkNode(Node currentNode, int col, int row, int cost) {
    Node adjacentNode = searchArea[row][col];
    if (!adjacentNode.isBlock() && !closedSet.contains(adjacentNode)) 
    {
      if (!openList.contains(adjacentNode)) 
      {
        adjacentNode.setNodeData(currentNode, cost);
        openList.add(adjacentNode);
      } 
      else if (adjacentNode.checkBetterPath(currentNode, cost)) 
      {
          // Remove and Add the changed node, so that the PriorityQueue can sort again its
          // contents with the modified "finalCost" value of the modified node
          openList.remove(adjacentNode);
          openList.add(adjacentNode);
      }
    }
  }

  private boolean isFinalNode(Node currentNode) {
    return currentNode.equals(finalNode);
  }

  private void setBlock(int row, int col) {
    searchArea[row][col].setBlock(true);
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
    return Arrays.copyOf(searchArea, searchArea.length);
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
}

public class Pathfinding 
{
  private Pathfinding()
  {
    throw new IllegalStateException("Pathfinding is a helper class");
  }

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
    List<Translation2d> finalPath = new ArrayList<>(path.size());

    // Use a for loop since it's faster than streams.
    for (int i = 0; i < path.size(); i++)
    {
      Node p = path.get(i);
      finalPath.set(i, new Translation2d(Units.inchesToMeters(p.getCol() * 10), Units.inchesToMeters(p.getRow() * 10)));
    }
    return finalPath;
  }
}