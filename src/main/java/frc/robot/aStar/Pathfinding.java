package frc.robot.aStar;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

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
    this.hvCost = hvCost;
    this.diagonalCost = diagonalCost;
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

  private int getDiagonalCost() {
    return diagonalCost;
  }

  private void setDiagonalCost(int diagonalCost) {
    this.diagonalCost = diagonalCost;
  }
}

public class Pathfinding {

  public static List<Translation2d> generatePath(int startX, int startY, int endX, int endY) {
    Node initialNode = new Node((int) (Units.metersToInches(startX) / 10), (int) (Units.metersToInches(startY) / 10));
    Node finalNode = new Node((int) (Units.metersToInches(endX) / 10), (int) (Units.metersToInches(endY) / 10));
    int rows = 32;
    int cols = 65;
    AStar aStar = new AStar(rows, cols, initialNode, finalNode);

    // Put in blocks Array to block cells {0, 1}, {1, 1}
    int[][] blocksArray = new int[][] {};

    aStar.setBlocks(blocksArray);
    List<Node> path = aStar.findPath();
    List<Node> midPoints = path.subList(2, path.size() - 1);
    return midPoints.stream()
        .map((p) -> new Translation2d(Units.inchesToMeters(p.getRow() * 10), Units.inchesToMeters(p.getCol() * 10)))
        .collect(Collectors.toList());
  }
}