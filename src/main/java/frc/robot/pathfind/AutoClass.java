// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfind;

/** Add your docs here. */
public class AutoClass {
  Node node;
  int level;
  GamePiece gamePiece;

  /**
   * If level is -1 then it is a preplaced; game piece 2 is cube, 1 is cone
   */
  public AutoClass(Node node, int level, GamePiece gamePiece) {
    this.node = node;
    this.gamePiece = gamePiece;
    this.level = level;
  }

  public GamePiece getGameObject() {
    return gamePiece;
  }

  public Node getNode() {
    return node;
  }

}