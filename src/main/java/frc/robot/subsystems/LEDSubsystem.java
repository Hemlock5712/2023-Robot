package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.enums.GamePiece;

public class LEDSubsystem extends SubsystemBase {

  private Spark blinkin = new Spark(0);

  private boolean isHoldingGamePiece = false;

  private GamePiece gamePiece = GamePiece.NONE;

  private double currentColor = BlinkinPatterns.GOLD;

  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {

  }

  public void setGamePiece(GamePiece gamePiece) {
    this.gamePiece = gamePiece;
    if (gamePiece != GamePiece.NONE) {
      if (gamePiece == GamePiece.CONE) {
        currentColor = BlinkinPatterns.GOLD;
      } else if (gamePiece == GamePiece.CUBE) {
        currentColor = BlinkinPatterns.PURPLE;
      }
    }
  }

  public void setIsHoldingGamePiece(boolean holdingGamePiece) {
    isHoldingGamePiece = holdingGamePiece;
  }

  public GamePiece getGamePiece() {
    return this.gamePiece;
  }

  public void idleAnimation() {
    currentColor = BlinkinPatterns.END_TO_END;
  }

  public void setLEDColor(double color) {
    currentColor = color;
  }

  @Override
  public void periodic() {
    if (isHoldingGamePiece) {
      blinkin.set(BlinkinPatterns.GREEN);
    } else {
      blinkin.set(currentColor);
    }
  }
}

class BlinkinPatterns {
  public static final double RAINBOW_RAINBOW_PALETTE = -0.99;
  public static final double RAINBOW_CONFETTI_PALETTE = -0.87;
  public static final double TWINKLE_RAINBOW = -0.55;
  public static final double GOLD = 0.67;
  public static final double YELLOW = 0.69;
  public static final double PURPLE = 0.91;
  public static final double GREEN = 0.77;
  public static final double END_TO_END = 0.45;
}
