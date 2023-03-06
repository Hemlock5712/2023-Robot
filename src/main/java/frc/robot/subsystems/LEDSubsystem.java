package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.enums.GamePiece;

public class LEDSubsystem extends SubsystemBase {

    private AddressableLED led = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(120);

    private GamePiece gamePiece = GamePiece.NONE;

    private int firstPixelHue = 0;

    /**
     * Creates a new LEDSubsystem.
     */
    public LEDSubsystem() {
        led.setLength(buffer.getLength());
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
        if(gamePiece != GamePiece.NONE) {
            for (int i = 0; i < buffer.getLength(); i++) {
                if (gamePiece == GamePiece.CONE) {
                    buffer.setRGB(i, 255, 255, 0);
                } else if (gamePiece == GamePiece.CUBE) {
                    buffer.setRGB(i, 150, 0, 235);
                }
            }
        }
    }

    public void idleAnimation() {
        if(gamePiece == GamePiece.NONE) {
            for (var i = 0; i < buffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (firstPixelHue + (i * 180 / buffer.getLength())) % 180;
                // Set the value
                buffer.setHSV(i, hue, 255, 128);
            }
            firstPixelHue += 3;
            // Check bounds
            firstPixelHue %= 180;
        }
    }

    public void setLEDColor(int index, int red, int green, int blue) {
        buffer.setRGB(index, red, green, blue);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        idleAnimation();
        led.setData(buffer);
    }
}
