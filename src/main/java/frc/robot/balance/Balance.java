package frc.robot.balance;

import static frc.robot.Constants.DrivetrainConstants.PIGEON_ID;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.XboxController;

// TODO: Import DriveTrain and other code to handle positioning.


public class Balance {
  private final static WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID);
  private final static XboxController controller = new XboxController(0);

  // This is a temporary definition of distanceFromCenter until actual code is
  // implemented to detect the distance from the center of the Charging Station.
  private static double distanceFromCenter = 0.0;



  // This notates how many meters per second the robot should overshoot
  // the velocity for every meter away the robot is from the center of the
  // Charge Station. If you do want this to impact the velocity set this to zero.
  private static double velocityPerMeter = 1.0;

  private static double constantAddedVelocity = 0.0;

  // The range in meters from the center when the constantAddedVelocity will stop
  // being added to the total velocity.
  private static double posRangeConstantDisable = 0.0;

  // The range in radians from the 0 radians when the constantAddedVelocity
  // will stop being added to the total velocity.
  private static double pitchRangeConstantDisable = 0.0;

  // The range in meters from the center when all velocity will stop
  // being added.
  private static double posRangeDisable = 0.0;

  // The range in radians from the 0 radians when all velocity will stop
  // being added.
  private static double pitchRangeDisable = 0.0;



  // This code is only for testing! Remove for production!
  // --------------------------------------------------------------------------

  // Set to true if you want the force of gravity to impact the velocity of the
  // robot when going up the Charging Station.
  private boolean useGravity = true;

  // Set true you don't want constantAddedVelocity to be added within a certain
  // specified amount of meters from the center of the Charging Station.
  private boolean useDisableConstantOnPos = false;

  // Set true you don't want constantAddedVelocity to be added within a certain
  // specified amount of radians from 0 radians.
  private boolean useDisableConstantOnPitch = true;

  // Set true you don't want any velocity to be added within a certain
  // specified amount of meters from the center of the Charging Station.
  private boolean useDisableOnPos = false;

  // Set true you don't want any velocity to be added within a certain
  // specified amount of radians from 0 radians.
  private boolean useDisableOnPitch = true;

   // --------------------------------------------------------------------------



  // TODO: Make this function actually make sense and work on a button press. (Also just make it work in general)

  public Balance() {
    pigeon.configMountPosePitch(0);

    // TODO: Possibility - Let robot drive onto the drive Station without slowing down.
    // TODO: Make sure robot can actually go onto the Charge Station from location.
    // TODO: When button clicked drive onto the Charge Station.

    if (controller.getStartButton()) {
      double pitch = pigeon.getPitch();

      // TODO: Get distance from center point of Charging Station.

      double velocity;

      if (
        (useDisableOnPitch && ((-pitchRangeDisable < pitch) && (pitchRangeDisable > pitch))) ||
        (useDisableOnPos && ((-posRangeDisable < distanceFromCenter) && (posRangeDisable > distanceFromCenter)))
      ) {

        velocity = 0.0;

      } else {
        // Calculating how much the robot accelerates downward based upon gravity and the angle of the platform.
        double velocityOfGravity = 9.81 * Math.sin(pitch) * (useGravity ? 1.0 : 0.0); //Ternary operator is for testing! DON'T use in production!
        double velocityOfDistance = velocityPerMeter * distanceFromCenter;
        double velocityAdded;
        // Checking if useDisableConstantOnPitch or useDisableConstantOnPos are in use and if so
        // we will set the velocity added to zero otherwise set it to our velocity constant.
        if (
          (useDisableConstantOnPitch && ((-pitchRangeConstantDisable < pitch) && (pitchRangeConstantDisable > pitch))) ||
          (useDisableConstantOnPos && ((-posRangeConstantDisable < distanceFromCenter) && (posRangeConstantDisable > distanceFromCenter)))
          ) {
            velocityAdded = 0;
          } else {
            velocityAdded = constantAddedVelocity;
          }
        velocity = velocityOfGravity + velocityOfDistance + velocityAdded;
      }

      // TODO: Make Swerve Drive behave like normal arcade drive.
      // TODO: Send velocity to Drive Terrain.


    }
  }
}
