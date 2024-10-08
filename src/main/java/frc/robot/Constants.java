// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; 
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kHelperControllerPort = 1;
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
  }
  public static class PneumaticPorts{

    public static final int tongueForward = 0;
    public static final int tongueReverse = 1;

  }
  public static class CANIDs{
    /**
     * 1-12 on drive train
     */
    
    public static final int upperIntake = 13;
    public static final int motorLower = 14;
    public static final int motorIndexer = 15;
    public static final int motorShooter1 = 16;
    public static final int motorShooter2 = 18;
    public static final int motorClimber = 17;
    
    public static final int REVPHCompressor = 19;

    
  }

  public static class DigitalInputs {
    public static final int NoteSensor = 0;
    public static final int ClimberMagnet1 = 1;
    public static final int ClimberMagnet2 = 2;

  
    
  }
}
