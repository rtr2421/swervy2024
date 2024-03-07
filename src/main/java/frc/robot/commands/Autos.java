// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */


  /**
   * Creates an Autonomous command where we drive forward 2 meters
   * @param swerveSubsystem
   * @return
   */
  public static Command driveForward(SwerveSubsystem swerveSubsystem) {
    // return new DriveDistance(swerveSubsystem, 2);
    return new PathPlannerAuto("sean1");
  }
  /**
   * Creates an autonomous command where we:
   * - Shoot a high shot
   * - Drive and pick up a piece
   * - Drive back and shoot a high shot
   * 
   * @param swerveSubsystem
   * @param shooter
   * @param intake
   * @param indexer
   * @return
   */
  public static Command shoot2Pieces(SwerveSubsystem swerveSubsystem, Shooter shooter, Intake intake, Indexer indexer) {
    return Commands.sequence(new ShootNote(shooter, indexer, true), 
      new DriveDistance(swerveSubsystem, 2), 
      new RunIntake(indexer, intake), 
      new DriveDistance(swerveSubsystem, -2), 
      new ShootNote(shooter, indexer, true));
  }





   public static Command shoot2HighShots(){
    return new PathPlannerAuto("shoot2HighShots");
  }

  
  public static Command centerFarHoop(){
    return new PathPlannerAuto("centerFarHoop");
  }

  public static Command leftShoot3Shots(){
    return new PathPlannerAuto("leftShoot3shots");
  }
  
  public static Command simpleAutoRight() {
    return new PathPlannerAuto("simpleAutoRight");
  }

  public static Command simpleAutoLeft() {
    return new PathPlannerAuto("simpleAutoLeft");
  }

  public static Command disruptCenterRight() {
    return new PathPlannerAuto("disruptCenterRight");
  }

  public static Command rightShoot3Shots() {
    return new PathPlannerAuto("rightShoot3Shots");
  }







  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
  
 
}
