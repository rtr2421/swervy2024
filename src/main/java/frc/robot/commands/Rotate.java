// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class Rotate extends Command {
  


  private SwerveSubsystem swerveSubsystem;
  private int angle;
  private Rotation2d starting;

  /** Creates a new Rotate. */
  public Rotate(SwerveSubsystem swerveSubsystem, int angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (angle > 0){
      swerveSubsystem.drive(new ChassisSpeeds(0,0, 0.025));
    } else{
      swerveSubsystem.drive(new ChassisSpeeds(0,0,-0.025));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
