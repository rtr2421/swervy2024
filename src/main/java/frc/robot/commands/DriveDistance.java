// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveDistance extends Command {
  private SwerveSubsystem swerveSubsystem;
  private Translation2d starting;
  private double distance;

  /** Creates a new DriveDistance. */
  public DriveDistance(SwerveSubsystem swerveSubsystem, int distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.distance = distance;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    starting = swerveSubsystem.getPose().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.drive(new ChassisSpeeds(0.025,0,0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return starting.getDistance(swerveSubsystem.getPose().getTranslation()) > distance;
  }
}
