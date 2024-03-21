// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private Indexer indexer;
  private Intake intake;
  
  /** Creates a new RunIntake. */
  public RunIntake(Indexer indexer, Intake intake) {
    addRequirements(indexer, intake);
    this.indexer = indexer;
    this.intake = intake;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.start();
    indexer.startIntaking();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    indexer.stop();
  }

  // Returns true when the playing piece is detected.
  @Override
  public boolean isFinished() {
  return intake.isLoaded();
   
  }
}
