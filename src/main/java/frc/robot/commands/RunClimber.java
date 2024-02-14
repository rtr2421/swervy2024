// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberStateEnum;

public class RunClimber extends Command {
  private Climber climber;

  
  
  /** Creates a new RunClimber. */
  public RunClimber(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setMovement();
  
  }
  private void setMovement() {
    if (climber.atBottom() == true){
      climber.extend();
    } else if(climber.atTop() == true){
      climber.retract();
    } else{
      climber.toggle();
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // System.out.println("execute run climber");
      if (climber.getClimberState() == ClimberStateEnum.goingUp){
      climber.extend();
      
      } else{
        climber.retract();
      }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((ClimberStateEnum.goingUp == climber.getClimberState() && climber.atTop()) || (ClimberStateEnum.goingDown == climber.getClimberState() && climber.atBottom()));
  }
}
