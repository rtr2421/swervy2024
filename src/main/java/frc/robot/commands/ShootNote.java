// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {

  public ShootNote(Shooter shooter, Indexer indexer, boolean highShot) {
    
    if (highShot) {
      addCommands(new InstantCommand(()-> shooter.highShot())
        .andThen(new WaitUntilCommand(shooter::atSpeed).withTimeout(1))
        .andThen(new InstantCommand(()-> indexer.startShooting()))
        .andThen(new WaitUntilCommand(shooter::atlowSpeed).withTimeout(0.5))
        .andThen(new WaitUntilCommand(shooter::atSpeed).withTimeout(0.5)));
  
      } else { 
        addCommands(new InstantCommand(()-> shooter.lowShot())
        // .andThen(new WaitUntilCommand(shooter::atlowSpeed)).withTimeout(0.5)
        .andThen(new InstantCommand(()-> indexer.startShootingLow()))
        .andThen(new WaitCommand(1.2))
        .andThen(new InstantCommand(() -> shooter.retractTongue()))
        .andThen(new WaitCommand(0.3))
        .andThen(new InstantCommand(() -> shooter.extendTongue()))
        .andThen(new WaitCommand(1))
        // .andThen(new InstantCommand(() -> shooter.retractTongue()))
        );
        
      }

    
  }


}
