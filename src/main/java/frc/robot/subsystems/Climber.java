// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputs;
import frc.robot.Constants.MotorPorts;

public class Climber extends SubsystemBase {
  private static final double climberSpeed = 0.6;
  private final CANSparkMax climberMotor = new CANSparkMax(MotorPorts.motorClimber, MotorType.kBrushless); 
  private final DigitalInput magnet1 = new DigitalInput(DigitalInputs.ClimberMagnet1);
  private final DigitalInput magnet2 = new DigitalInput(DigitalInputs.ClimberMagnet2);
  /** Creates a new Climber. */
  public Climber() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber at top", atTop());
  }

  public boolean atBottom(){
    return !(magnet1.get());
  }

  public boolean atTop(){
    return !(magnet2.get());
  }

  public void extend(){
    climberMotor.set(climberSpeed);
  }

  public void retract(){
    climberMotor.set(-climberSpeed);
  }
  
  public void stop(){
    climberMotor.set(0);
  }
}
