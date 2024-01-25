// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}
  private final CANSparkMax shooterMotor = new CANSparkMax(16, MotorType.kBrushless); 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void start(){
    shooterMotor.set(1);
  }

  /**
   * stops the index motor
   */
  public void stop(){
    shooterMotor.set(0);
  }
}
