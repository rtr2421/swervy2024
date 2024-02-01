// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private static final int motorIndexer = 15;
  private final CANSparkMax indexMotor = new CANSparkMax(motorIndexer, MotorType.kBrushless); 
  /** Creates a new Index. */
  public Indexer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * starts the index motor
   */
  public void start(){
    indexMotor.set(1);
  }

  /**
   * stops the index motor
   */
  public void stop(){
    indexMotor.set(0);
  }
}
