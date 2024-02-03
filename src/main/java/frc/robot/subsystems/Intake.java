// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputs;
import frc.robot.Constants.MotorPorts;

public class Intake extends SubsystemBase {
  private static final double UPPERSPEED = -0.5;
  private static final double LOWERSPEED = 0.5;
  private final DigitalInput beam = new DigitalInput(DigitalInputs.NoteSensor);
  private final WPI_TalonSRX upperMotor = new WPI_TalonSRX(MotorPorts.upperIntake);
  private final WPI_TalonSRX lowerMotor = new WPI_TalonSRX(MotorPorts.motorLower);
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * Returns true if holding a playing piece
   */
  public boolean isLoaded(){
    return !(beam.get());
  }

  /**
   * Tells intake to start 
   */
  public void start(){
    upperMotor.set(UPPERSPEED);
    lowerMotor.set(LOWERSPEED);
  }
  /**
   * Stops intake
   */

  public void stop(){
    upperMotor.set(0);
    lowerMotor.set(0);

  }

}
