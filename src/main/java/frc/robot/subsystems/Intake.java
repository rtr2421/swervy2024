// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputs;
import frc.robot.Constants.CANIDs;

public class Intake extends SubsystemBase {
  private boolean isRunning = false;
  private static final double UPPERSPEED = -1.0;
  private static final double LOWERSPEED = -1.0;
  private final DigitalInput beam = new DigitalInput(DigitalInputs.NoteSensor);
  private final WPI_TalonSRX upperMotor = new WPI_TalonSRX(CANIDs.upperIntake);
  private final WPI_TalonSRX lowerMotor = new WPI_TalonSRX(CANIDs.motorLower);
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean( "Has Game Piece?", isLoaded());
    SmartDashboard.putBoolean( "Is Intake Running?", isRunning);
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
    isRunning = true;
  }

  /** Runs the intake in reverse, such as to push a stuck piece out */
  public void reverse() {
    upperMotor.set(-UPPERSPEED);
    lowerMotor.set(-LOWERSPEED);
  }


  /**
   * Stops intake
   */

  public void stop(){
    upperMotor.set(0);
    lowerMotor.set(0);
    isRunning = false;
  }

}
