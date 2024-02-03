// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorPorts;
import frc.robot.Constants.PneumaticPorts;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}
  private final CANSparkMax shooterMotor = new CANSparkMax(MotorPorts.motorShooter, MotorType.kBrushless); 
  private final DoubleSolenoid flap = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticPorts.flapForward, PneumaticPorts.flapReverse);
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * sets motor and flap for lowshot
   */
  public void lowShot(){
    shooterMotor.set(0.5);
    flap.set(DoubleSolenoid.Value.kForward);
  }
  /**
   * sets motor and flap for highshot
   */
  public void highShot(){
    shooterMotor.set(1);
    flap.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * stops the index motor
   */
  public void stop(){
    shooterMotor.set(0);
    flap.set(DoubleSolenoid.Value.kReverse);
  }
  /**
   * Returns true if at right speed
   */
  public boolean atSpeed(){
    return true;
  }
}
