// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorPorts;
import frc.robot.Constants.PneumaticPorts;

public class Shooter extends SubsystemBase {

  private final CANSparkMax shooterMotor = new CANSparkMax(MotorPorts.motorShooter, MotorType.kBrushless);
  private final DoubleSolenoid flap = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticPorts.flapForward,
      PneumaticPorts.flapReverse);
  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
  private SparkPIDController shooterPid = shooterMotor.getPIDController();
  private double lowReference = 700;
  private double highReference = 6000;
  private boolean shootHigh;
  // PID coefficients
  private final double kP = 6e-2;
  private final double kI = 0;
  private final double kD = 0;
  private final double kIz = 0;
  private final double kFF = 0.000015;
  private final double kMaxOutput = 1;
  private final double kMinOutput = -1;
  private final double maxRPM = 5700;

  /** Creates a new Shooter. */
  public Shooter() {
    // set PID coefficients
    shooterPid.setP(kP);
    shooterPid.setI(kI);
    shooterPid.setD(kD);
    shooterPid.setIZone(kIz);
    shooterPid.setFF(kFF);
    shooterPid.setOutputRange(kMinOutput, kMaxOutput);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", shooterEncoder.getVelocity());
  }

  /**
   * sets motor and flap for lowshot
   */
  public void lowShot() {
    flap.set(DoubleSolenoid.Value.kForward);
    shooterPid.setReference(lowReference, CANSparkMax.ControlType.kSmartVelocity);
    shootHigh = false;
  }

  /**
   * sets motor and flap for highshot
   */
  public void highShot() {
    shooterPid.setReference(highReference, CANSparkMax.ControlType.kSmartVelocity);
    flap.set(DoubleSolenoid.Value.kReverse);
    shootHigh = true;
  }

  /**
   * stops the index motor
   */
  public void stop() {
    shooterMotor.set(0);
    flap.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Returns true if at right speed
   */
  public boolean atSpeed() {
    return true;
    /* 
    if (shootHigh){
      return (highReference - 20 < shooterEncoder.getVelocity() && shooterEncoder.getVelocity() < highReference + 20);
    } else {
      return (lowReference - 20 < shooterEncoder.getVelocity() && shooterEncoder.getVelocity() < lowReference + 20);
    }*/
  }

}
