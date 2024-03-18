// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.PneumaticPorts;

public class Shooter extends SubsystemBase {

  private final CANSparkMax shooterMotor1 = new CANSparkMax(CANIDs.motorShooter1, MotorType.kBrushless);
  private final CANSparkMax shooterMotor2 = new CANSparkMax(CANIDs.motorShooter2, MotorType.kBrushless);
  private final DoubleSolenoid flap = new DoubleSolenoid(
    CANIDs.REVPHCompressor, 
    PneumaticsModuleType.REVPH, 
    PneumaticPorts.tongueForward, 
    PneumaticPorts.tongueReverse);
  
   private final Compressor tongueCompressor = new Compressor(CANIDs.REVPHCompressor, PneumaticsModuleType.REVPH);
  private final RelativeEncoder shooterEncoder1 = shooterMotor1.getEncoder();
  private SparkPIDController shooterPid1 = shooterMotor1.getPIDController();
  private final RelativeEncoder shooterEncoder2 = shooterMotor2.getEncoder();
  private SparkPIDController shooterPid2 = shooterMotor2.getPIDController();

  private double lowReference = 550;
  private double highReference = 5500;
  private boolean shootHigh;
  // PID coefficients
  // private final double kP = 6e-2;
  private final double kP = 0.001;
  private final double kI = 0;
  private final double kD = 0;
  private final double kIz = 0;
  // private final double kFF = 0.000015;
  private final double kFF = 0.0;
  private final double kMaxOutput = 1;
  private final double kMinOutput = -1;
  private final double maxRPM = 5700;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterPid1.setP(kP);
    shooterPid1.setI(kI);
    shooterPid1.setD(kD);
    shooterPid1.setIZone(kIz);
    shooterPid1.setFF(kFF);
    shooterPid1.setOutputRange(kMinOutput, kMaxOutput);
    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterPid2.setP(kP);
    shooterPid2.setI(kI);
    shooterPid2.setD(kD);
    shooterPid2.setIZone(kIz);
    shooterPid2.setFF(kFF);
    shooterPid2.setOutputRange(kMinOutput, kMaxOutput);
    shooterMotor2.setIdleMode(IdleMode.kCoast);
    tongueCompressor.enableDigital();
    shooterPid1.setFeedbackDevice(shooterEncoder1);
    shooterPid2.setFeedbackDevice(shooterEncoder2);
    // shooterEncoder1.setVelocityConversionFactor(1);
    // shooterEncoder2.setVelocityConversionFactor(1);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("MotorShooterSpeed1", shooterEncoder1.getVelocity());
    SmartDashboard.putNumber("MotorShooterSpeed2", shooterEncoder2.getVelocity());
  }

  /**
   * sets motor and flap for lowshot
   */
  public void lowShot() {
    flap.set(DoubleSolenoid.Value.kForward);
    shooterMotor1.set(-0.1);
    shooterMotor2.set(0.1);
    //shooterPid.setReference(lowReference, CANSparkMax.ControlType.kVelocity);
    // shootHigh = false;
  }

  /**
   * sets motor and flap for highshot
   */
  public void highShot() {
    shooterPid1.setReference(-highReference, CANSparkMax.ControlType.kVelocity);
    shooterPid2.setReference(highReference, CANSparkMax.ControlType.kVelocity);
    // shooterMotor1.set(-1);
    // shooterMotor2.set(1);
    flap.set(DoubleSolenoid.Value.kReverse);
    // shootHigh = true;
  }

  public void setVelocity(double velocity1, double velocity2) {
    shooterPid1.setReference(-velocity1, CANSparkMax.ControlType.kVelocity);
    shooterPid2.setReference(velocity2, CANSparkMax.ControlType.kVelocity);
    System.out.println("setting shooter velocity1 " + velocity1 + ". velocity2 " + velocity2);
  }

  public void setP(double p){
    shooterPid1.setP(p);
    shooterPid2.setP(p);
  }

  public void extendTongue() {
    flap.set(DoubleSolenoid.Value.kForward);
  }

  public void retractTongue() {
    flap.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * stops all of the motors
   */
  public void stop() {
    shooterMotor1.set(0);
    shooterMotor2.set(0);
    flap.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Returns true if at right speed
   */
  public boolean atSpeed() {
    
    
    /*if (shootHigh){
      return (highReference - 20 < shooterEncoder1.getVelocity() && shooterEncoder2.getVelocity() < highReference + 20);
    } else {
      return (lowReference - 20 < shooterEncoder1.getVelocity() && shooterEncoder2.getVelocity() < lowReference + 20);
    }*/
    if (shootHigh){
      return (highReference < shooterEncoder1.getVelocity() && shooterEncoder2.getVelocity() > highReference);
    } else {
      return (lowReference < shooterEncoder1.getVelocity() && shooterEncoder2.getVelocity() > lowReference);
    }
  }
  public boolean atlowSpeed() {
    if (shootHigh){
      return (highReference-50 > shooterEncoder1.getVelocity() && shooterEncoder2.getVelocity() < highReference-50);
    } else {
      return (lowReference-50 > shooterEncoder1.getVelocity() && shooterEncoder2.getVelocity() < lowReference-50);
    }

  }

}
