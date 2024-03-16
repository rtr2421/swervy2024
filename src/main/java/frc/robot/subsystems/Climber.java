// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputs;
import frc.robot.Constants.CANIDs;

public class Climber extends SubsystemBase {
  private static final double climberSpeed = -1.0;
  private boolean goingUpwards;
  public enum ClimberStateEnum {goingUp, goingDown, nothing};
  ClimberStateEnum climberstate = ClimberStateEnum.nothing;
  private final CANSparkMax climberMotor = new CANSparkMax(CANIDs.motorClimber, MotorType.kBrushless); 
  private final DigitalInput magnet1 = new DigitalInput(DigitalInputs.ClimberMagnet1);
  private final DigitalInput magnet2 = new DigitalInput(DigitalInputs.ClimberMagnet2);

  

  /** Creates a new Climber. */
  public Climber() {
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -460);

    climberMotor.getEncoder().setPosition(0);
    climberMotor.setIdleMode(IdleMode.kBrake);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber at top", atTop());
    SmartDashboard.putString( "Climber Status", status());
  }

  public ClimberStateEnum getClimberState(){
    return climberstate;
  }
  public boolean atBottom(){
    return !(magnet1.get());
  }

  private String status(){
    if (climberstate == ClimberStateEnum.goingDown){
      return "Climber is going Down";
    } else if (climberstate == ClimberStateEnum.goingUp){
      return "Climber is going Up";
    } else{
      return "Climber is not moving";
    }
  }

  public boolean atTop(){
    return !(magnet2.get());
  }

  public void extend(){
    climberMotor.set(climberSpeed);
    climberstate = ClimberStateEnum.goingUp;
  }

  public void retract(){
    climberMotor.set(-climberSpeed);
    climberstate = ClimberStateEnum.goingDown;
  }

  public void setSafety(boolean safe) {
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, safe);
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, safe);
  }


  public void toggle(){
    if (getClimberState() == ClimberStateEnum.goingUp || getClimberState()  == ClimberStateEnum.nothing){
      retract();
    } else{
      extend();
    }
  }
  
  public void stop(){
    climberMotor.set(0);
   // climberstate = ClimberStateEnum.nothing;
  }

}
