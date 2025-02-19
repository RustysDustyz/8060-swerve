// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SignedMotors extends SubsystemBase {
  private final Spark positiveMotor;
  private final Spark negativeMotor;

  public SignedMotors(Spark positiveMotor, Spark negativeMotor){
    this.positiveMotor = positiveMotor;
    this.negativeMotor = negativeMotor;
  }

  public void set(double speed){
    positiveMotor.set(speed > 0 ? speed : 0);
    negativeMotor.set(speed < 0 ? -speed : 0);
  }

  public void stop(){
    positiveMotor.stopMotor();
    negativeMotor.stopMotor();
  }

  @Override
  public void periodic() { 
  }
}
