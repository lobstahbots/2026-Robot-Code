// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
  /** Creates a new Intake. */
  private final SparkMax armMotor;
  private final SparkMax rollerMotor;

  private final RelativeEncoder armEncoder;
  private final RelativeEncoder rollerEncoder;


  public IntakeIOSparkMax(int armMotorID, int rollerMotorID) {
    this.armMotor = new SparkMax(armMotorID, MotorType.kBrushless);
    this.rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();

    config.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.encoder.velocityConversionFactor(1.0);
    
    armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armEncoder = armMotor.getEncoder();
    rollerEncoder = rollerMotor.getEncoder();
  }

  public void stopArmMotor() {
    armMotor.stopMotor();
  }

  public void stopRollerMotor() {
    rollerMotor.stopMotor();
  }

  public void setArmVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  public void setRollerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void setArmIdleMode(boolean isBrake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setRollerIdleMode(boolean isBrake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs (IntakeIOInputs inputs) {
    inputs.armVelocity = armEncoder.getVelocity();
    inputs.armAppliedVoltage = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
    inputs.armCurrentAmps = armMotor.getOutputCurrent();
    inputs.armTempCelcius = armMotor.getMotorTemperature();

    inputs.rollerVelocity = rollerEncoder.getVelocity();
    inputs.rollerAppliedVoltage = rollerMotor.getAppliedOutput() *armMotor.getBusVoltage();
    inputs.rollerCurrentAmps = rollerMotor.getOutputCurrent();
    inputs.rollerTempCelcius = rollerMotor.getMotorTemperature();
  }

}
