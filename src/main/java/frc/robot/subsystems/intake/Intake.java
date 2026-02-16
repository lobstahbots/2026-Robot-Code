// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  private boolean isDeployed;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
    this.isDeployed = false;
  }

  public void stopArmMotor() {
    io.stopArmMotor();
  }

  public void stopRollerMotor() {
    io.stopRollerMotor();
  }

  public void setArmVoltage(double volts) {
    io.setArmVoltage(volts);
  }

  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  public void setArmSpeed(double speed) {
    io.setArmSpeed(speed);
  }

  public void setRollerSpeed(double speed) {
    io.setRollerSpeed(speed);
  }

  public void setArmIdleMode(boolean isBrake) {
    io.setArmIdleMode(isBrake);
  }

  public void setRollerIdleMode(boolean isBrake) {
    io.setRollerIdleMode(isBrake);
  }

  public double getArmVelocity() {
    return inputs.ArmVelocity;
  }

  public double getRollerVelocity() {
    return inputs.rollerVelocity;
  }

  public double getArmCurrent() {
    return inputs.armCurrentAmps;
  }

  public double getRollerCurrent() {
    return inputs.rollerCurrentAmps;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command toggleIntakeDeployCommand () {
    // return Commands.run(
    //     () -> this.isDeployed ? this.setArmSpeed(IntakeConstants.ARM_DEPLOY_SPEED) : this.setArmSpeed(-IntakeConstants.ARM_DEPLOY_SPEED)
    //     , this)
    //     .until(() -> this.getArmCurrent() > IntakeConstants.ARM_DEPLOY_CURRENT_THRESHHOLD);

    return Commands.run(() -> {
        if(isDeployed) {
            this.setArmSpeed(IntakeConstants.ARM_DEPLOY_SPEED);
        } else {
            this.setArmSpeed(-IntakeConstants.ARM_DEPLOY_SPEED);
        }
        isDeployed = !isDeployed;
    }, this)
        .until(() -> this.getArmCurrent() > IntakeConstants.ARM_DEPLOY_CURRENT_THRESHOLD);
  }

  public Command spinIntakeCommand (double speed) {
    return Commands.run(() -> this.setRollerSpeed(speed), this);
  }

  public Command stopIntakeRollerCommand () {
    return Commands.run(() -> this.stopRollerMotor(), this);
  }

  public Command stopIntakeArmCommand () {
    return Commands.run(() -> this.stopRollerMotor(), this);
  }
}
