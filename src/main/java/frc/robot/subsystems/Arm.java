// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine.MechStateMachine;
import frc.robot.StateMachine.StateMachineTypes;
import frc.robot.StateMachine.MechStates.ArmState;
import frc.robot.StateMachine.MechStates.MechState;

public class Arm extends SubsystemBase implements MechStateMachine{
  /** Creates a new Arm. */
  private StateMachineInputs inputs = new StateMachineInputs();
  private Map<MechState, Runnable> stateMap;

  private CANSparkMax armMotor;
  private ProfiledPIDController armPID;
  private RelativeEncoder armEncoder;

  private double simPosition;

  

  private double intendedPosition;

  public Arm(ArmState defaultState) {
    inputs.state = defaultState;
    inputs.previousState = inputs.state;

    armMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(60, 35);

    armPID = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(50, 45));
    armPID.setTolerance(.1);

    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0);

    intendedPosition = armEncoder.getPosition();

    initializeStateMap();
  }

  public double getSimPos() {
    return simPosition;
  }

  

  @Override
  public void initializeStateMap() {
    stateMap = new HashMap<>();

    stateMap.put(ArmState.IN, () -> intendedPosition = ArmState.IN.getPosition());
    stateMap.put(ArmState.LOW, () -> intendedPosition = ArmState.LOW.getPosition());
    stateMap.put(ArmState.MID, () -> intendedPosition = ArmState.MID.getPosition());
    stateMap.put(ArmState.HIGH, () -> intendedPosition = ArmState.HIGH.getPosition());
  }

  @Override
  public void handleStateAction() {
    stateMap.get(getState()).run();
  }

  @Override
  public void setState(MechState state, boolean action) {
    inputs.previousState = inputs.state;
    inputs.state = state;

    if (inputs.previousState != inputs.state && action) {
      handleStateAction();
    }
  }

  @Override
  public MechState getState() {
    return inputs.state;
  }

  @Override
  public Map<MechState, Runnable> getStateMap() {
    return stateMap;
  }

  @Override
  public double getPosition() {
    return armEncoder.getPosition();
  }

  @Override
  public boolean atSetPoint() {
    // return Math.abs(getPosition() - intendedPosition) <= .1;
    return Math.abs(simPosition - intendedPosition) <= .1;
  }

  @Override
  public StateMachineTypes getType() {
    return StateMachineTypes.ARM;
  }

  @Override
  public void periodic() {
    // armMotor.set(armPID.calculate(armEncoder.getPosition(), intendedPosition));

    simPosition += armPID.calculate(simPosition, intendedPosition);

    SmartDashboard.putNumber("Arm Intended Position", intendedPosition);
    SmartDashboard.putNumber("Arm Sim Position", simPosition);
  }

  @Override
  public void setIntendedPosition(double intendedPosition) {
    this.intendedPosition = intendedPosition;
  }
}
