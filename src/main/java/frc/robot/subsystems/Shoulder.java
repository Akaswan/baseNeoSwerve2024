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
import frc.robot.StateMachine.MechStates.MechState;
import frc.robot.StateMachine.MechStates.ShoulderState;

public class Shoulder extends SubsystemBase implements MechStateMachine{
  /** Creates a new Arm. */
  private StateMachineInputs inputs = new StateMachineInputs();
  private Map<MechState, Runnable> stateMap;

  private CANSparkMax shoulderMotor;
  private ProfiledPIDController shoulderPID;
  private RelativeEncoder shoulderEncoder;

  private double simPosition;

  private double intendedPosition;

  public Shoulder(ShoulderState defaultState) {
    inputs.state = defaultState;
    inputs.previousState = inputs.state;

    shoulderMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shoulderMotor.setSmartCurrentLimit(60, 35);

    shoulderPID = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
    shoulderPID.setTolerance(.1);

    shoulderEncoder = shoulderMotor.getEncoder();
    shoulderEncoder.setPosition(0);

    intendedPosition = shoulderEncoder.getPosition();

    initializeStateMap();
  }

  @Override
  public void initializeStateMap() {
    stateMap = new HashMap<>();

    stateMap.put(ShoulderState.IN, () -> intendedPosition = ShoulderState.IN.getPosition());
    stateMap.put(ShoulderState.LOW, () -> intendedPosition = ShoulderState.LOW.getPosition());
    stateMap.put(ShoulderState.MID_CONE, () -> intendedPosition = ShoulderState.MID_CONE.getPosition());
    stateMap.put(ShoulderState.MID_CUBE, () -> intendedPosition = ShoulderState.MID_CUBE.getPosition());
    stateMap.put(ShoulderState.HIGH_CONE, () -> intendedPosition = ShoulderState.HIGH_CONE.getPosition());
    stateMap.put(ShoulderState.HIGH_CUBE, () -> intendedPosition = ShoulderState.HIGH_CUBE.getPosition());
  }

  @Override
  public void handleStateAction() {
    stateMap.get(getState()).run();
  }

  @Override
  public void setState(MechState state) {
    inputs.previousState = inputs.state;
    inputs.state = state;

    if (inputs.previousState != inputs.state) {
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
    return shoulderEncoder.getPosition();
  }

  @Override
  public boolean atSetPoint() {
    // return Math.abs(getPosition() - intendedPosition) <= .1;
    return Math.abs(simPosition - intendedPosition) <= .1;
  }

  @Override
  public StateMachineTypes getType() {
    return StateMachineTypes.SHOULDER;
  }

  @Override
  public void periodic() {
    // shoulderMotor.set(shoulderPID.calculate(shoulderEncoder.getPosition(), intendedPosition));

    simPosition += shoulderPID.calculate(simPosition, intendedPosition);

    SmartDashboard.putNumber("Shoulder Intended Position", intendedPosition);
    SmartDashboard.putNumber("Shoulder Sim Position", simPosition);
  }
}
