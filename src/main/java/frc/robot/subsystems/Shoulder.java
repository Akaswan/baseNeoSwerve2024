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
import frc.robot.StateMachine.MechStates.MechState;
import frc.robot.StateMachine.MechStates.ShoulderState;

public class Shoulder extends SubsystemBase implements MechStateMachine{
  /** Creates a new Arm. */
  private StateMachineInputs inputs = new StateMachineInputs();
  private Map<MechState, Runnable> stateMap;

  private CANSparkMax armMotor;
  private ProfiledPIDController armPID;
  private RelativeEncoder armEncoder;

  private double intendedPosition;

  public Shoulder(ShoulderState defaultState) {
    inputs.state = defaultState;
    inputs.previousState = inputs.state;

    armMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(60, 35);

    armPID = new ProfiledPIDController(.1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
    armPID.setTolerance(.1);

    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0);

    intendedPosition = armEncoder.getPosition();

    initializeStateMap();
  }

  @Override
  public void initializeStateMap() {
    stateMap = new HashMap<>();

    stateMap.put(ShoulderState.IN, () -> intendedPosition = 0);
    stateMap.put(ShoulderState.LOW, () -> intendedPosition = 10);
    stateMap.put(ShoulderState.MID_CONE, () -> intendedPosition = 20);
    stateMap.put(ShoulderState.MID_CUBE, () -> intendedPosition = 15);
    stateMap.put(ShoulderState.HIGH_CONE, () -> intendedPosition = 30);
    stateMap.put(ShoulderState.HIGH_CUBE, () -> intendedPosition = 25);
  }

  @Override
  public void handleStateAction() {
    stateMap.get(getState()).run();
  }

  @Override
  public void setState(MechState state) {
    inputs.previousState = inputs.state;
    inputs.state = state;

    // if (inputs.previousState != inputs.state) {
    //   handleStateAction();
    // }
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
    return Math.abs(getPosition() - intendedPosition) <= .1;
  }

  @Override
  public void periodic() {
    armMotor.set(armPID.calculate(armEncoder.getPosition(), intendedPosition));

    SmartDashboard.putNumber("Intended Position", intendedPosition);
  }
}
