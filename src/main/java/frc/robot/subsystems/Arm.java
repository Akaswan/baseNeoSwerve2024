// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine.ArmState;
import frc.robot.StateMachine.StateMachine;

public class Arm extends SubsystemBase implements StateMachine <ArmState>{
  /** Creates a new Arm. */
  private StateMachineInputs<ArmState> inputs = new StateMachineInputs<ArmState>();
  private Map<ArmState, Runnable> stateMap;

  private CANSparkMax armMotor;
  private ProfiledPIDController armPID;
  private RelativeEncoder armEncoder;

  private double intendedPosition;

  public Arm(ArmState defaultState) {
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

    stateMap.put(ArmState.IN, () -> intendedPosition = 0);
    stateMap.put(ArmState.LOW, () -> intendedPosition = 10);
    stateMap.put(ArmState.MEDIUM, () -> intendedPosition = 20);
    stateMap.put(ArmState.HIGH, () -> intendedPosition = 30);
  }

  @Override
  public void handleStateAction() {
    stateMap.get(getState()).run();
  }

  @Override
  public void setState(ArmState state) {
    inputs.previousState = inputs.state;
    inputs.state = state;

    if (inputs.previousState != inputs.state) {
      handleStateAction();
    }
  }

  @Override
  public ArmState getState() {
    return inputs.state;
  }

  @Override
  public void periodic() {
    armMotor.set(armPID.calculate(armEncoder.getPosition(), intendedPosition));

    SmartDashboard.putNumber("Intended Position", intendedPosition);
  }
}
