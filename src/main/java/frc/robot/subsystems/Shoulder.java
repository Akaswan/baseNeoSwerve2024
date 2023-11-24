// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.StateMachine.MechStateMachine;
import frc.robot.StateMachine.MechStates.MechState;
import frc.robot.StateMachine.MechStates.ShoulderState;
import frc.robot.StateMachine.StateMachineTypes;
import java.util.HashMap;
import java.util.Map;

public class Shoulder extends SubsystemBase implements MechStateMachine {
  /** Creates a new Arm. */
  private StateMachineInputs inputs = new StateMachineInputs();

  private Map<MechState, Runnable> stateMap;

  private CANSparkMax shoulderMotor;
  private ProfiledPIDController shoulderPID;
  private RelativeEncoder shoulderEncoder;

  private double simPosition;

  private double intendedPosition;

  Mechanism2d mech = new Mechanism2d(3, 3);
  MechanismRoot2d root = mech.getRoot("SuperStructure", 2.5, 0);
  MechanismRoot2d lowRoot = mech.getRoot("lowroot", 0, .3);
  MechanismRoot2d midRoot = mech.getRoot("midroot", 0, .85);
  MechanismRoot2d highRoot = mech.getRoot("highroot", 0, 1.4);
  MechanismLigament2d Low;
  MechanismLigament2d Mid;
  MechanismLigament2d High;
  MechanismLigament2d structure;
  MechanismLigament2d mechShoulder;

  public Shoulder(ShoulderState defaultState) {
    inputs.state = defaultState;
    inputs.previousState = inputs.state;

    shoulderMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shoulderMotor.setSmartCurrentLimit(60, 35);

    shoulderPID = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(50, 45));
    shoulderPID.setTolerance(.1);

    shoulderEncoder = shoulderMotor.getEncoder();
    shoulderEncoder.setPosition(0);

    intendedPosition = shoulderEncoder.getPosition();

    structure =
        root.append(new MechanismLigament2d("structure", 1, 90, 30, new Color8Bit(Color.kPurple)));
    mechShoulder = structure.append(new MechanismLigament2d("shoulder", 0, 0));
    Low =
        lowRoot.append(
            new MechanismLigament2d("Low", 1.8, 0, 150, new Color8Bit(Color.kBlueViolet)));
    Mid =
        midRoot.append(
            new MechanismLigament2d("Mid", 1.2, 0, 150, new Color8Bit(Color.kBlueViolet)));
    High =
        highRoot.append(
            new MechanismLigament2d("High", .6, 0, 150, new Color8Bit(Color.kBlueViolet)));

    SmartDashboard.putData("Mech2d", mech);

    initializeStateMap();
  }

  @Override
  public void initializeStateMap() {
    stateMap = new HashMap<>();

    stateMap.put(ShoulderState.IN, () -> intendedPosition = ShoulderState.IN.getPosition());
    stateMap.put(ShoulderState.LOW, () -> intendedPosition = ShoulderState.LOW.getPosition());
    stateMap.put(
        ShoulderState.MID_CONE, () -> intendedPosition = ShoulderState.MID_CONE.getPosition());
    stateMap.put(
        ShoulderState.MID_CUBE, () -> intendedPosition = ShoulderState.MID_CUBE.getPosition());
    stateMap.put(
        ShoulderState.HIGH_CONE, () -> intendedPosition = ShoulderState.HIGH_CONE.getPosition());
    stateMap.put(
        ShoulderState.HIGH_CUBE, () -> intendedPosition = ShoulderState.HIGH_CUBE.getPosition());
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
  public void setIntendedPosition(double intendedPosition) {
    this.intendedPosition = intendedPosition;
  }

  public void addIntendedPosition(double intendedPosition) {
    this.intendedPosition += intendedPosition;
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

  public double getSimPos() {
    return simPosition;
  }

  @Override
  public void periodic() {
    // shoulderMotor.set(shoulderPID.calculate(shoulderEncoder.getPosition(), intendedPosition));

    simPosition += shoulderPID.calculate(simPosition, intendedPosition);

    SmartDashboard.putNumber("Setpoint velocity", shoulderPID.getSetpoint().velocity);
    SmartDashboard.putNumber("Shoulder Intended Position", intendedPosition);
    SmartDashboard.putNumber("Shoulder Sim Position", simPosition);

    mechShoulder.setAngle(-((simPosition * 10) - 135));
    mechShoulder.setLength((RobotContainer.m_arm.getSimPos() / 5) + .25);
  }
}
