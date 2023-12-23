// package frc.robot.subsystems.manager;

// import frc.robot.subsystems.manager.ServoMotorSubsystem.SubsystemState;

// public class SubsystemStateContainer {
//         private SubsystemState m_armState;
//         private SubsystemState m_wristState;
//         private SubsystemState m_elevatorState;

//         public SubsystemStateContainer(SubsystemState armState, SubsystemState wristState,
// SubsystemState elevatorState) {
//             m_armState = armState;
//             m_wristState = wristState;
//             m_elevatorState = elevatorState;
//         }

//         public SubsystemState getArmState() {
//             return m_armState;
//         }

//         public SubsystemState getWristState() {
//             return m_wristState;
//         }

//         public SubsystemState getElevatorState() {
//             return m_elevatorState;
//         }

//         public boolean equalTo(SubsystemStateContainer container) {
//             if (container.getArmState() == m_armState && container.getWristState() ==
// m_wristState && container.getElevatorState() == m_elevatorState) {
//                 return true;
//             } else {
//                 return false;
//             }
//         }
//     }
