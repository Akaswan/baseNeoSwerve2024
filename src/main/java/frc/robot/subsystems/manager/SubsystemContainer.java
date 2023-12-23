// package frc.robot.subsystems.manager;

// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.manager.ServoMotorSubsystem.SubsystemType;

// public class SubsystemContainer {
//     private Arm m_arm;
//     private Wrist m_wrist;
//     private Elevator m_elevator;

//     public SubsystemContainer(Arm arm, Wrist wrist, Elevator elevator) {
//         m_arm = arm;
//         m_wrist = wrist;
//         m_elevator = elevator;
//     }

//     public Arm getArm() {
//         return m_arm;
//     }

//     public Wrist getWrist() {
//         return m_wrist;
//     }

//     public Elevator getElevator() {
//         return m_elevator;
//     }

//     public ServoMotorSubsystem getType(SubsystemType type) {
//         switch (type) {
//             case ARM:
//                 return m_arm;
//             case ELEVATOR:
//                 return m_elevator;
//             case WRIST:
//                 return m_wrist;
//             default:
//                 return null;
//         }
//     }

//     public ServoMotorSubsystem[] getArray() {
//         return new ServoMotorSubsystem[] {m_arm, m_wrist, m_elevator};
//     }
// }
