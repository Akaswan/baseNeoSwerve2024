// package frc.robot.commands.SwerveDrivebase;

// import frc.robot.subsystems.SwerveDrive;
// import frc.robot.utilities.CreateEventMap;

// import java.util.HashMap;
// import java.util.List;


// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// /*
//  * <h3>CreatePath<h3>
//  * 
//  */
// public class CreateAuto extends SequentialCommandGroup {

//     public static HashMap<String, Command> eventMap = new HashMap<>();

//     private Command pathCommand;
    
//     /**
//      * <h3>CreatePath</h3>
//      * 
//      * adding path constraints and builds auto command
//      * 
//      * @param preCommand a command that runs before the path, use command groups to run multiple commands
//      * @param m_drivebase the required subsystem
//      * @param pathName name of path (pathPlanner's path)
//      * @param maxVelocity max velocity of path
//      * @param maxAcceleration max acceleration of path
//      * @param postCommand a command that is run after the path completes, use command groups to run multiple commands
//      */
//     public CreateAuto(String pathName, Command preCommand, Command postCommand) {

//         // creates a command based on the path with post and pre commands added
//         pathCommand = AutoBuilder.buildAuto(pathName);
//         if (preCommand != null) {
//             addCommands(preCommand);
//         }
//         addCommands(
//             pathCommand
//         );
//         if (postCommand != null) {
//             addCommands(postCommand);
//         }
//     }

//     public CreateAuto(String pathName) {

//         pathCommand = AutoBuilder.buildAuto(pathName);
//         addCommands(
//             pathCommand
//         );
//     }
// }