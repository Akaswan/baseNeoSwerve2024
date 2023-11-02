package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;

public class SwerveModuleConstants {    
  
    public final int driveMotorChannel;
    public final int turningMotorChannel;
    public final int cancoderID;
    public final double angleOffset;
    public final PIDController dummyDriveController;
    public final PIDController dummyTurnController;
 

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID,
        int turningMotorID,
        int cancoderID,
        double angleOffset) {

        this.driveMotorChannel = driveMotorID;
        this.turningMotorChannel = turningMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
        dummyDriveController = new PIDController(0, 0, 0);
        dummyTurnController = new PIDController(0, 0, 0);
    }

    public SwerveModuleConstants(SwerveModuleConstants constants, PIDController dummyDriveController, PIDController dummyTurnController) {
        driveMotorChannel = constants.driveMotorChannel;
        turningMotorChannel = constants.turningMotorChannel;
        cancoderID = constants.cancoderID;
        angleOffset = constants.angleOffset;

        this.dummyDriveController = dummyDriveController;
        this.dummyTurnController = dummyTurnController;
    }
    

}
