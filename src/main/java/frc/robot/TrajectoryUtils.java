package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.driveSubsystem;

public class TrajectoryUtils {

    private driveSubsystem m_drive;
    private double maxSpeedMetersPerSecond = 0.5;
    private double maxAccelerationMetersPerSecondSquared = 0.25;
    private double maxVoltage = 6;
    private DifferentialDriveKinematics kDriveKinematics = null;
    private double kRamseteB;
    private double kRamseteZeta;
    private DifferentialDriveVoltageConstraint autoVoltageConstraint = null;
    private SmartDashboard smartDashboard = null;

    // TODO: comment
    public TrajectoryUtils(
            driveSubsystem drive, 
            DifferentialDriveKinematics ddk,
            double ms,
            double ma,
            double mv, 
            double rB,
            double rZeta,
            SmartDashboard sd) {

        smartDashboard = sd;
        m_drive = drive;
        kDriveKinematics = ddk;
        kRamseteB = rB;
        kRamseteZeta = rZeta;
        setMaxSpeedMetersPerSecond(ms);
        setMaxAccelerationMetersPerSecondSquared(ma);
        setMaxVoltage(mv);

        // Create a voltage constraint to ensure we don't accelerate too fast
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_drive.getFeedforward(), kDriveKinematics, maxVoltage);

    }

    public void setMaxSpeedMetersPerSecond(double ms) {
        maxSpeedMetersPerSecond = ms;
        if (smartDashboard != null) {
            SmartDashboard.putNumber("Auton Max Velocity (m/s)", maxSpeedMetersPerSecond);
        }
    }

    public void setMaxAccelerationMetersPerSecondSquared(double ma) {
        maxAccelerationMetersPerSecondSquared = ma;
        if (smartDashboard != null) {
            SmartDashboard.putNumber("Auton Max Acceleration (m/s^2)", maxAccelerationMetersPerSecondSquared);
        }
    }

    public void setMaxVoltage(double mv) {
        maxVoltage = mv;
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_drive.getFeedforward(), kDriveKinematics, maxVoltage);
        if (smartDashboard != null) {
            SmartDashboard.putNumber("Auton Max Voltage (V)", maxVoltage);
        }
    }

    /**
     * createTrajectoryCommand - given a start pose, some intermediate points, and a
     * finish pose, create a Ramsete Command to execute the path follow.
     * 
     * @param startPose
     * @param translationList
     * @param endPose
     * @param isReversed
     * @param maxSpeedMetersPerSecond
     * @param maxAccelerationMetersPerSecondSquared
     * @return Ramsete Path Follow Command, intake side of robot is isReversed =
     *         true and negative values
     */
    public RamseteCommand createTrajectoryCommand(Pose2d startPose, List<Translation2d> translationList,
            Pose2d endPose,
            boolean isReversed) {
        
        TrajectoryConfig config;

        // Create config for trajectory
        config = new TrajectoryConfig(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint).setReversed(isReversed);

        var initialTime = System.nanoTime();

        // trajectory to follow. All units in meters.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, translationList, endPose, config);

        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory, 
            m_drive::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            m_drive.getFeedforward(),
            kDriveKinematics,
            m_drive::getWheelSpeeds,
            m_drive.getLeftPidController(),
            m_drive.getRightPidController(),
            m_drive::tankDriveVolts,
            m_drive);

        var dt = (System.nanoTime() - initialTime) / 1E6;
        System.out.println("RamseteCommand generation time: " + dt + "ms");

        // Run path following command, then stop at the end.
        return ramseteCommand;
    }

}
