package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.SwerveModule;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.LoggingConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.Swerve.*;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public LoggedSubsystem logger;
    private PIDController angularDrivePID;

    public Swerve() {

        logger = new LoggedSubsystem("Swerve", LoggingConstants.SWERVE);

        gyro = new Pigeon2(PIGEON_ID);
        gyro.configFactoryDefault();
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getYaw());

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Mod0.constants),
                new SwerveModule(1, Mod1.constants),
                new SwerveModule(2, Mod2.constants),
                new SwerveModule(3, Mod3.constants)
        };

        logger.addDouble("Yaw", () -> gyro.getYaw(), "Gyro");
        logger.addDouble("Roll", () -> gyro.getRoll(), "Gyro");
        logger.addDouble("Pitch", () -> gyro.getPitch(), "Gyro");
        

        for (int i = 0; i < mSwerveMods.length; i++) {
            logger.add(new LoggedFalcon("Angle Motor: " + i, logger, mSwerveMods[i].getAngleMotor(), "Motor", true));
            logger.add(new LoggedFalcon("Drive Motor: " + i, logger, mSwerveMods[i].getDriveMotor(), "Motor", true));
        }

    

        angularDrivePID = new PIDController(AngularDriveConstants.ANGLE_KP,
                AngularDriveConstants.ANGLE_KI, AngularDriveConstants.ANGLE_KD);

        angularDrivePID.enableContinuousInput(0, 360);
        angularDrivePID.setTolerance(AngularDriveConstants.TURN_TO_ANGLE_TOLERANCE);
    }

    /**
     * 
     * @param translation   Translation2d holding the desired velocities on each
     *                      axis
     * @param rotation      Desired rotational velocity
     * @param fieldRelative If true, the robot behaves as field relative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    private double lastDesiredDegrees;
    private double lastTime;

    /**
     * @param translation    Translation2d holding the desired velocities on each
     *                       axis
     * @param desiredDegrees The desired orientation of the robot, whith 0 being
     *                       straight forward
     * @param fieldRelative  If true, the robot behaves as field relative
     * @param isOpenLoop
     */
    public void angularDrive(Translation2d translation, Rotation2d desiredDegrees, boolean fieldRelative,
            boolean isOpenLoop) {

        Rotation2d desiredAngularVelocity = Rotation2d.fromDegrees(
                (lastDesiredDegrees - desiredDegrees.getDegrees()) / (Timer.getFPGATimestamp() - lastTime));
        lastDesiredDegrees = desiredDegrees.getDegrees();
        lastTime = Timer.getFPGATimestamp();

        double rotation = 0;
        // convert yaw into -180 -> 180 and abslute
        double yaw = (0 > getYaw().getDegrees() ? getYaw().getDegrees() % 360 + 360 : getYaw().getDegrees() % 360);
        rotation = MathUtil.clamp(
                angularDrivePID.calculate(yaw, desiredDegrees.getDegrees() + // PID
                        (!angularDrivePID.atSetpoint() ? // feed foward
                                (-desiredAngularVelocity.getRadians() * AngularDriveConstants.ANGLE_KV) + // Kv Velocity
                                                                                                          // Feedforward
                                        ((Math.signum(angularDrivePID.getPositionError())
                                                * AngularDriveConstants.ANGLE_KS))
                                : 0)), // Ks Static Friction Feedforward
                -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void resetToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {

        for (SwerveModule mod : mSwerveMods) {

        }
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        swerveKinematics, // SwerveDriveKinematics
                        new PIDController(3.2023, 0, 0), // X controller. Tune these values for your robot. Leaving them
                                                         // 0 will only use feedforwards.
                        new PIDController(3.2023, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(THETA_KP, 0, 0), // Rotation controller. Tune these values for your robot.
                                                           // Leaving them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer // This argument is optional if you don't use
                                               // event markers
                        this // Requires this drive subsystem
                ));
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
        return followTrajectoryCommand(traj, false);
    }

}