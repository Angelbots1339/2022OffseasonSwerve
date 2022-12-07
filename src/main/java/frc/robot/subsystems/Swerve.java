package frc.robot.subsystems;
import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.SwerveModule;
import frc.robot.Constants.AngularDriveConstants;
import frc.lib.util.logging.SubsystemLogger;
import frc.lib.util.logging.SubsystemLogger.LogType;
import frc.robot.Constants;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SubsystemLogger logger;

    private PIDController angularDrivePID;
    private double lastDesiredDegrees;
    private double lastTime;
    

    public Swerve() {
        logger = new SubsystemLogger("Swerve", LoggingConstants.Swerve.logMotors, LoggingConstants.Swerve.logSensors, LoggingConstants.Swerve.logCalculatedValues);
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        logger.add("Yaw", () -> gyro.getYaw(), LogType.SENSOR);
        logger.add("Roll", () -> gyro.getRoll(), LogType.SENSOR);
        logger.add("Pitch", () -> gyro.getPitch(), LogType.SENSOR);
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        angularDrivePID =  new PIDController(AngularDriveConstants.turnToAngleP,
        AngularDriveConstants.turnToAngleI, AngularDriveConstants.turnToAngleD);

        angularDrivePID.enableContinuousInput(-180, 180);
        angularDrivePID.setTolerance(AngularDriveConstants.turnToAngleTolerance);
    }

    /**
     * 
     * @param translation Translation2d holding the desired velocities on each axis
     * @param rotation Desired rotational velocity
     * @param fieldRelative If true, the robot behaves as field relative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /**
     * 
     * 
     * @param translation Translation2d holding the desired velocities on each axis
     * @param desiredDegrees The desired orientation of the robot in degrees, whith 0 being straight forward
     * @param fieldRelative If true, the robot behaves as field relative
     * @param isOpenLoop
     */
    public void angularDrive(Translation2d translation, Rotation2d desiredDegrees, boolean fieldRelative, boolean isOpenLoop) {

        double deltaDegrees = (lastDesiredDegrees - desiredDegrees.getDegrees()) / (Timer.getFPGATimestamp() - lastTime);

        double rotation = MathUtil.clamp(
            angularDrivePID.calculate(-gyro.getYaw(), desiredDegrees.getDegrees()) + // PID
            ((deltaDegrees * AngularDriveConstants.turnToAngleKF) + (Math.signum(angularDrivePID.getPositionError()) * AngularDriveConstants.turnToAngleKS )), // Feedforward
             -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity); 
        
             drive(translation, rotation, fieldRelative, isOpenLoop);

        lastDesiredDegrees = desiredDegrees.getDegrees();
        lastTime = Timer.getFPGATimestamp();

    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Shuld be real angle " + mod.moduleNumber, mod.getCanCoder().getDegrees() - mod.angleOffset.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}