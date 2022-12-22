package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.wpilibj.I2C.Port;


public final class Constants {
    public static final double stickDeadband = 0.04;
    public static final double angularStickDeadband = 0.04;

    public static final class Swerve {
        public static final int PIGEON_ID = 16;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.75); 
        public static final double WHEEL_BASE = Units.inchesToMeters(19.75); 
        public static final double WHEEL_CIRCUMFRENCE = 0.1;

        /* Module Gear Ratios */
        /** SDS MK4i l1 - 8.14 : 1 */
        public static final double DRIVE_GEAR_RATIO =  (8.14 / 1.0);
        /** SDS MK4i l1 - (150 / 7) : 1 */
        public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = true;
        public static final boolean DRIVE_MOTOR_INVERT = true;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERT = false;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Swerve Current Limiting */
        public static final int ANGLE_COUNTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAL_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 0.3;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.05; //TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double DRIVE_KS = (0.65713 / 12); 
        public static final double DRIVE_KV = (2.8621 / 12);
        public static final double DRIVE_KA = (0.20391 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 1.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 5.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Module Specific Constants */
        /** Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 5;
            public static final int CANCODER_ID = 9;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(353.84765625);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 10;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(181.0546875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /** Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 7;
            public static final int CANCODER_ID = 11;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(65.21484375);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 12;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(187.822265625);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        public static final class AngularDriveConstants {

            public static final double ANGLE_KP = 0.1; //radians per sec per degrees
            public static final double ANGLE_KI = 0;
            public static final double ANGLE_KD = 0.01;
    
            //Feedfowards
            public static final double ANGLE_KV = 0.0;
            public static final double ANGLE_KS = 0.7; //radians per sec
            
            public static final double TURN_TO_ANGLE_TOLERANCE = 2; // Degrees
        }

    }
    public static final class AutoConstants {
      
        public static final double X_KP = 1;
        public static final double Y_KP = 1;
        public static final double THETA_KP = 3.2;
    
    }
    public static final class Elevator {
        public static final int ELEVATOR_MOTOR_ID = -1;//TODO
    }

    public static final class VisonConstants {
        public static final String CAMERA_NAME = "";
        public static final PhotonCamera CAMERA = new PhotonCamera(CAMERA_NAME);
    }
    public final static class MultiplexerConstants {
        public static final byte DEFAULT_ADDRESS = 0x70;
        public static final Port DEFAULT_PORT = Port.kOnboard;
    }
}
