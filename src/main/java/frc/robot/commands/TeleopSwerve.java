package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSupX;
    private DoubleSupplier rotationSupY;
    private BooleanSupplier isFieldRelative;
    private BooleanSupplier turnModeButtonPressed;

    private boolean isTurnModePIDToAngle = false;

    // private SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(TeleopSwerveConstants.turnToAngleKS,
    //         TeleopSwerveConstants.turnToAngleKV, TeleopSwerveConstants.turnToAngleKA);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSupX, DoubleSupplier rotationSupY, BooleanSupplier isFieldRelative,
            BooleanSupplier turnModeButtonPressed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSupX = rotationSupX;
        this.rotationSupY = rotationSupY;
        this.isFieldRelative = isFieldRelative;
        this.turnModeButtonPressed = turnModeButtonPressed;
    }

    private double desiredRadians;
    private double lastRadians;

    @Override
    public void execute() {

        if (turnModeButtonPressed.getAsBoolean())
            isTurnModePIDToAngle = turnModeButtonPressed.getAsBoolean() ? false : true;

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        if(Math.abs(rotationSupX.getAsDouble()) > 0.4 || Math.abs(rotationSupY.getAsDouble()) > 0.4){
            desiredRadians = Math.atan2(rotationSupY.getAsDouble(), rotationSupX.getAsDouble());
            lastRadians = desiredRadians;
        } else {
            desiredRadians = lastRadians;
        }

        double rotationVal = MathUtil.applyDeadband(rotationSupX.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        if(isTurnModePIDToAngle) {
            s_Swerve.angularDrive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                new Rotation2d(desiredRadians),
                isFieldRelative.getAsBoolean(), // Field relative
                true);
        } else{
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                (rotationVal * Constants.Swerve.maxAngularVelocity),
                isFieldRelative.getAsBoolean(), // Field relative
                true);
        }
    }
}
