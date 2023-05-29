package frc.robot;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.io.github.anishthewizard.electronics.IMU.Pigeon2IMU;
import frc.io.github.anishthewizard.electronics.encoders.ThreadedCANcoder;
import frc.io.github.anishthewizard.electronics.motors.LazyTalonFX;
import frc.io.github.anishthewizard.swervey.Swerve;
import frc.io.github.anishthewizard.swervey.SwerveConfiguration;

public class Drivetrain extends SubsystemBase {

    private final static Drivetrain INSTANCE = new Drivetrain();

    @SuppressWarnings("WeakerAccess")
    public static Drivetrain getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Drivetrain. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */

    Swerve swerve;
    private Drivetrain() {

        LazyTalonFX[] drives = new LazyTalonFX[4];
        LazyTalonFX[] steers = new LazyTalonFX[4];

        ThreadedCANcoder[] encoders = new ThreadedCANcoder[4];

        double[][] modulePositions = new double[][]{
                new double[]{0.5794/2, 0.5794/2},
                new double[]{-0.5794/2, 0.5794/2},
                new double[]{-0.5794/2, -0.5794/2},
                new double[]{0.5794/2, -0.5794/2}
        };

        double ticksPerMeter = 0;

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        for(int i = 0; i < 4; i++) {
            TalonFX drive = new TalonFX(i);
            drive.config_kF(0, 1/(4.94*ticksPerMeter));
            drive.config_kP(0, 0.05);

            drives[i] = new LazyTalonFX(drive, ticksPerMeter);
            steers[i] = new LazyTalonFX(new TalonFX(i+4), ticksPerMeter);

            encoders[i] = new ThreadedCANcoder(i, Math.PI, 0, 20);
        }

        Pigeon2IMU gyro = new Pigeon2IMU(0);

        SwerveConfiguration config = new SwerveConfiguration();

        config.drives = drives;
        config.steers = steers;
        config.encoders = encoders;
        config.gyro = gyro;
        config.modulePositions = modulePositions;
        config.translationalPIDGains = new double[]{0.03, 0.0, 0.0};
        config.rotationalPIDGains = new double[]{0.65, 0.0, 0.0};
        config.drivePIDFGains = new double[]{0.01, 0.0, 0.0, 1.0/4.96824};
        config.steerPIDGains = new double[]{0.62, 0.0, 0.0};
        config.MAX_MODULE_SPEED = 4.96824;
        config.radius = Math.hypot(0.5794/2, 0.5794/2);
        config.numberOfModules = 4;

        swerve = Swerve.fromConfiguration(config);
    }

    public void control(double x, double y, double rotate) {
        swerve.controlWithPercent(x, y, rotate);
    }
}

