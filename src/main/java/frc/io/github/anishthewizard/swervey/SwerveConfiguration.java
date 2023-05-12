package frc.io.github.anishthewizard.swervey;

import frc.io.github.anishthewizard.electronics.IMU.Gyro;
import frc.io.github.anishthewizard.electronics.encoders.ThreadedEncoder;
import frc.io.github.anishthewizard.electronics.motors.LazyMotorController;

public class SwerveConfiguration {
    public LazyMotorController<?>[] drives = null;
    public LazyMotorController<?>[] steers = null;

    public ThreadedEncoder<?>[] encoders = null;

    public Gyro<?> gyro = null;

    public double[][] modulePositions = null;

    public double[] translationalPIDGains = null;

    public double[] rotationalPIDGains = null;

    public double[] drivePIDFGains = null;

    public double[] steerPIDGains = null;

    public double MAX_MODULE_SPEED = Double.parseDouble(null);

    public int numberOfModules = Integer.parseInt(null);

    public boolean isConfigReady() {
        boolean condition = drives == null ||
                steers == null ||
                encoders == null ||
                gyro == null ||
                modulePositions == null ||
                translationalPIDGains == null ||
                rotationalPIDGains == null ||
                drivePIDFGains == null ||
                steerPIDGains == null ||
                MAX_MODULE_SPEED == Double.parseDouble(null) ||
                numberOfModules == Integer.parseInt(null);

        return !condition;
    }
}
