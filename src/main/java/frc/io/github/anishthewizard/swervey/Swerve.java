package frc.io.github.anishthewizard.swervey;

import frc.io.github.anishthewizard.control.PIDController;
import frc.io.github.anishthewizard.electronics.IMU.Gyro;
import frc.io.github.anishthewizard.electronics.IMU.Pigeon2IMU;
import frc.io.github.anishthewizard.electronics.motors.LazyMotorController;
import frc.io.github.anishthewizard.electronics.encoders.ThreadedEncoder;

import java.util.Arrays;

public class Swerve {
    private SwerveModule[] modules;

    private Pigeon2IMU gyro;

    private final double[] rotationAngles;
    private double maxModuleSpeed;

    private double[] speeds;
    private double[] thetas;

    private PIDController translationalPIDController;
    private PIDController rotationalPIDController;

    public Swerve(
            LazyMotorController<?>[] drives,
            LazyMotorController<?>[] steers,
            ThreadedEncoder<?>[] steercoders,
            Gyro<?> gyro,
            double[][] modulePositions,
            double[] translationalPIDGains,
            double[] rotationalPIDGains,
            double[] drivePIDFGains,
            double[] steerPIDGains,
            double MAX_MODULE_SPEED,
            int numberOfModules) {

        modules = new SwerveModule[numberOfModules];

        speeds = new double[numberOfModules];
        thetas = new double[numberOfModules];

        maxModuleSpeed = MAX_MODULE_SPEED;

        rotationAngles = new double[numberOfModules];

        translationalPIDController = new PIDController(translationalPIDGains);
        rotationalPIDController = new PIDController(rotationalPIDGains);

        for(int i = 0; i < numberOfModules; i++) {
            modules[i] = new SwerveModule(
                    drives[i],
                    steers[i],
                    steercoders[i],
                    modulePositions[i],
                    drivePIDFGains,
                    steerPIDGains);

            speeds[i] = 0;
            thetas[i] = 0;

            rotationAngles[i] = Math.atan2(modulePositions[i][1], modulePositions[i][0]) + (Math.PI/2);
        }
    }

    public void control(double x, double y, double rotate) {
        double chassisHeading = gyro.getYaw();


        for(int i = 0; i < modules.length; i++) {
            double[] rotationVector = new double[] {
                    rotate * Math.cos(rotationAngles[i] + chassisHeading),
                    rotate * Math.sin(rotationAngles[i] + chassisHeading)
            };

            double[] targetVector = vectorSum(rotationVector, new double[]{x, y});
            double speed = targetVector[0];
            double theta = targetVector[1];

            theta -= chassisHeading;

            if(!(x == 0 && y == 0 && rotate == 0))
                thetas[i] = theta;

            speeds[i] = speed;
        }

        speeds = normalize(speeds);

        for(int i = 0; i < modules.length; i++) {
            modules[i].set(speeds[i], thetas[i], chassisHeading);
        }

    }

    public void setSwerveState(double[] position, double linearVelocity, double angularVelocity, double heading) {
        double[] currentSwerveState = getSwerveState();

        double xLinearVelocity = linearVelocity * Math.cos(heading);
        double yLinearVelocity = linearVelocity * Math.sin(heading);

        double adjustedXLinearVel = xLinearVelocity + translationalPIDController.calculate(currentSwerveState[0], position[0]);
        double adjustedYLinearVel = yLinearVelocity + translationalPIDController.calculate(currentSwerveState[1], position[1]);

        double adjustedAngularVel = angularVelocity + rotationalPIDController.calculate(currentSwerveState[2], position[2]);

        control(adjustedXLinearVel, adjustedYLinearVel, adjustedAngularVel);
    }

    public double[] getSwerveState() {
        double x = 0, y = 0;
        double xVelocity = 0, yVelocity = 0;
        for(SwerveModule mod : modules) {
            double[] modPose = mod.getPose();
            double[] modVelocity = mod.getVelocityVector();

            x += modPose[0]/modules.length;
            y += modPose[1]/modules.length;

            xVelocity += modVelocity[0];
            yVelocity += modVelocity[1];
        }

        double linearVelocity = Math.hypot(xVelocity, yVelocity);
        double heading = Math.atan2(yVelocity, xVelocity);

        return new double[]{x, y, gyro.getYaw(), linearVelocity, heading};
    }

    public void reset() {
        for(SwerveModule mod : modules) {
            mod.reset();
        }
        gyro.reset();
    }

    public void setMaxModuleSpeed(double value) {
        maxModuleSpeed = value;
    }

    private double[] vectorSum(double[] a, double[] b) {
        double xSum = a[0] + b[0];
        double ySum = a[1] + b[1];
        return new double[]{Math.hypot(xSum, ySum), Math.atan2(ySum, xSum)};
    }

    private double[] normalize(double[] arr) {
        double inputtedMaxSpeed = Arrays.stream(arr).max().getAsDouble();
        if(inputtedMaxSpeed > maxModuleSpeed)
            for(int i = 0; i < arr.length; i++) {
                arr[i] /= inputtedMaxSpeed;
                arr[i] *= maxModuleSpeed;
            }
        return arr;
    }
}
