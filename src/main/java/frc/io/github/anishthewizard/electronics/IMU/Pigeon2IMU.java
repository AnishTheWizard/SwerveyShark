package frc.io.github.anishthewizard.electronics.IMU;

import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon2IMU implements Gyro<Pigeon2> {

    private Pigeon2 gyro;

    public Pigeon2IMU(Pigeon2 gyro) {
        this.gyro = gyro;
    }

    public Pigeon2IMU(int port) {
        this.gyro = new Pigeon2(port);
    }

    @Override
    public Pigeon2 getGyro() {
        return gyro;
    }

    @Override
    public double getPitch() {
        return gyro.getPitch();
    }

    @Override
    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public double getYaw() {
        return gyro.getYaw();
    }

    @Override
    public void setYaw(double value) {
        gyro.setYaw(value);
    }

    @Override
    public void reset() {
        gyro.setYaw(0);
    }
}
