package frc.io.github.anishthewizard.electronics.motors;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class LazyTalonFX implements LazyMotorController<TalonFX> {

    private TalonFX motor;
    private double lastSpeed = 0.;

    private double ticksPerMeter;

    public LazyTalonFX(TalonFX motor) {
        this.motor = motor;
    }

    public LazyTalonFX(int port) {
        this.motor = new TalonFX(port);
    }

    public void applyConfiguration(TalonFXConfiguration config) {
        this.motor.configAllSettings(config);
    }

    @Override
    public TalonFX getMotorController() {
        return motor;
    }

    @Override
    public void set(double speed) {
        if(speed != lastSpeed)
            motor.set(ControlMode.PercentOutput, speed);
        lastSpeed = speed;
    }

    @Override
    public double getPosition() {
        return motor.getSelectedSensorPosition()/ticksPerMeter;
    }

    @Override
    public double getVelocity() {
        return motor.getSelectedSensorVelocity()/ticksPerMeter;
    }
}
