package frc.io.github.anishthewizard.electronics.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class LazySparkMax implements LazyMotorController<CANSparkMax> {

    private CANSparkMax motor;

    private double lastSpeed = 0.;

    private double ticksPerMeter;

    public LazySparkMax(CANSparkMax motor, double ticksPerMeter) {
        this.motor = motor;
        this.ticksPerMeter = ticksPerMeter;
    }

    public LazySparkMax(int port, MotorType type, double ticksPerMeter) {
        this.motor = new CANSparkMax(port, type);
        this.ticksPerMeter = ticksPerMeter;
    }

    @Override
    public CANSparkMax getMotorController() {
        return motor;
    }

    @Override
    public void set(double speed) {
        if(speed != lastSpeed)
            motor.set(speed);
        lastSpeed = speed;
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition()/ticksPerMeter;
    }

    @Override
    public double getVelocity() {
        return motor.getEncoder().getVelocity()/ticksPerMeter;
    }
}
