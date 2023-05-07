package frc.io.github.anishthewizard.electronics.motors;

public interface LazyMotorController<T> {

    T getMotorController();

    void set(double speed);

    double getPosition();

    double getVelocity();
}
