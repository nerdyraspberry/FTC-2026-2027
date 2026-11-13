package org.firstinspires.ftc.teamcode.kowalski.autoshooter;

public enum AutoShooterState {
    Idle,
    SpinningUp,
    Shooting;

    public AutoShooterState back() {
        switch (this) {
            case Idle: case SpinningUp:
                return Idle;
            case Shooting:
                return SpinningUp;
            default:
                throw new IllegalArgumentException("This cannot occur!");
        }
    }

    public AutoShooterState next() {
        switch (this) {
            case Idle:
                return SpinningUp;
            case SpinningUp: case Shooting:
                return Shooting;
            default:
                throw new IllegalArgumentException("This cannot occur!");
        }
    }
}
