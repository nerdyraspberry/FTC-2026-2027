package org.firstinspires.ftc.teamcode.kowalski.autoshooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoShooter {
    private final DcMotor shooter;
    private final Servo hatch;

    private AutoShooterState state;

    private double activationTime = -1;

    public final double MOTOR_OFF = 0.0;
    public final double MOTOR_ON = -1.0;
    public final double SERVO_OFF = 0.0;
    public final double SERVO_ON = 90.0;

    public AutoShooter(DcMotor shooter, Servo hatch) {
        this.shooter = shooter;
        this.hatch = hatch;

        this.state = AutoShooterState.Idle;
    }

    public void start(double time) {
        this.activationTime = time;
    }

    public void abort() {
        this.setState(AutoShooterState.Idle);
    }

    public boolean isRunning() {
        return this.state != AutoShooterState.Idle;
    }

    public void setState(AutoShooterState state) {
        if (this.state == state)
            return;

        this.state = state;
        boolean motorOn = state == AutoShooterState.SpinningUp
                || state == AutoShooterState.Shooting;
        boolean servoOn = state == AutoShooterState.Shooting;
        this.shooter.setPower(motorOn ? MOTOR_ON : MOTOR_OFF);
        this.hatch.setPosition(servoOn ? SERVO_ON : SERVO_OFF);

        if (state == AutoShooterState.Idle)
            this.activationTime = -1;
    }

    public AutoShooterState getState() {
        return this.state;
    }

    public void tick(double time) {
        if (this.activationTime < 0)
            return;
        time -= this.activationTime;

        if (time == 0.0)
            this.setState(AutoShooterState.SpinningUp);
        else if (time >= 0.5 && time <= 0.6)
            this.setState(AutoShooterState.Shooting);
        else if (time >= 5+.0)
            this.setState(AutoShooterState.Idle);
    }
}
