package org.firstinspires.ftc.teamcode.kowalski.statemachine;

public class State<T> {
    public StateMachine<T> stateMachine;
    public T runtime;

    public void onTransition(State<T> from) {
    }

    public void onTick(double timeSinceStart) {
    }
}
