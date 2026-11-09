package org.firstinspires.ftc.teamcode.kowalski.statemachine;

/** An extremely over-engineered state machine class
 * @param <T> The type of the "runtime" object to make available to the states.
 */
public class StateMachine<T> {
    private final T runtime;
    private State<T> state;
    private boolean stateJustChanged = false;
    private double transitionTime = 0.0;

    /** Initialise the state machine with a "runtime" object, which the states will
     * be able to use.
     */
    public StateMachine(T runtime) {
        this.runtime = runtime;
    }

    /** Transition to a new state. Please note that the State object must have a runtime parameter
     * of the same type as the used state machine.
     */
    public void transitionTo(State<T> to) {
        // Set some references for convenience
        to.stateMachine = this;
        to.runtime = this.runtime;
        // Run the hook
        to.onTransition(this.state);
        // Store the new state data
        this.state = to;
        this.stateJustChanged = true;
    }

    /** Run the tick hook of the current state. */
    public void tick(double time) {
        // Set the start time if the state just changed.
        // Doing it like this allows us to keep transitionTo() time-agnostic.
        if (this.stateJustChanged) {
            this.stateJustChanged = false;
            this.transitionTime = time;
        }
        // Run the tick hook
        this.state.onTick(time - this.transitionTime);
    }
}
