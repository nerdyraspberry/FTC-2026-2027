package org.firstinspires.ftc.teamcode.kowalski;

/**
 * A helper class to (rising edge) debounce gamepad inputs.
 */
public class Debouncer {
    private boolean pressedAlready = false;

    /** Returns true if the given boolean was just pressed.
     * This is a rising-edge detector.
     */
    public boolean wasJustPressed(boolean currentValue) {
        if (currentValue) {
            if (!this.pressedAlready) {
                this.pressedAlready = true;
                return true;
            }
        } else {
            this.pressedAlready = false;
        }
        return false;
    }
}
