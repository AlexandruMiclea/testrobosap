package org.firstinspires.ftc.teamcode.drive;

public abstract class Subsystem {

    protected enum Mode {
        IDLE,
        BUSY,
    }

    protected Mode mode;

    protected abstract void update();

    protected void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() { return mode != Mode.IDLE; }
}
