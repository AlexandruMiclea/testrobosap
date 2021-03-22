package org.firstinspires.ftc.teamcode.drive.subsystems;

public abstract class Subsystem {

    protected enum Mode {
        IDLE,
        BUSY
    }

    protected Mode mode;

    protected abstract void update();

    protected void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    protected boolean isBusy() { return mode != Subsystem.Mode.IDLE; }
}
