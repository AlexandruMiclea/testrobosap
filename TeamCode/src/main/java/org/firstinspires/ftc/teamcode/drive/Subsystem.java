package org.firstinspires.ftc.teamcode.drive;

public abstract class Subsystem {

    protected enum SubMode {
        IDLE,
        BUSY
    }

    protected SubMode mode;

    protected abstract void update();

    protected void waitForSubIdle() {
        while (!Thread.currentThread().isInterrupted() && isSubBusy()) {
            update();
        }
    }

    protected boolean isSubBusy() { return mode != Subsystem.SubMode.IDLE; }
}
