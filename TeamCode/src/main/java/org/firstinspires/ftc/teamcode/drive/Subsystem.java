package org.firstinspires.ftc.teamcode.drive;

public abstract class Subsystem {

    protected enum SubMode {
        SUB_IDLE,
        SUB_BUSY
    }

    protected SubMode subMode;

    protected abstract void updateSub();

    protected void waitForSubIdle() {
        while (!Thread.currentThread().isInterrupted() && isSubBusy()) {
            updateSub();
        }
    }

    protected boolean isSubBusy() { return subMode != SubMode.SUB_IDLE; }
}
