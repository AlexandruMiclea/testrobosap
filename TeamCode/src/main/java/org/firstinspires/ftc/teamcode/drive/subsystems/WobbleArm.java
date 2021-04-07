package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Subsystem;

//TODO: to the thread - (clasa Subsystem care e thread)
public class WobbleArm extends Subsystem {

    private int LOW_CONSTRAINT = -850;
    private int HIGH_CONSTRAINT = 450;
    private double MAX_LIFT_SPEED = 0.5, MAX_LOWER_SPEED = 0.3;
    private double CLAMPED_POS = 0.8, UNCLAMPED_POS = 0;

    private DcMotorEx armMotor;
    private Servo clamServo;
    private boolean isConstraints;

    public WobbleArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "motorBrat");
        clamServo = hardwareMap.servo.get("servoBrat");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPositionTolerance(100);

        clamServo.setPosition(CLAMPED_POS);

        subMode = SubMode.SUB_IDLE;
    }

    public void setConstraints(boolean constraints){
        this.isConstraints = constraints;
    }

    public void setMotorMode(DcMotor.RunMode mode){ armMotor.setMode(mode); }

    public DcMotor.RunMode getMotorMode(){ return armMotor.getMode(); }

    public boolean getMotorIsBusy(){ return armMotor.isBusy(); }

    public SubMode getMode(){ return subMode; }

    public int getPosition() { return armMotor.getCurrentPosition(); }

    public int getTargetPosition() { return armMotor.getTargetPosition(); }

    public boolean getConstraints(){
        return isConstraints;
    }

    public void stop() {
        armMotor.setPower(0);
    }

    public void moveArm(double speed) {
        if (isConstraints){
            if(speed > 0 && armMotor.getCurrentPosition() > HIGH_CONSTRAINT){
                stop();
            }
            else if(speed < 0 && armMotor.getCurrentPosition() < LOW_CONSTRAINT){
                stop();
            } else {
                armMotor.setPower(speed * (speed > 0? MAX_LIFT_SPEED : MAX_LOWER_SPEED));
            }

        } else {
            armMotor.setPower(speed * (speed > 0? MAX_LIFT_SPEED : MAX_LOWER_SPEED));
        }
    }

    public void clawToggle() {
        if (clamServo.getPosition() == CLAMPED_POS) {
            clamServo.setPosition(UNCLAMPED_POS);
        } else if (clamServo.getPosition() == UNCLAMPED_POS) {
            clamServo.setPosition(CLAMPED_POS);
        }
    }

    public void clawToggle(boolean clamped) {
        if (!clamped) {
            clamServo.setPosition(UNCLAMPED_POS);
        } else if (clamped) {
            clamServo.setPosition(CLAMPED_POS);
        }
    }

    public void armPositionToggleAsync(boolean up){
        armMotor.setTargetPosition(up ? HIGH_CONSTRAINT : LOW_CONSTRAINT);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.2);
        subMode = SubMode.SUB_BUSY;
    }

    public void armPositionToggleAsync(boolean up, double customSpeed){
        armMotor.setTargetPosition(up ? HIGH_CONSTRAINT : LOW_CONSTRAINT);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(customSpeed);
        subMode = SubMode.SUB_BUSY;
    }

    public void armPositionToggle(boolean up){
        armPositionToggleAsync(up);
        waitForSubIdle();
    }

    public void armPositionToggle(boolean up, double customSpeed){
        armPositionToggleAsync(up, customSpeed);
        waitForSubIdle();
    }

    public void updateSub(){
        switch (subMode){
            case SUB_IDLE:
                //do nothing
                break;
            case SUB_BUSY:
                if (!armMotor.isBusy()) {
                    subMode = SubMode.SUB_IDLE;
                }
                break;
        }
    }
}
