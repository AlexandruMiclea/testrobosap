package org.firstinspires.ftc.teamcode.drive.subsystems;

import android.content.IntentFilter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Subsystem;

//TODO: to the thread - (clasa Subsystem care e thread)
public class WobbleArm extends Subsystem {

    private int LOW_CONSTRAINT = -1100;
    private int HIGH_CONSTRAINT = -100;
    private double MAX_LIFT_SPEED = 0.5, MAX_LOWER_SPEED = 0.3;
    private double CLAMPED_POS = 0.8, UNCLAMPED_POS = 0;

    private DcMotor motorBrat;
    private Servo servoBrat;
    private boolean isConstraints;

    public WobbleArm(HardwareMap hardwareMap) {
        motorBrat = hardwareMap.dcMotor.get("motorBrat");
        servoBrat = hardwareMap.servo.get("servoBrat");

        motorBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBrat.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoBrat.setPosition(CLAMPED_POS);

        subMode = SubMode.SUB_IDLE;
    }

    public void setConstraints(boolean constraints){
        this.isConstraints = constraints;
    }

    public void setMotorMode(DcMotor.RunMode mode){ motorBrat.setMode(mode); }

    public DcMotor.RunMode getMotorMode(){ return motorBrat.getMode(); }

    public Subsystem.SubMode getMode(){ return subMode; }

    public int getPosition() { return motorBrat.getCurrentPosition(); }

    public int getTargetPosition() { return motorBrat.getTargetPosition(); }

    public boolean getConstraints(){
        return isConstraints;
    }

    public void stop() {
        motorBrat.setPower(0);
    }

    public void moveArm(double speed) {
        if (isConstraints){
            if(speed > 0 && motorBrat.getCurrentPosition() > HIGH_CONSTRAINT){
                stop();
            }
            else if(speed < 0 && motorBrat.getCurrentPosition() < LOW_CONSTRAINT){
                stop();
            } else {
                motorBrat.setPower(speed * (speed > 0? MAX_LIFT_SPEED : MAX_LOWER_SPEED));
            }

        } else {
            motorBrat.setPower(speed * (speed > 0? MAX_LIFT_SPEED : MAX_LOWER_SPEED));
        }
    }

    public void clawToggle() {
        if (servoBrat.getPosition() == CLAMPED_POS) {
            servoBrat.setPosition(UNCLAMPED_POS);
        } else if (servoBrat.getPosition() == UNCLAMPED_POS) {
            servoBrat.setPosition(CLAMPED_POS);
        }
    }

    public void clawToggle(boolean clamped) {
        if (!clamped) {
            servoBrat.setPosition(UNCLAMPED_POS);
        } else if (clamped) {
            servoBrat.setPosition(CLAMPED_POS);
        }
    }

    public void armPositionToggleAsync(boolean up){
        motorBrat.setPower(0.2);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setTargetPosition(up ? HIGH_CONSTRAINT : LOW_CONSTRAINT);
        subMode = SubMode.SUB_BUSY;
    }

    public void armPositionToggle(boolean up){
        armPositionToggleAsync(up);
        waitForSubIdle();
    }

    public void updateSub(){
        switch (subMode){
            case SUB_IDLE:
                //do nothing
                break;
            case SUB_BUSY:
                motorBrat.setPower(0.2);
                if (!motorBrat.isBusy() && Math.abs(motorBrat.getCurrentPosition())<(Math.abs(motorBrat.getTargetPosition())+5) && Math.abs(motorBrat.getCurrentPosition())>(Math.abs(motorBrat.getTargetPosition())-5)) {
                    subMode = SubMode.SUB_IDLE;
                }
                break;
        }
    }
}
