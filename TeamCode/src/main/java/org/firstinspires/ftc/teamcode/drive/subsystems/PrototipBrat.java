package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: to the thread - (clasa Subsystem care e thread)
public class PrototipBrat {
    private int LOW_CONSTRAINT = 950;
    private int HIGH_CONSTRAINT = -380;
    private double MAX_LIFT_SPEED = 0.5, MAX_LOWER_SPEED = 0.3;

    private DcMotor motorBrat;
    private Servo servoBrat;
    private boolean isConstraints;

    public PrototipBrat(HardwareMap hardwareMap) {
        motorBrat = hardwareMap.dcMotor.get("motorBrat");
        servoBrat = hardwareMap.servo.get("servoBrat");

        motorBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBrat.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoBrat.setPosition(1);
    }

    public void setConstraints(boolean constraints){
        this.isConstraints = constraints;
    }

    public void setMotorMode(DcMotor.RunMode mode){ motorBrat.setMode(mode); }

    public DcMotor.RunMode getMotorMode(){ return motorBrat.getMode(); }

    public int getPosition() { return motorBrat.getCurrentPosition(); }

    public boolean getConstraints(){
        return isConstraints;
    }

    public boolean getIsBusy(){
        return motorBrat.isBusy();
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
        if (servoBrat.getPosition() == 0.8) {
            servoBrat.setPosition(0);
        } else if (servoBrat.getPosition() == 0) {
            servoBrat.setPosition(0.8);
        }
    }

    public void clawToggle(boolean clamped) {
        if (!clamped) {
            servoBrat.setPosition(0);
        } else if (clamped) {
            servoBrat.setPosition(0.8);
        }
    }

    public void armPositionToggle(boolean up){
        motorBrat.setPower(0.3);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setTargetPosition(up? HIGH_CONSTRAINT : LOW_CONSTRAINT);
    }
}
