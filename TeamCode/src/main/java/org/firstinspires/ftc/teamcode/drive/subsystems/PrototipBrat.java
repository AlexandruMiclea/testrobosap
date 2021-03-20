package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PrototipBrat {
    private DcMotor motorBrat;
    private Servo servoBrat;
    private int lowConstraint = -370;
    private int highConstraint = -1400;
    private boolean isBusy, isConstraints;
    private double maxLiftSpeed = 0.5, maxLowerSpeed = 0.3;

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

    public int getLowConstraint() { return lowConstraint;  }

    public int getHighConstraint() { return highConstraint; }

    public boolean getConstraints(){
        return isConstraints;
    }

    public boolean getIsBusy(){
        return isBusy;
    }

    public void stop() {
        motorBrat.setPower(0);
    }

    public void liftArm(double speed) {
        if (isConstraints){
            if(motorBrat.getCurrentPosition()>highConstraint){
                stop();
            } else {
                motorBrat.setPower(speed * maxLiftSpeed);
            }
        } else {
            motorBrat.setPower(speed * maxLiftSpeed);
        }

    }

    public void lowerArm(double speed) {
        if(isConstraints){
            if(motorBrat.getCurrentPosition()<lowConstraint){
                stop();
            } else {
                motorBrat.setPower(-speed * maxLowerSpeed);
            }
        } else {
            motorBrat.setPower(-speed * maxLowerSpeed);
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

    public void toPosition(int position){
        motorBrat.setPower(0.3);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setTargetPosition(position);
        while(motorBrat.isBusy()){
            isBusy = true;
        }
        motorBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stop();
    }

}
