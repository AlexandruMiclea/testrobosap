package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Subsystem;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

//TODO: to the thread - (clasa Subsystem care e thread)
public class WobbleArm extends Subsystem {
    //TODO De decomentat daca nu merge tuner ul de constraints uri si de comentat linia de jos
    private final int LOW_CONSTRAINT = -642, HIGH_CONSTRAINT = 224, MIDDLE_CONSTRAINT = 786;
//    private static int LOW_CONSTRAINT, HIGH_CONSTRAINT, MIDDLE_CONSTRAINT;

    private final double MAX_LIFT_SPEED = 0.5, MAX_LOWER_SPEED = 0.3;
    private final double CLAMPED_POS = 1, UNCLAMPED_POS = 0;

    private DcMotorEx armMotor;
    private Servo clampServo;
    private boolean isConstraints;

    public WobbleArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "motorBrat");
        clampServo = hardwareMap.servo.get("servoBrat");

        //TODO De comentat daca nu merge tuner ul de constraints uri
        //********
//        try (InputStream input = getClass().getResourceAsStream("/org/firstinspires/ftc/teamcode/drive/constants.properties")) {
//            //load a properties file
//            Properties constants = new Properties();
//            constants.load(input);
//
//            HIGH_CONSTRAINT = Integer.parseInt(constants.getProperty("WA_HIGH_CONSTRAINT"));
//            MIDDLE_CONSTRAINT = Integer.parseInt(constants.getProperty("WA_MIDDLE_CONSTRAINT"));
//            LOW_CONSTRAINT = Integer.parseInt(constants.getProperty("WA_LOW_CONSTRAINT"));
//        }
//        catch (IOException e){
//            e.printStackTrace();
//        }
        //*********

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPositionTolerance(100);

        clampServo.setPosition(CLAMPED_POS);

        subMode = SubMode.SUB_IDLE;
    }

    public void setConstraints(boolean constraints){ this.isConstraints = constraints; }

    public void setMotorMode(DcMotor.RunMode mode){ armMotor.setMode(mode); }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior beh){ armMotor.setZeroPowerBehavior(beh); }

    public void setSpeed(double speed){ armMotor.setVelocity(speed); }

    public double getSpeed(){ return armMotor.getVelocity(); }

    public DcMotor.RunMode getMotorMode(){ return armMotor.getMode(); }

    public boolean getMotorIsBusy(){ return armMotor.isBusy(); }

    public SubMode getMode(){ return subMode; }

    public int getPosition() { return armMotor.getCurrentPosition(); }

    public double getServoPosition() { return clampServo.getPosition(); }

    public int getTargetPosition() { return armMotor.getTargetPosition(); }

    public boolean getConstraints(){ return isConstraints; }

    public void stop() { armMotor.setPower(0); }

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

    public void clawToggle(boolean clamped) {
        if (!clamped) {
            clampServo.setPosition(UNCLAMPED_POS);
        } else if (clamped) {
            clampServo.setPosition(CLAMPED_POS);
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
        subMode = SubMode.SUB_BUSY;
        armMotor.setPower(customSpeed);
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
