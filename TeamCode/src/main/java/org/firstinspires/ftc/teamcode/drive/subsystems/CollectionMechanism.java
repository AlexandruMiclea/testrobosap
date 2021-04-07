package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Subsystem;

public class CollectionMechanism extends Subsystem {
    private DcMotor collectArmMotor;
    //private Servo servoHoldRing;

    //TODO gasit aceste pozitii
    private double CLAMPED_POSE = 1, UNCLAMPED_POSE = 0;
    private int THROW_RAMP_POSE, COLLECT_POSE;
    private double MAX_LIFT_SPEED = 0.9, MAX_LOWER_SPEED = 0.9;
    private boolean isConstraints;

    public CollectionMechanism(HardwareMap hardwareMap){
        collectArmMotor = hardwareMap.dcMotor.get("motorColectare");

        collectArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);

//        servoHoldRing =  hardwareMap.servo.get("servoTinutInele");
//        servoHoldRing.setPosition(UNCLAMPED_POSE);

        //TODO true cand avem constraints
        isConstraints = false;
    }

    public double getPosition(){ return collectArmMotor.getCurrentPosition(); }

    public DcMotor.RunMode getMotorMode(){ return collectArmMotor.getMode(); }

    public void setMotorMode(DcMotor.RunMode mode){ collectArmMotor.setMode(mode); }

    public void setConstraints(boolean constraints){
        this.isConstraints = constraints;
    }

    /*public void holdRingToggle(boolean hold){
        servoHoldRing.setPosition(hold? CLAMPED_POSE : UNCLAMPED_POSE);
    }*/
    public void stop() { collectArmMotor.setPower(0); }

    public void moveArm(double speed) {
        if (isConstraints){
            if(speed > 0 && collectArmMotor.getCurrentPosition() > THROW_RAMP_POSE){
                stop();
            }
            else if(speed < 0 && collectArmMotor.getCurrentPosition() < COLLECT_POSE){
                stop();
            } else {
                collectArmMotor.setPower(speed * (speed > 0? MAX_LIFT_SPEED : MAX_LOWER_SPEED));
            }
        } else {
            collectArmMotor.setPower(speed * (speed > 0? MAX_LIFT_SPEED : MAX_LOWER_SPEED));
        }
    }

    public void collectToggleAsync(boolean isCollecting){
        collectArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectArmMotor.setPower(0.2);
        collectArmMotor.setTargetPosition(isCollecting ? COLLECT_POSE : THROW_RAMP_POSE);
        subMode = SubMode.SUB_BUSY;
    }

    public void collectToggle(boolean isCollecting){
        collectToggleAsync(isCollecting);
        waitForSubIdle();
    }

    @Override
    public void updateSub() {
        switch (subMode){
            case SUB_IDLE:
                //do nothing
                break;
            case SUB_BUSY:
                if(collectArmMotor.getCurrentPosition() == collectArmMotor.getTargetPosition()){
                    subMode = SubMode.SUB_IDLE;
                }
                break;
        }
    }
}
