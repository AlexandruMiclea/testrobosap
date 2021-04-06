package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Subsystem;

public class CollectionMechanism extends Subsystem {
    private DcMotor motorSistColectare;
    private Servo servoHoldRing;

    //TODO gasit aceste pozitii
    private double CLAMPED_POSE, UNCLAMPED_POSE;
    private int THROW_RAMP_POSE, COLLECT_POSE;
    private double MAX_LIFT_SPEED = 0.9, MAX_LOWER_SPEED = 0.9;
    private boolean isConstraints;

    public CollectionMechanism(HardwareMap hardwareMap){
        motorSistColectare = hardwareMap.dcMotor.get("motorColectare");

        motorSistColectare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSistColectare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSistColectare.setDirection(DcMotorSimple.Direction.FORWARD);

        servoHoldRing =  hardwareMap.servo.get("servoTinutInele");
        servoHoldRing.setPosition(UNCLAMPED_POSE);

        isConstraints = false;
    }

    public double getPosition(){ return motorSistColectare.getCurrentPosition(); }

    public DcMotor.RunMode getMotorMode(){ return motorSistColectare.getMode(); }

    public void setMotorMode(DcMotor.RunMode mode){ motorSistColectare.setMode(mode); }

    public void setConstraints(boolean constraints){
        this.isConstraints = constraints;
    }

    public void holdRingToggle(boolean hold){
        if(hold){
            servoHoldRing.setPosition(CLAMPED_POSE);
        } else {
            servoHoldRing.setPosition(UNCLAMPED_POSE);
        }
    }

    public void stop() { motorSistColectare.setPower(0); }

    public void moveArm(double speed) {
        if (isConstraints){
            if(speed > 0 && motorSistColectare.getCurrentPosition() > THROW_RAMP_POSE){
                stop();
            }
            else if(speed < 0 && motorSistColectare.getCurrentPosition() < COLLECT_POSE){
                stop();
            } else {
                motorSistColectare.setPower(speed * (speed > 0? MAX_LIFT_SPEED : MAX_LOWER_SPEED));
            }
        } else {
            motorSistColectare.setPower(speed * (speed > 0? MAX_LIFT_SPEED : MAX_LOWER_SPEED));
        }
    }

    public void collectToggleAsync(boolean isCollecting){
        motorSistColectare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSistColectare.setPower(0.2);
        motorSistColectare.setTargetPosition(isCollecting ? COLLECT_POSE : THROW_RAMP_POSE);
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
                if(motorSistColectare.getCurrentPosition() == motorSistColectare.getTargetPosition()){
                    subMode = SubMode.SUB_IDLE;
                }
                break;
        }
    }
}
