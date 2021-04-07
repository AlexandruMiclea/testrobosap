package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.Subsystem;

public class ThrowingMechanism extends Subsystem {
    private DcMotorEx throwWheelMotor;
    private Servo pushServo;

    private ElapsedTime timer;

    private double SERVO_PUSHED = 1, SERVO_REST = 0;
    //TODO  this is an arbitrary value
    private int SPIN_TIME = 5000;

    public ThrowingMechanism(HardwareMap hardwareMap){
        throwWheelMotor = hardwareMap.get(DcMotorEx.class, "motorAruncare");

        throwWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        throwWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        throwWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pushServo = hardwareMap.servo.get("servoImpins");
        pushServo.setPosition(SERVO_REST);

        subMode = SubMode.SUB_IDLE;

        timer = new ElapsedTime();
    }

    public double getPower(){ return throwWheelMotor.getPower(); }

    public double getVelo(){ return throwWheelMotor.getVelocity();}

    public void stop(){ throwWheelMotor.setPower(0); }

    //TODO nu stiu daca acest set de instructiuni functioneaza sau daca da skip peste ele
    public void pushRing(){
        pushServo.setPosition(SERVO_PUSHED);
        pushServo.setPosition(SERVO_REST);
    }

    public void pushRing(boolean push){
        pushServo.setPosition(push? SERVO_PUSHED : SERVO_REST);
    }

    public void rotateAsync(double speed){
        speed = Range.clip(speed, -0.9, 0.9);
        throwWheelMotor.setPower(speed);
        timer.reset();
        subMode = SubMode.SUB_BUSY;
    }

    public void rotate(double speed){
        rotateAsync(speed);
        waitForSubIdle();
    }

    @Override
    protected void updateSub() {
        switch (subMode){
            case SUB_IDLE:
                //do nothing
                break;
            case SUB_BUSY:
                if(timer.milliseconds() >= SPIN_TIME){
                    stop();
                    subMode = SubMode.SUB_IDLE;
                }
                break;
        }
    }
}
