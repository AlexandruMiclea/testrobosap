package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.Subsystem;

public class ThrowingMechanism extends Subsystem {
    private DcMotor motorAruncare;

    //TODO decomentat cand e montat sau mapat in config
//    private Servo servoImpingator;

    private double SERVO_PUSHED = 1, SERVO_REST = 0;

    public ThrowingMechanism(HardwareMap hardwareMap){
        motorAruncare = hardwareMap.dcMotor.get("motorAruncare");

        motorAruncare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorAruncare.setDirection(DcMotorSimple.Direction.REVERSE);
        motorAruncare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        servoImpingator = hardwareMap.servo.get("servoImpins");
//        servoImpingator.setPosition(SERVO_REST);
    }

    public double getPower(){
        return motorAruncare.getPower();
    }


    //TODO nu stiu daca acest set de instructiuni functioneaza sau daca da skip peste ele
//    public void pushRing(){
//        servoImpingator.setPosition(SERVO_PUSHED);
//        servoImpingator.setPosition(SERVO_REST);
//    }

    public void rotateAsync( double speed){
        speed = Range.clip(speed, -0.9, 0.9);
        motorAruncare.setPower(speed);
        subMode = SubMode.SUB_BUSY;
    }

    //TODO: NU!!!! folositi functia asta fara sa dati un stop manual altfel va fi blocat intr un loop
    public void rotate( double speed){
        rotateAsync(speed);
        waitForSubIdle();
    }

    public void stop(){
        motorAruncare.setPower(0);
        subMode = SubMode.SUB_IDLE;
    }


    //TODO teoretic functia asta nu face nimic
    @Override
    protected void updateSub() {
        switch (subMode){
            case SUB_IDLE:
                //do nothing
                break;
            case SUB_BUSY:
                if(motorAruncare.getPower() == 0){
                    stop();
                }
                break;
        }

    }
}
