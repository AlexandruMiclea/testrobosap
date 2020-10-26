package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PrototipSistColectare {
    private DcMotor motorSistColectare;

    public PrototipSistColectare(HardwareMap hardwareMap){
        motorSistColectare = hardwareMap.dcMotor.get("MotorSistemColectare");

        motorSistColectare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorSistColectare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorSistColectare.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intake() {motorSistColectare.setPower(0.9);}
    public void outtake() {motorSistColectare.setPower(-0.9);}
    public void stop(){motorSistColectare.setPower(0);}
}
