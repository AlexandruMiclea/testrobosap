package org.firstinspires.ftc.teamcode.drive.subsystems;

//import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;

//import org.firstinspires.ftc.teamcode.drive.localizer.encoder.BrokeEncoderLocalizer;
//import org.firstinspires.ftc.teamcode.drive.localizer.vision.VuforiaThread;
import org.firstinspires.ftc.teamcode.drive.localizer.BrokeEncoderLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
//import org.firstinspires.ftc.teamcode.drive.localizer.vision.TensorFlowThread;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

@Config
public class MecanumDriveMR extends SampleMecanumDriveBase { //TODO: switch to MecanumDriveREVOptimized if this succeeds

    private DcMotor motorFL, motorBL, motorBR, motorFR;
    private List<DcMotor> motors;
    private ModernRoboticsI2cGyro gyro;

    public MecanumDriveMR(HardwareMap hardwareMap) {
        super();

        //gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        //gyro.calibrate();


        motorFL = hardwareMap.dcMotor.get("MotorFL");
        motorFR = hardwareMap.dcMotor.get("MotorFR");
        motorBL = hardwareMap.dcMotor.get("MotorBL");
        motorBR = hardwareMap.dcMotor.get("MotorBR");

        motors = Arrays.asList(motorFL, motorBL, motorBR, motorFR);

        for (DcMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        //add localizer
        // setLocalizer(new BrokeEncoderLocalizer(hardwareMap));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        ModernRoboticsUsbDcMotorController controller = (ModernRoboticsUsbDcMotorController) motorFL.getController();
        DifferentialControlLoopCoefficients coefficients = controller.getDifferentialControlLoopCoefficients(motorFL.getPortNumber());
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotor motor : motors) {
            ModernRoboticsUsbDcMotorController controller = (ModernRoboticsUsbDcMotorController) motor.getController();
            controller.setDifferentialControlLoopCoefficients(motor.getPortNumber(), new DifferentialControlLoopCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD
            ));
        }
    }

    //  @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        motorFL.setPower(v);
        motorBL.setPower(v1);
        motorBR.setPower(v2);
        motorFR.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return gyro.getHeading();
    }
}
