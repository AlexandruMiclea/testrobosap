//package org.firstinspires.ftc.teamcode.drive.subsystems;
//
////import android.support.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
////import org.firstinspires.ftc.teamcode.drive.localizer.encoder.BrokeEncoderLocalizer;
////import org.firstinspires.ftc.teamcode.drive.localizer.vision.VuforiaThread;
//import org.firstinspires.ftc.teamcode.drive.localizer.BrokeEncoderLocalizer;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
////import org.firstinspires.ftc.teamcode.drive.localizer.vision.TensorFlowThread;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
//
//@Config
//public class MecanumDrive extends SampleMecanumDriveBase { //TODO: switch to MecanumDriveREVOptimized if this succeeds
//
//    private DcMotorEx motorFL, motorBL, motorBR, motorFR;
//    private List<DcMotorEx> motors;
//    private BNO055IMU imu;
//
//    public MecanumDrive(HardwareMap hardwareMap) {
//        super(hardwareMap);
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);
//
//
//        motorFL = hardwareMap.get(DcMotorEx.class, "MotorFL");
//        motorBL = hardwareMap.get(DcMotorEx.class, "MotorBL");
//        motorBR = hardwareMap.get(DcMotorEx.class, "MotorBR");
//        motorFR = hardwareMap.get(DcMotorEx.class, "MotorFR");
//
//        motors = Arrays.asList(motorFL, motorBL, motorBR, motorFR);
//
//        for (DcMotor motor : motors) {
//            if (RUN_USING_ENCODER) {
//                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//
//        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
//            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
//        }
//
//        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //add localizer
//        // setLocalizer(new BrokeEncoderLocalizer(hardwareMap));
//    }
//
////    @Override
////    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode runMode) {
////        PIDFCoefficients coefficients = motorFL.getPIDFCoefficients(runMode);
////        return new PIDFCoefficients(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
////    }
////
////    @Override
////    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
////        for (DcMotorEx motor : motors) {
////            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
////                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
////            ));
////        }
////    }
//
//    //  @NonNull
//    @Override
//    public List<Double> getWheelPositions() {
//        List<Double> wheelPositions = new ArrayList<>();
//        for (DcMotorEx motor : motors) {
//            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
//        }
//        return wheelPositions;
//    }
//
//    @Override
//    public List<Double> getWheelVelocities() {
//        List<Double> wheelVelocities = new ArrayList<>();
//        for (DcMotorEx motor : motors) {
//            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
//        }
//        return wheelVelocities;
//    }
//
//    @Override
//    public void setMotorPowers(double v, double v1, double v2, double v3) {
//        motorFL.setPower(v);
//        motorBL.setPower(v1);
//        motorBR.setPower(v2);
//        motorFR.setPower(v3);
//    }
//
//    @Override
//    public double getRawExternalHeading() {
//        return imu.getAngularOrientation().firstAngle;
//    }
//}