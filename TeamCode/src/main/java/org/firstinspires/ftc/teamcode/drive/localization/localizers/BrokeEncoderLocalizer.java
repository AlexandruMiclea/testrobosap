package org.firstinspires.ftc.teamcode.drive.localization.localizers;

//import android.support.annotation.NonNull;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class BrokeEncoderLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4175; //TODO
    public static double WHEEL_RADIUS = 2; // inch
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

//    public static double ENCODER_RATIO = 1.9621556886; // ratio between left encoder ticks per rev and right encoder ticks per rev
    public static double ENCODER_RATIO = 0.5096435547;

    public static double LATERAL_DISTANCE = 14; // inch; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

    private Encoder rightEncoder, middleEncoder;
    private BNO055IMU imu;

    public BrokeEncoderLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        middleEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "middleEncoder"));
        middleEncoder.setDirection(Encoder.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public List<Integer> getTicks(){
        return Arrays.asList(
          middleEncoder.getCurrentPosition(),
          rightEncoder.getCurrentPosition()
        );
    }

    public List<Double> getTicksPerRev(){
        return Arrays.asList(
                middleEncoder.getTicksPerRev(),
                rightEncoder.getTicksPerRev()
        );
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * ENCODER_RATIO,
                encoderTicksToInches(middleEncoder.getCurrentPosition())
        );
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getHeadingVelocity(){
        return (double) imu.getAngularVelocity().xRotationRate;
    }

    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * ENCODER_RATIO,
                encoderTicksToInches(middleEncoder.getRawVelocity())
        );
    }
}

