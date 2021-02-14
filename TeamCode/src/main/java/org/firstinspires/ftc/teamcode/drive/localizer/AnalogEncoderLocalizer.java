package org.firstinspires.ftc.teamcode.drive.localizer;

//import android.support.annotation.NonNull;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;


public class AnalogEncoderLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4175; //TODO
    public static double WHEEL_RADIUS = 6; // inch
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double ENCODER_RATIO = 1.0; // ratio between left encoder ticks per rev and right encoder ticks per rev

    public static double LATERAL_DISTANCE = 13; // inch; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

//    private AnalogSensor rightEncoder, middleEncoder;
//    private double rightPosition, middlePosition; //volts
//    private double lastMiddlePos, lastRightPos; //volts

    private  absoluteEncoder rightEncoder = new absoluteEncoder();
    private absoluteEncoder middleEncoder = new absoluteEncoder();
    BNO055IMU imu;

    public AnalogEncoderLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        rightEncoder.encoder = (AnalogSensor) hardwareMap.get("rightEncoder");
        middleEncoder.encoder = (AnalogSensor) hardwareMap.get("middleEncoder");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public static double voltageToInches(double pos) {
        return ((pos * WHEEL_RADIUS * 3.14) / 5);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                voltageToInches(rightEncoder.getTotalVoltage()),
                voltageToInches(middleEncoder.getTotalVoltage())
        );
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}

class absoluteEncoder{
    public AnalogSensor encoder;
    public double lastVoltage;
    public double totalVoltage;

//    absoluteEncoder (AnalogSensor encoder, double lastVoltage, double totalVoltage){
//        this.encoder = encoder;
//        this.lastVoltage = lastVoltage;
//        this.totalVoltage = totalVoltage;
//    }

    public double getTotalVoltage(){
        double current = encoder.readRawVoltage();

        double derivative = current - lastVoltage;

        if (derivative < 2.5){
            derivative +=5;
        } else if (derivative > 2.5){
            derivative -=5;
        }

        lastVoltage = current;

        totalVoltage +=derivative;

        return totalVoltage;

    }
}

