package org.firstinspires.ftc.teamcode.drive.localization.localizers;

//import android.support.annotation.NonNull;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.localization.absoluteEncoder;

import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;

public class AnalogEncoderLocalizer extends TwoTrackingWheelLocalizer {
    public static double WHEEL_DIAMETER = 4; // inch
    public static double MAX_VOLTAGE = 3.27;

    public static double LATERAL_DISTANCE = 14 ; // inch; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

    private absoluteEncoder middleEncoder = new absoluteEncoder();
    private absoluteEncoder rightEncoder = new absoluteEncoder();

    private BNO055IMU imu;

    public AnalogEncoderLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));


        middleEncoder.encoder = hardwareMap.get(AnalogInput.class, "middleEncoder");
        rightEncoder.encoder = hardwareMap.get(AnalogInput.class, "rightEncoder");

        middleEncoder.setInitVolt(middleEncoder.encoder.getVoltage());
        rightEncoder.setInitVolt(rightEncoder.encoder.getVoltage());


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public static double voltageToInches(double pos) {
        return ((pos * WHEEL_DIAMETER * Math.PI) / MAX_VOLTAGE);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                voltageToInches(rightEncoder.getVoltageWithIndex()),
                voltageToInches(middleEncoder.getVoltageWithIndex())
        );
    }

     public List <Double> getTotalVoltagesWithIndex(){
        return Arrays.asList(
                rightEncoder.getVoltageWithIndex(),
                middleEncoder.getVoltageWithIndex()
        );
    }

    public List <Double> getIndex(){
        return Arrays.asList(
                rightEncoder.getTurnIndex(),
                middleEncoder.getTurnIndex()
        );
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

}

