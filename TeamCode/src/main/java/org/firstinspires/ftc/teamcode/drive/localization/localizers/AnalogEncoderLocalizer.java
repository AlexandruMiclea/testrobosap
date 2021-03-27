package org.firstinspires.ftc.teamcode.drive.localization.localizers;

//import android.support.annotation.NonNull;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AbsoluteEncoder;

import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;

public class AnalogEncoderLocalizer extends TwoTrackingWheelLocalizer {
    private static double WHEEL_DIAMETER = 4; // inch

    private static double LATERAL_DISTANCE = 14 ; // inch; distance between the left and right wheels
    private static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

    private AbsoluteEncoder middleEncoder, rightEncoder;

    private BNO055IMU imu;

    public AnalogEncoderLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), //right wheel distance from center in inch
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) //front wheel distance from center in inch
        ));

        rightEncoder = new AbsoluteEncoder(hardwareMap.get(AnalogInput.class, "rightEncoder"));
        middleEncoder = new AbsoluteEncoder(hardwareMap.get(AnalogInput.class, "middle"));

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public double voltageToInches(AbsoluteEncoder encoder) {
        return ((encoder.getVoltageWithIndex() * WHEEL_DIAMETER * Math.PI) / encoder.getMaxVoltage());
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                voltageToInches(rightEncoder),
                voltageToInches(middleEncoder)
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

    public List <Double> getMaxVoltageRev(){
        return Arrays.asList(
                rightEncoder.getMaxVoltageRev(),
                middleEncoder.getMaxVoltageRev()
        );
    }



    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

}

