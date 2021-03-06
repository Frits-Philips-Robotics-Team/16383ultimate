package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.18110236; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double PARALLEL_X = 6.1; // X is the up and down direction
    public static double PARALLEL_Y = -1.378; // Y is the strafe direction
    public static double PERPENDICULAR_X = 6.61;
    public static double PERPENDICULAR_Y = 1.969;

    final double MULTIPLIER_X = 1.02; //0.96585075 1.01121930 1.01869158
    final double MULTIPLIER_Y = 1.02; // 1.00757515 1.01658649 1.0296

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis

    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rrDrive"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rlDrive"));
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
        this.drive = drive;
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * MULTIPLIER_X,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * MULTIPLIER_Y
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())


        );
    }
}