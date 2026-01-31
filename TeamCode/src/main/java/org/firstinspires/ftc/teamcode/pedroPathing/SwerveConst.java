package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SwerveConst {
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.075)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .xVelocity(82)
            .yVelocity(82)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-81)
            .lateralZeroPowerAcceleration(-81)
            .centripetalScaling(.00001)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.005, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(.7, 0, 0.04, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.35, 0, 0.11, 0.06, 0))
            .automaticHoldEnd(false)
            .mass(4);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 20, 2);

    public static Follower createFollower(HardwareMap hardwareMap, Gamepad gamepad) {

        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(new SwervePedro(hardwareMap, driveConstants, gamepad))
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }


}
