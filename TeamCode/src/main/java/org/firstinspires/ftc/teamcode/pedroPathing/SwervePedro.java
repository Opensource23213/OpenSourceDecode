package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.Drivetrain;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/**
 * This is the Mecanum class, a child class of Drivetrain. This class takes in inputs Vectors for driving, heading
 * correction, and translational/centripetal correction and returns an array with wheel powers.
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 4/30/2025
 */
@Config
public class SwervePedro extends Drivetrain {
    public MecanumConstants constants;
    public swerve_drive chassis = new swerve_drive();

    private final VoltageSensor voltageSensor;
    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;
    private final List<DcMotorEx> motors;
    public boolean teleop = false;
    public boolean x_out = false;
    public boolean zero = false;
    public boolean stop_and_point = false;
    public double angle = 0;
    public Gamepad gamepad1;

    /**
     * This creates a new Mecanum, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel.
     *
     * @param hardwareMap      this is the HardwareMap object that contains the motors and other hardware
     * @param mecanumConstants this is the MecanumConstants object that contains the names of the motors and directions etc.
     */
    public SwervePedro(HardwareMap hardwareMap, MecanumConstants mecanumConstants, Gamepad gamepad) {
        constants = mecanumConstants;
        teleop = false;
        gamepad1 = gamepad;
        this.maxPowerScaling = mecanumConstants.maxPower;
        this.motorCachingThreshold = mecanumConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = true;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        chassis.initialize(hardwareMap);

        motors = Arrays.asList(chassis.fr.wheel, chassis.fl.wheel, chassis.rr.wheel, chassis.rl.wheel);

        setMotorsToBrake();
        breakFollowing();

        Vector copiedFrontLeftVector = mecanumConstants.frontLeftVector.normalize();
        vectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};
    }

    public void updateConstants() {
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array of four doubles, one for each wheel's motor power.
     * <p>
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower    this Vector points in the direction of the robot's current heading, and
     *                        the magnitude tells the robot how much it should turn and in which
     *                        direction.
     * @param pathingPower    this Vector points in the direction the robot needs to go to continue along
     *                        the Path.
     * @param robotHeading    this is the current heading of the robot, which is used to calculate how
     *                        much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        angle = robotHeading;
        setMotorsToBrake();
        if (correctivePower.getMagnitude() > maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        // the powers for the wheel vectors
        double[] wheelPowers = new double[4];

        // This contains a copy of the mecanum wheel vectors
        Vector[] mecanumVectorsCopy = new Vector[4];

        // this contains the pathing vectors, one for each side (heading control requires 2)
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == maxPowerScaling) {
            // checks for corrective power equal to max power scaling in magnitude. if equal, then set pathing power to that
            truePathingVectors[0] = correctivePower.copy();
            truePathingVectors[1] = correctivePower.copy();
        } else {
            // corrective power did not take up all the power, so add on heading power
            Vector leftSideVector = correctivePower.minus(headingPower);
            Vector rightSideVector = correctivePower.plus(headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                //if the combined corrective and heading power is greater than 1, then scale down heading power
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower, maxPowerScaling), findNormalizingScaling(correctivePower, headingPower.times(-1), maxPowerScaling));
                truePathingVectors[0] = correctivePower.minus(headingPower.times(headingScalingFactor));
                truePathingVectors[1] = correctivePower.plus(headingPower.times(headingScalingFactor));
            } else {
                // if we're here then we can add on some drive power but scaled down to 1
                Vector leftSideVectorWithPathing = leftSideVector.plus(pathingPower);
                Vector rightSideVectorWithPathing = rightSideVector.plus(pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    // too much power now, so we scale down the pathing vector
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower, maxPowerScaling), findNormalizingScaling(rightSideVector, pathingPower, maxPowerScaling));
                    truePathingVectors[0] = leftSideVector.plus(pathingPower.times(pathingScalingFactor));
                    truePathingVectors[1] = rightSideVector.plus(pathingPower.times(pathingScalingFactor));
                } else {
                    // just add the vectors together and you get the final vector
                    truePathingVectors[0] = leftSideVectorWithPathing.copy();
                    truePathingVectors[1] = rightSideVectorWithPathing.copy();
                }
            }
        }

        truePathingVectors[0] = truePathingVectors[0].times(2.0);
        truePathingVectors[1] = truePathingVectors[1].times(2.0);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            // this copies the vectors from mecanumVectors but creates new references for them
            mecanumVectorsCopy[i] = vectors[i].copy();

            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent());

        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] *= voltageNormalized;
            }
        }

        double wheelPowerMax = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])), Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));

        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] = (wheelPowers[0] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[1] = (wheelPowers[1] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[2] = (wheelPowers[2] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[3] = (wheelPowers[3] / wheelPowerMax) * maxPowerScaling;
        }
        double FL = wheelPowers[0];
        double FR = wheelPowers[2];
        double BL = wheelPowers[1];
        double[] output = new double[3];
        output[0] = (FL + FR) / 2.0;
        output[1] = (FL - BL) / 2.0;
        output[2] = (BL - FR) / 2.0;
        return output;
    }

    /**
     * This sets the motors to the zero power behavior of brake.
     */
    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * This sets the motors to the zero power behavior of float.
     */
    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToBrake();
        chassis.fl.pod1servo.setPower(0);
        chassis.fr.pod1servo.setPower(0);
        chassis.rl.pod1servo.setPower(0);
        chassis.rr.pod1servo.setPower(0);
        //setMotorsToFloat();
    }

    public void runDrive(double[] doubles) {
        if(x_out){
            chassis.x();
        }else if(zero){
            chassis.zero();
        }else {
            if(teleop){
                double real_angle = Math.toDegrees(angle);
                if(real_angle < 0){
                    real_angle = abs(real_angle);
                }else{
                    real_angle = 360 - real_angle;
                }
                real_angle -= 90;
                if(real_angle < 0){
                    real_angle += 360;
                }
                real_angle = Math.toRadians(real_angle);
                double fwd = -gamepad1.left_stick_y * Math.cos(real_angle) + (gamepad1.left_stick_x) * Math.sin(real_angle);
                double str = (gamepad1.left_stick_x) * Math.cos(real_angle) + gamepad1.left_stick_y * Math.sin(real_angle);
                chassis.drive(fwd, str, gamepad1.right_stick_x);
            }else {
                chassis.drive(doubles[0], doubles[1], doubles[2]);
            }
        }
        x_out = false;
        stop_and_point = false;
        zero = false;
    }

    @Override
    public void startTeleopDrive() {
        teleop = true;
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        teleop = true;
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    public void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    public double xVelocity() {
        return constants.xVelocity;
    }

    public double yVelocity() {
        if (stop_and_point){
            return 1;
        }else{
            return 0;
        }
    }
    @Override
    public void setXVelocity(double xMovement) {
        //Makes the swerve wheels "x_out"
        x_out = true;
    }
    @Override
    public void setYVelocity(double yMovement) {
        //Makes the swerve wheels point in the correct direction without moving
        stop_and_point = true;
    }

    public double getStaticFrictionCoefficient() {

        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        //Makes all of the swerve wheels zero out
        zero = true;
        return null;
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }



    public static double fr_offset = -400;
    public static double fl_offset = 1410;
    public static double rr_offset = 1000;
    public static double rl_offset = 100;
    public double p = .001, i = 0, d = .00004;

    public double f = 0.1;
    public static double degree_offset = 0;
    public static double change = 150;

    public class swerve_drive {
        double L = 0;
        double W = 0;
        double R = 0;
        double A = 0;
        double B = 0;
        double C = 0;
        double D = 0;
        double frs = 0;
        double fls = 0;
        double rrs = 0;
        double rls = 0;
        double fra = 0;
        double fla = 0;
        double rra = 0;
        double rla = 0;
        public swerve_drive.swerve_pod fr = new swerve_drive.swerve_pod();
        public swerve_drive.swerve_pod fl = new swerve_drive.swerve_pod();
        public swerve_drive.swerve_pod rr = new swerve_drive.swerve_pod();
        public swerve_drive.swerve_pod rl = new swerve_drive.swerve_pod();

        public void initialize(HardwareMap hardwareMap) {
            fr.initialize(hardwareMap, "fr_motor", "fr_servo", "fr_analog");
            fl.initialize(hardwareMap, "fl_motor", "fl_servo", "fl_analog");
            rr.initialize(hardwareMap, "rr_motor", "rr_servo", "rr_analog");
            rl.initialize(hardwareMap, "rl_motor", "rl_servo", "rl_analog");
        }
        public class swerve_pod {

            public PIDController controller;

            public double position = 0;
            public CRServo pod1servo = null;
            public DcMotorEx wheel = null;
            public AnalogInput pod1rotation = null;
            public ElapsedTime time = new ElapsedTime();
            public double wheel_pos;
            public double target;
            public double speed = 0;
            public double power = 0;
            public double wheel_power = 0;

            public void initialize(HardwareMap hardwareMap, String wheel_name, String servo_name, String analog_name) {
                controller = new PIDController(p, i, d);
                wheel = hardwareMap.get(DcMotorEx.class, wheel_name);
                wheel.setDirection(DcMotorSimple.Direction.REVERSE);
                pod1servo = hardwareMap.get(CRServo.class, servo_name);
                pod1rotation = hardwareMap.get(AnalogInput.class, analog_name);
                time.reset();
            }

            public void move(double offset, double inversion) {
                wheel_pos = (abs(inversion - pod1rotation.getVoltage() / 3.3) * 3600) + offset;
                if (wheel_pos < 0) {
                    wheel_pos += 3600;
                }
                if (wheel_pos > 3600) {
                    wheel_pos -= 3600;
                }
                target = position * 10;
                if (abs(target - wheel_pos) > 900 && abs(target - wheel_pos) < 2700) {
                    target -= 1800;
                    wheel_power *= -1;
                }
                if (target < 0) {
                    target += 3600;
                }
                if (target - wheel_pos > 1800) {
                    target -= 3600;
                } else if (target - wheel_pos < -1800) {
                    target += 3600;
                }
                controller.setPID(p, i, d);
                double pid = controller.calculate(wheel_pos, target);
                double ff = Math.cos(Math.toRadians(target)) * f;
                power = pid + ff;
                if (inversion == 1) {
                    pod1servo.setPower(-power);
                } else {
                    pod1servo.setPower(power);
                }
                if(!stop_and_point) {
                    if (teleop) {
                        wheel.setPower(wheel_power);
                    } else {
                        wheel.setPower(wheel_power / (abs(target - wheel_pos) / change));
                    }
                }
            }
        }
        public void zero(){
            fr.position = 0;
            fr.wheel_power = 0;
            fl.position = 0;
            fl.wheel_power = 0;
            rr.position = 0;
            rr.wheel_power = 0;
            rl.position = 0;
            rl.wheel_power = 0;
            fr.move(fr_offset,1);
            fl.move(fl_offset,1);
            rr.move(rr_offset,1);
            rl.move(rl_offset,1);

        }
        public void x(){
            fr.position = 315;
            fr.wheel_power = 0;
            fl.position = 45;
            fl.wheel_power = 0;
            rr.position = 45;
            rr.wheel_power = 0;
            rl.position = 315;
            rl.wheel_power = 0;
            fr.move(fr_offset,1);
            fl.move(fl_offset,1);
            rr.move(rr_offset,1);
            rl.move(rl_offset,1);

        }
        public void drive(double fwd, double str, double yaw) {
            L = 8.25;
            W = 13.375;
            R = Math.sqrt(L * L + W * W);
            A = str - yaw * L / R;
            B = str + yaw * L / R;
            C = fwd - yaw * W / R;
            D = fwd + yaw * W / R;
            frs = Math.sqrt(B * B + C * C);
            fls = Math.sqrt(B * B + D * D);
            rrs = Math.sqrt(A * A + C * C);
            rls = Math.sqrt(A * A + D * D);
            fra = Math.toDegrees(Math.atan2(B, C)) + degree_offset;
            fla = Math.toDegrees(Math.atan2(B, D)) + degree_offset;
            rra = Math.toDegrees(Math.atan2(A, C)) + degree_offset;
            rla = Math.toDegrees(Math.atan2(A, D)) + degree_offset;
            double max = Math.max(fls, frs);
            max = Math.max(max, rrs);
            max = Math.max(max, rls);
            if (max > 1) {
                fls = fls / max;
                frs = frs / max;
                rrs = rrs / max;
                rls = rls / max;
            }
            if (fra < 0) {
                fra += 360;
            }
            if (fla < 0) {
                fla += 360;
            }
            if (rra < 0) {
                rra += 360;
            }
            if (rla < 0) {
                rla += 360;
            }
            fr.position = fra;
            fr.wheel_power = frs;
            fl.position = fla;
            fl.wheel_power = fls;
            rr.position = rra;
            rr.wheel_power = rrs;
            rl.position = rla;
            rl.wheel_power = rls;
            fr.move(fr_offset, 1);
            fl.move(fl_offset, 1);
            rr.move(rr_offset, 1);
            rl.move(rl_offset, 1);
        }

    }
}