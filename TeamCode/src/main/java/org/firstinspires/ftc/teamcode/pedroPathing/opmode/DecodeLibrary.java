package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Config
//@Disabled
public class DecodeLibrary extends OpMode {
    public static double color = 0;
    public static double angle_offset = 0;
    public static double moving_offset = 30;
    public static double robot_velocity_stop_shoot = 25;
    public double old_dis = 0;
    public DcMotorEx intake = null;
    public spinny spinny = new spinny();
    public shooter shooter = new shooter();
    public turret turret = new turret();
    public CameraCode cameraCode = new CameraCode();
    public static double adjust = 6000;
    public double flippy_up = 0.32;
    public double flippy_down = .68;
    public double flippy_pos = flippy_up;
    public Servo flippy = null;
    public static double sppeed = 0;
    public static double fllap = .04;

    @Override
    public void init(){
        initialize();
    }

    public void initialize() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flippy = hardwareMap.get(Servo.class, "flippy");
        cameraCode = new CameraCode();
        cameraCode.init();
        spinny.initialize();
        shooter.initialize();
        turret.initialize();
        drive_init();
    }

    @Override
    public void loop() {

    }
    public Button2 button2 = new Button2();
    public Button1 button1 = new Button1();
    public class Button2{
        List<String> button = new ArrayList<>();
        List<String> nowbutton = new ArrayList<>();
        List<String> lastbutton = new ArrayList<>();
        String type = "";
        public void button(){
            if (gamepad2.a && !button.contains("a")) {
                button.add("a");
            }
            if(gamepad2.b && !button.contains("b")){
                button.add("b");
            }
            if (gamepad2.x && !button.contains("x")){
                button.add("x");
            }
            if (gamepad2.y && !button.contains("y")){
                button.add("y");
            }
            if (gamepad2.right_bumper && !button.contains("r1")){
                button.add("r1");
            }
            if (gamepad2.left_bumper && !button.contains("l1")){
                button.add("l1");
            }
            if (gamepad2.left_trigger > .4 && !button.contains("l2")){
                button.add("l2");
            }
            if (gamepad2.right_trigger > .4 && !button.contains("r2")){
                button.add("r2");
            }
            if (gamepad2.dpad_up && !button.contains("up")){
                button.add("up");
            }
            if (gamepad2.dpad_down && !button.contains("down")){
                button.add("down");
            }
            if (gamepad2.dpad_left && !button.contains("left")){
                button.add("left");
            }
            if (gamepad2.dpad_right && !button.contains("right")){
                button.add("right");
            }
            if (gamepad2.ps && !button.contains("ps")){
                button.add("ps");
            }
            if (gamepad2.left_stick_button && !button.contains("l3")){
                button.add("l3");
            }
            if (gamepad2.right_stick_button && !button.contains("r3")){
                button.add("r3");
            }
            endbutton();
            ButtonControl();
            nowbutton.clear();
        }
        public void ButtonControl(){

        }
        public void endbutton(){
            if (!gamepad2.a && button.contains("a")) {
                button.remove("a");
                nowbutton.add("a");
            }
            if(!gamepad2.b && button.contains("b")){
                button.remove("b");
                nowbutton.add("b");
            }
            if (!gamepad2.x && button.contains("x")){
                button.remove("x");
                nowbutton.add("x");
            }
            if (!gamepad2.y && button.contains("y")){
                button.remove("y");
                nowbutton.add("y");
            }
            if (!gamepad2.right_bumper && button.contains("r1")){
                button.remove("r1");
                nowbutton.add("r1");
            }
            if (!gamepad2.left_bumper && button.contains("l1")){
                button.remove("l1");
                nowbutton.add("l1");
            }
            if (!(gamepad2.left_trigger > .4) && button.contains("l2")){
                button.remove("l2");
                nowbutton.add("l2");
            }
            if (!(gamepad2.right_trigger > .4) && button.contains("r2")){
                button.remove("r2");
                nowbutton.add("r2");
            }
            if (!gamepad2.dpad_up && button.contains("up")){
                button.remove("up");
                nowbutton.add("up");
            }
            if (!gamepad2.dpad_down && button.contains("down")){
                button.remove("down");
                nowbutton.add("down");
            }
            if (!gamepad2.dpad_left && button.contains("left")){
                button.remove("left");
                nowbutton.add("left");
            }
            if (!gamepad2.dpad_right && button.contains("right")){
                button.remove("right");
                nowbutton.add("right");
            }
            if (!gamepad2.ps && button.contains("ps")){
                button.remove("ps");
                nowbutton.add("ps");
            }
            if (!gamepad2.left_stick_button && button.contains("l3")){
                button.remove("l3");
                nowbutton.add("l3");
            }
            if (!gamepad2.right_stick_button && button.contains("r3")){
                button.remove("r3");
                nowbutton.add("r3");
            }
        }
    }
    public class Button1{
        List<String> button = new ArrayList<>();
        List<String> nowbutton = new ArrayList<>();
        List<String> lastbutton = new ArrayList<>();
        String type = "";
        public void button(){
            if (gamepad1.a && !button.contains("a")) {
                button.add("a");
            }
            if(gamepad1.b && !button.contains("b")){
                button.add("b");
            }
            if (gamepad1.x && !button.contains("x")){
                button.add("x");
            }
            if (gamepad1.y && !button.contains("y")){
                button.add("y");
            }
            if (gamepad1.right_bumper && !button.contains("r1")){
                button.add("r1");
            }
            if (gamepad1.left_bumper && !button.contains("l1")){
                button.add("l1");
            }
            if (gamepad1.left_trigger > .4 && !button.contains("l2")){
                button.add("l2");
            }
            if (gamepad1.right_trigger > .4 && !button.contains("r2")){
                button.add("r2");
            }
            if (gamepad1.dpad_up && !button.contains("up")){
                button.add("up");
            }
            if (gamepad1.dpad_down && !button.contains("down")){
                button.add("down");
            }
            if (gamepad1.dpad_left && !button.contains("left")){
                button.add("left");
            }
            if (gamepad1.dpad_right && !button.contains("right")){
                button.add("right");
            }
            if (gamepad1.ps && !button.contains("ps")){
                button.add("ps");
            }
            if (gamepad1.left_stick_button && !button.contains("l3")){
                button.add("l3");
            }
            if (gamepad1.right_stick_button && !button.contains("r3")){
                button.add("r3");
            }
            endbutton();
            ButtonControl();
            nowbutton.clear();
        }
        public void ButtonControl(){
            if (nowbutton.contains("l1")) {
                intake.setPower(-1);
            }if(nowbutton.contains("a")){
                intake.setPower(0);
            }
            if(nowbutton.contains("r1")){
                robot_going_forward = !robot_going_forward;
            }
            /*



            GAMEPAD1 LEFT TRIGGER SPINS THE SPINDEXER INFINITELY



             */
            if(gamepad1.left_trigger > .4){
                if(abs(follower.getVelocity().getMagnitude()) < robot_velocity_stop_shoot && gamepad1.right_trigger < .4 && turret.shootable) {
                    flippy_pos = flippy_up;
                }else{
                    flippy_pos = flippy_down;
                }

            }else{
                flippy_pos = flippy_down;
            }
            if((spinny.intakeswitch1.getState() && spinny.intakeswitch2.getState() && spinny.switchwaspressed) || nowbutton.contains("r2")) {
                spinny.position -= 1;
                spinny.switchwaspressed = false;
                shooter.rpms.add(shooter.shoot1.getVelocity());
                if (spinny.position > 3) {
                    spinny.position = 1;
                } else if (spinny.position < 1) {
                    spinny.position = 3;
                }
            }



            if(lastbutton.contains("b")){
                if(old_dis > 0) {
                    shooter.speed = 247.01 * (cameraCode.distance_from_target + (cameraCode.distance_from_target - old_dis) * moving_offset) + 1247.05;
                }
                old_dis = cameraCode.distance_from_target;
                if(nowbutton.contains("b")) {
                    spinny.position = 1;
                    shooter.speed = 0;
                    intake.setPower(0);
                    flippy_pos = flippy_down;
                    lastbutton.remove("b");
                }
            }else{
                if(nowbutton.contains("b")) {
                    spinny.position = 1;
                    shooter.speed = 1;
                    intake.setPower(1);
                    flippy_pos = flippy_up;
                    lastbutton.add("b");
                }
            }
        }
        public void endbutton(){
            if (!gamepad1.a && button.contains("a")) {
                button.remove("a");
                nowbutton.add("a");
            }
            if(!gamepad1.b && button.contains("b")){
                button.remove("b");
                nowbutton.add("b");
            }
            if (!gamepad1.x && button.contains("x")){
                button.remove("x");
                nowbutton.add("x");
            }
            if (!gamepad1.y && button.contains("y")){
                button.remove("y");
                nowbutton.add("y");
            }
            if (!gamepad1.right_bumper && button.contains("r1")){
                button.remove("r1");
                nowbutton.add("r1");
            }
            if (!gamepad1.left_bumper && button.contains("l1")){
                button.remove("l1");
                nowbutton.add("l1");
            }
            if (!(gamepad1.left_trigger > .4) && button.contains("l2")){
                button.remove("l2");
                nowbutton.add("l2");
            }
            if (!(gamepad1.right_trigger > .4) && button.contains("r2")){
                button.remove("r2");
                nowbutton.add("r2");
            }
            if (!gamepad1.dpad_up && button.contains("up")){
                button.remove("up");
                nowbutton.add("up");
            }
            if (!gamepad1.dpad_down && button.contains("down")){
                button.remove("down");
                nowbutton.add("down");
            }
            if (!gamepad1.dpad_left && button.contains("left")){
                button.remove("left");
                nowbutton.add("left");
            }
            if (!gamepad1.dpad_right && button.contains("right")){
                button.remove("right");
                nowbutton.add("right");
            }
            if (!gamepad1.ps && button.contains("ps")){
                button.remove("ps");
                nowbutton.add("ps");
            }
            if (!gamepad1.left_stick_button && button.contains("l3")){
                button.remove("l3");
                nowbutton.add("l3");
            }
            if (!gamepad1.right_stick_button && button.contains("r3")){
                button.remove("r3");
                nowbutton.add("r3");
            }
        }
    }


    public double power_level = 1;
    //public Follower follower;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public IMU imu;
    public boolean robot_going_forward = true;
    public Follower follower;
    public void drive_init(){
        follower = Constants.createFollower(hardwareMap);
        if (color == 0) {
            follower.setPose(new Pose(0, 0, Math.toRadians(-90)));
        }else {
            follower.setPose(new Pose(0, 0, Math.toRadians(90)));
        }
        front_left = hardwareMap.get(DcMotor.class, "leftFront");
        front_right = hardwareMap.get(DcMotor.class, "rightFront");
        rear_left = hardwareMap.get(DcMotor.class, "leftRear");
        rear_right = hardwareMap.get(DcMotor.class, "rightRear");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void drive(){
        follower.update();
        Pose poseEstimate = follower.getPose();
        double angle = poseEstimate.getHeading();
        double axial = 0;
        double lateral = 0;
        double yaw = 0; // set control to the sticks
        axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;


        if (gamepad1.ps) {
            telemetry.addData("Yaw", "Resetting\n");
            follower.setPose(new Pose(poseEstimate.getX(), poseEstimate.getY(), 0));
        }

        //elbow1.setPosition(servo1pose);
        //elbow2.setPosition(servo2pose);

        double leftFrontPower = (axial + lateral + yaw) * power_level;
        double rightFrontPower = (axial - lateral - yaw) * power_level;
        double leftBackPower = (axial - lateral + yaw) * power_level;
        double rightBackPower = (axial + lateral - yaw) * power_level;

        // If the sticks are being used

        double yaw_rad = /*orientation.getYaw(AngleUnit.RADIANS)*/angle + 3.14159 / 2;
        double temp = axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
        lateral = -axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
        //double temp = axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
        //lateral = -axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
        axial = temp;

        // Combie the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.

        leftFrontPower = (axial + lateral + yaw) * power_level;
        rightFrontPower = (axial - lateral - yaw) * power_level;
        leftBackPower = (axial - lateral + yaw) * power_level;
        rightBackPower = (axial + lateral - yaw) * power_level;
        // Normalize the values so no wheel power exceeds 00%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
        max = Math.max(max, abs(leftBackPower));
        max = Math.max(max, abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }//Arm code Shoulder

        front_left.setPower(leftFrontPower);
        front_right.setPower(rightFrontPower);
        rear_left.setPower(leftBackPower);
        rear_right.setPower(rightBackPower);
        if(leftBackPower + leftFrontPower + rightBackPower + rightFrontPower > 0 && abs(follower.getVelocity().getMagnitude()) > 30){
            robot_going_forward = true;
        }else if(leftBackPower + leftFrontPower + rightBackPower + rightFrontPower < 0 && abs(follower.getVelocity().getMagnitude()) > 30){
            robot_going_forward = false;
        }
        telemetry.addData("Is the robot going forward?", robot_going_forward);


    }
    public class CameraCode{
        public Limelight3A limelight;
        public double x = 0;
        public double y = 0;
        public double r = 0;
        public double z = 0;
        public double ticks_per_degree = 5.228;
        public double tag_to_target_distance = .47;
        public double distance_from_target = 0;
        public double angle_from_target = 0;
        public double robot_x = 0;
        public double robot_y = 0;
        public double robot_angle;
        public boolean robot_auto_on = false;
        public double feet = .3048;
        public double angle = 0;
        public LLResult result;
        public void init() {
            limelight = hardwareMap.get(Limelight3A.class,"limelight");
            limelight.start();
            if(color == 0) {
                limelight.pipelineSwitch(4);
            }else{
                limelight.pipelineSwitch(5);
            }
            telemetry.setMsTransmissionInterval(11);
            limelight.start();
        }

        public void camera_calculations(){
            result = limelight.getLatestResult();
            if(result.isValid()) {
                Pose3D botpose = result.getFiducialResults().get(0).getTargetPoseRobotSpace();
                x = botpose.getPosition().x - .06;
                y = botpose.getPosition().z;
                r = Math.sqrt(x * x + y * y);
                double B = Math.toRadians(180 - Math.toDegrees(angle));
                double other_angle = Math.toRadians(botpose.getOrientation().getPitch());
                angle = other_angle  - Math.atan(x/y);
                distance_from_target = Math.sqrt(r*r + tag_to_target_distance * tag_to_target_distance - 2 * r * tag_to_target_distance * Math.cos(B));
                angle_from_target = Math.toDegrees(Math.asin((r * Math.sin(B)) / distance_from_target));
                robot_angle = 41 -  angle_from_target;

                robot_x = distance_from_target * Math.cos(Math.toRadians(robot_angle)) - 6 * feet;
                robot_y = 6 * feet - distance_from_target * Math.sin(Math.toRadians(robot_angle));
                if(result.getFiducialResults().get(0).getFiducialId() == 24){
                    robot_y = (distance_from_target * Math.cos(Math.toRadians(robot_angle)) - 6 * feet) * -1;
                    robot_x = 6 * feet - distance_from_target * Math.sin(Math.toRadians(robot_angle));
                }
                robot_auto_on = ((robot_y >= robot_x && robot_y >= -robot_x && robot_y >= 0) || (abs(robot_y) * -1 <= robot_x - .8 && abs(robot_y) * -1 <= -robot_x - .8 && abs(robot_y) * -1 <= -.8));
                telemetry.addData("Limelight xb:", x);
                telemetry.addData("Limelight y:", y);
                telemetry.addData("Limelight r:", r);
                telemetry.addData("Limelight angle from apriltag:", Math.toDegrees(angle));
                telemetry.addData("Robot distance from target:", distance_from_target);
                telemetry.addData("Angle C:", angle_from_target);
                telemetry.addData("Robot Angle:", robot_angle);
                telemetry.addData("Robot X:", robot_x);
                telemetry.addData("Robot Y:", robot_y);
                telemetry.addData("Robot in zone?:", robot_auto_on);
            }else{
                robot_auto_on = false;
                telemetry.addLine("Nothing");
            }
        }
    }
    public class shooter{
        public Servo flap;
        public DcMotorEx shoot1;
        public DcMotorEx shoot2;
        public double speed = 0;
        public double position = 0.04;
        public ArrayList<Double> rpms = new ArrayList<>();
        public void initialize(){
            flap = hardwareMap.get(Servo.class, "flap");
            shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
            shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
            shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void shooting(){
            if((cameraCode.robot_y <= cameraCode.robot_x - .8 && cameraCode.robot_y <= -cameraCode.robot_x - .8 && cameraCode.robot_y <= -.8)){
                speed = 2300;
            }
            shoot1.setVelocity(speed);
            shoot2.setVelocity(speed);
            if(speed < 1680){
                position = .21;
            }else{
                position = .04;
            }
            if(speed == 2300){
                flap.setPosition(position);
            }else {
                flap.setPosition(position + (speed - shoot1.getVelocity()) / adjust);
            }
        }
    }
    public class spinny{
        public double position = 4;
        public double turns = 0;
        public double spinny_at = 0;
        public PIDController controller;

        public double p = 0.0003, i = 0, d = 0;

        public double f = 0.01;
        public CRServo spinny = null;
        public CRServo spinny2 = null;
        public AnalogInput spinnypos = null;
        public ElapsedTime time = new ElapsedTime();
        public double yo;
        public double spin_pos;
        public double target;
        public double speed = 0;
        public double old_speed = 0;
        public boolean jammed = false;
        public boolean moving = false;
        public double power = 0;
        public DigitalChannel intakeswitch1;
        public DigitalChannel intakeswitch2;
        public boolean switchwaspressed = false;
        public double balls = 0;
        public ElapsedTime flipoffset;
        public void initialize(){
            controller = new PIDController(p, i, d);
            flipoffset = new ElapsedTime();
            spinny = hardwareMap.get(CRServo.class, "spinny");
            spinny2 = hardwareMap.get(CRServo.class, "spinny2");
            spinnypos = hardwareMap.get(AnalogInput.class, "spinnypos");
            intakeswitch1 = hardwareMap.get(DigitalChannel.class, "intakeswitch1");
            intakeswitch2 = hardwareMap.get(DigitalChannel.class, "intakeswitch2");
            time.reset();
        }
        public void spin(){

            yo = abs(spinnypos.getVoltage() / 3.3) * 4800;
            if(time.milliseconds() > 20){
                speed = abs(yo - old_speed);
                old_speed = yo + 0;
                time.reset();
            }

            if(abs(yo - spinny_at) > 2200){
                if(spinny_at < yo){
                    turns += 1;
                }else{
                    turns -= 1;
                }
            }
            if(turns >= 3 || turns <= -3){
                turns = 0;
            }

            spinny_at = yo + 0;
            spin_pos = spinny_at - turns * 1200;
            if(spin_pos > 3600){
                spin_pos -= 3600;
            }else if(spin_pos < 0){
                spin_pos += 3600;
            }
            if(!robot_going_forward && flippy_pos == flippy_down) {
                target = (position + .5) * 1200 + 400;
            }else{
                target = position * 1200 + 400;
            }
            if(target - spin_pos > 1800){
                target -= 3600;
            }
            controller.setPID(p, i, d);
            double pid = controller.calculate(spin_pos, target);
            double ff = Math.cos(Math.toRadians(target)) * f;
            power = pid + ff;
            if(power > .2){
                power *= -1;
            }
            if(abs(power) > .2 && !jammed && moving && speed < 1){
                position -= 1;
                if(position > 3){
                    position -= 3;
                }
                jammed = true;
            }
            else if(abs(power) > .2 && speed > 60){
                moving = true;
            }else if(abs(power) < .05){
                moving = false;
            }
            if(abs(spin_pos - target) < 200 && !moving){
                jammed = false;
            }
            if(!intakeswitch1.getState() || !intakeswitch2.getState()){
                switchwaspressed = true;
            }if(gamepad1.right_trigger > .4 && gamepad1.left_trigger > .4){
                power = 1;
            }else if(position == 0 || gamepad1.left_trigger > .4){
                power = -1;
                //intake.setPower(1);

            }else if(position == 4){
                power = 0;
            }
            spinny.setPower(-power);
            spinny2.setPower(-power);
            telemetry.addData("spinny target", target);
            telemetry.addData("spinny position", spin_pos);
            telemetry.addData("switch 1", intakeswitch1.getState());
            telemetry.addData("switch 2", intakeswitch2.getState());

        }
    }
    public class turret{
        public PIDController controller;

        public double p = .01, i = 0, d = .001;
        public double f = 0;
        public DcMotorEx turret;
        public double limit = 90;
        public double turret_angle;
        public double power;
        public boolean zero = false;
        public boolean shootable = false;
        public void initialize(){
            controller = new PIDController(p, i, d);
            turret = hardwareMap.get(DcMotorEx.class, "turret");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            // Initialize the IMU with this mounting orientation
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
        }
        public void turret_move(){
            double power = 0;
            double current_angle = turret.getCurrentPosition() / cameraCode.ticks_per_degree;
            double turret_angle = (current_angle + (Math.toDegrees(follower.getHeading()) + 39)) + angle_offset;
            if(cameraCode.result.isValid() && !gamepad1.touchpad) {
                if(cameraCode.result.getFiducialResults().get(0).getFiducialId() == 24){
                    turret_angle = (current_angle + (Math.toDegrees(follower.getHeading()) - 39));
                }
                if(abs(turret_angle) < 10){
                    controller.setPID(p, i, d);
                    double pid = controller.calculate((cameraCode.result.getTx()), 0);
                    double ff = Math.cos(Math.toRadians(0)) * f;
                    power = pid + ff;
                    if(abs(cameraCode.result.getTx()) < 6){
                        shootable = true;
                    }else{
                        shootable = false;
                    }
                }else {
                    controller.setPID(p, i, d);
                    double pid = controller.calculate(cameraCode.result.getTx() - turret_angle / -10, 0);
                    double ff = Math.cos(Math.toRadians(0)) * f;
                    power = pid + ff;
                    if(abs(cameraCode.result.getTx() - turret_angle / -10) < 6){
                        shootable = true;
                    }else{
                        shootable = false;
                    }
                }
            }else{
                if(gamepad1.touchpad){
                    zero = true;
                }
                if(zero){
                    controller.setPID(p, i, d);
                    double pid = controller.calculate(current_angle, 0);
                    double ff = Math.cos(Math.toRadians(0)) * f;
                    power = pid + ff;
                    zero = false;
                }else {
                    power = 0;
                }
            }

            turret.setPower(power);

        }
    }



}
