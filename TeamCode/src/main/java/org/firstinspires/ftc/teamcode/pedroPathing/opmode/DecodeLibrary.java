package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Config
//@Disabled
public class DecodeLibrary extends OpMode {
    public static double color = 0;
    public double robot_x = 0;
    public double robot_y = 0;
    public double robot_heading = 0;
    public static double speed_far = 1260;
    public boolean manual_turret = false;
    public static double launch_angle = 30;
    public static double spin_speed = 1;
    public boolean first_count = false;
    public DcMotorEx spindexer;
    public static Pose auto_pose = new Pose();
    public static double tele_offset = 0;
    public static double shoot_multiplier = 210;

    public List<Double> balls = new ArrayList<>();
    public static double pattern = 1;
    public static double angle_offset = -90;
    public static double moving_offset = 1.4;
    public static double robot_velocity_stop_shoot = 10;
    public double old_dis = 0;
    public DcMotorEx intake = null;
    public spinny spinny = new spinny();
    public shooter shooter = new shooter();
    public turret turret = new turret();
    public sensors sensors = new sensors();
    public CameraCode cameraCode = new CameraCode();
    public static double adjust = 0.02;
    public double flippy_up = 0.35;
    public static double flippy_hold = .53;
    public double  flippy_down = .71;
    public double flippy_pos = flippy_down;
    public Servo flippy = null;
    public static double sppeed = 0;
    public static double fllap = .04;
    public Servo JamLight;
    public Servo ShootLight;
    public static double shoot_offset = -4;
    public ElapsedTime flip_up = new ElapsedTime();
    public static double flap_minimum = .06;
    public static double shoot_power_offset = 1670;
    public boolean teleop = true;
    public double index_steps = 0;
    public static double flap_switch = -70;
    public static double flap_height = 0;
    public static double shot_time = 200;

    @Override
    public void init(){
        initialize();
    }

    public void initialize() {
        if(color == 0){
            x_mod = 2;
            y_mod = -12;
        }else{
            x_mod = -3;
            y_mod = 3;
        }
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flippy = hardwareMap.get(Servo.class, "flippy");
        JamLight = hardwareMap.get(Servo.class, "JamLight");
        ShootLight = hardwareMap.get(Servo.class, "ShootLight");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cameraCode = new CameraCode();
        cameraCode.init();
        shooter.initialize();
        turret.initialize();
        sensors.initialize();
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
            if(nowbutton.contains("right")){
                if(color == 0) {
                    y_mod -= 1;
                }else{
                    y_mod += 1;
                }
            }else if(nowbutton.contains("left")){
                if(color == 1) {
                    y_mod -= 1;
                }else{
                    y_mod += 1;
                }
            }
            if(nowbutton.contains("up")){
                shoot_power_offset += 10;
            }else if(nowbutton.contains("down")){
                shoot_power_offset -= 10;
            }
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

            if(nowbutton.contains("l1")) {
                balls.clear();
                balls.add(1.0);
                balls.add(2.0);
                balls.add(3.0);
            }
            if(gamepad2.touchpadWasPressed()){
                manual_turret = true;
            }
            if(cameraCode.result.isValid() && 1==0) {
                shooter.speed = shoot_multiplier * ((cameraCode.distance_from_target) - 1.6) + shoot_power_offset;
            }else{
                shooter.speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset;
            }
            if(gamepad1.a){
                intake.setPower(0);
            }
            if(nowbutton.contains("right")){
                if(color == 0) {
                    y_mod -= 1;
                }else{
                    y_mod += 1;
                }
            }else if(nowbutton.contains("left")){
                if(color == 1) {
                    y_mod -= 1;
                }else{
                    y_mod += 1;
                }
            }
            if(nowbutton.contains("up")){
                x_mod += 1;
            }else if(nowbutton.contains("down")){
                x_mod -= 1;
            }
            if(nowbutton.contains("r1")){
                if(spin_speed == 1){
                    spin_speed = .6;
                }else{
                    spin_speed = 1;
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
    public DcMotorEx front_left = null;
    public DcMotorEx rear_left = null;
    public DcMotorEx front_right = null;
    public DcMotorEx rear_right = null;
    public boolean robot_going_forward = true;
    public Follower follower;
    public double currentfr = 0;
    public double currentfl = 0;
    public double currentrr = 0;
    public double currentrl = 0;
    public void drive_init_auto(){
        front_left = hardwareMap.get(DcMotorEx.class, "leftFront");
        front_right = hardwareMap.get(DcMotorEx.class, "rightFront");
        rear_left = hardwareMap.get(DcMotorEx.class, "leftRear");
        rear_right = hardwareMap.get(DcMotorEx.class, "rightRear");
    }
    public void drive_init(){
        front_left = hardwareMap.get(DcMotorEx.class, "leftFront");
        front_right = hardwareMap.get(DcMotorEx.class, "rightFront");
        rear_left = hardwareMap.get(DcMotorEx.class, "leftRear");
        rear_right = hardwareMap.get(DcMotorEx.class, "rightRear");
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void current_check(){
        if(front_right.getCurrent(CurrentUnit.AMPS)> currentfr){
            currentfr = front_right.getCurrent(CurrentUnit.AMPS);
        }
        if(front_left.getCurrent(CurrentUnit.AMPS)> currentfl){
            currentfl = front_left.getCurrent(CurrentUnit.AMPS);
        }
        if(rear_right.getCurrent(CurrentUnit.AMPS)> currentrr){
            currentrr = rear_right.getCurrent(CurrentUnit.AMPS);
        }
        if(rear_left.getCurrent(CurrentUnit.AMPS)> currentrl){
            currentrl = rear_left.getCurrent(CurrentUnit.AMPS);
        }
    }
    public void drive(){
        follower.update();
        Pose poseEstimate = follower.getPose();
        double angle = poseEstimate.getHeading() + Math.toRadians(angle_offset);
        double axial = 0;
        double lateral = 0;
        double yaw = 0; // set control to the sticks
        axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;


        if (gamepad1.ps) {
            telemetry.addData("Yaw", "Resetting\n");
            if(color == 0){
                y_mod = -12;
                x_mod = 2;
                follower.setPose(new Pose(0, -77, 0));
            }else{
                y_mod = 3;
                x_mod = -3;
                follower.setPose(new Pose(0, 77, 0));
            }

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
        if(gamepad1.left_trigger < .4 || sensors.last_time()) {
            front_left.setPower(leftFrontPower);
            front_right.setPower(rightFrontPower);
            rear_left.setPower(leftBackPower);
            rear_right.setPower(rightBackPower);
        }else{
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
        }


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
                //robot_auto_on = ((robot_y >= robot_x && robot_y >= -robot_x && robot_y >= 0) || (abs(robot_y) * -1 <= robot_x - .8 && abs(robot_y) * -1 <= -robot_x - .8 && abs(robot_y) * -1 <= -.8)) && abs(follower.getVelocity().getMagnitude()) < robot_velocity_stop_shoot && gamepad1.right_trigger < .4 && turret.shootable;
            }else{
                robot_auto_on = false;
            }
        }
    }
    public class shooter{
        public Servo flap;
        public DcMotorEx shoot1;
        public DcMotorEx shoot2;
        public double speed = 0;
        public double position = 0.04;
        public double speed_difference = 0;
        public double max_draw1 = 0;
        public double max_draw2 = 0;
        public double calc_velocity = 0;
        public double calc_rpm = 0;
        public double flap_velocity = 0;
        public double old_velocity = 0;
        public double last_shot = 850;
        public boolean far_shooting = false;
        public void initialize(){
            flap = hardwareMap.get(Servo.class, "flap");
            shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
            shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
            PIDFCoefficients pidf = new PIDFCoefficients(50, 20, 0, 1);
            shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void shooting() {
            if (robot_x < 48 && teleop) {
                /*speed = speed_far;
                position = .6;
                far_shooting = true;*/
                speed = last_shot;
                position = flap_height;
            } else if (teleop) {
                far_shooting = false;
                if (dead_distance * .0254 < moving_offset) {
                    position = .1;
                    speed += flap_switch;
                } else {
                    position = flap_height;
                }
            }
            shoot2.setVelocity(speed);
            shoot1.setVelocity(speed);
            flap.setPosition(position);
        }
    }
    public class spinny{
        public double position = 1;
        public double turns = 0;
        public double spinny_at = 0;
        public PIDController controller;

        public double p = 0.00031, i = 0, d = 0.00002;

        public double f = 0;
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
        public boolean switchwaspressed = false;
        public boolean going_forward = true;
        public ElapsedTime flipoffset;
        public void initialize(){
            controller = new PIDController(p, i, d);
            flipoffset = new ElapsedTime();
            spinny = hardwareMap.get(CRServo.class, "spinny");
            spinny2 = hardwareMap.get(CRServo.class, "spinny2");
            spinnypos = hardwareMap.get(AnalogInput.class, "spinnypos");
            time.reset();
        }
        public void spin(){
            if(teleop) {
                double temp = position;
                if (!robot_going_forward) {
                    temp -= 1;
                    if (temp < 1) {
                        temp = 3;
                    }
                }
                if (balls.contains(temp) && balls.size() < 3) {
                    position -= 1;
                    if (position < 1) {
                        position = 3;
                    }
                }
            }
            if(balls.size() == 3){
                JamLight.setPosition(0);
            }else{
                JamLight.setPosition(1);
            }

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
                if(going_forward) {
                    going_forward = false;
                    position -= 1;
                    if (position < 1){
                        position = 3;
                    }
                }
                target = (position + .5) * 1200 + 400;
            }else if(flippy_pos == flippy_down){
                if(!going_forward) {
                    going_forward = true;
                }
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
                if(position != 0) {
                    position -= 1;
                    if (position < 1) {
                        position = 3;
                    }
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
            /*if(!intakeswitch1.getState() || !intakeswitch2.getState()){
                switchwaspressed = true;
            }*/if(gamepad1.right_trigger > .4 && gamepad1.left_trigger > .4){
                power = 1;
            }else if(position == 0 || (gamepad1.left_trigger > .4 && flippy_pos == flippy_up)){
                balls.clear();
                if((teleop && flip_up.milliseconds() > 100) || (flip_up.milliseconds() > 200)) {
                    if(power == -1){
                        power = -.999;
                    }else {
                        power = -1;
                    }
                }
                //intake.setPower(1);

            }else if(position == 4){
                power = 0;
            }
            spinny.setPower(-power);
            spinny2.setPower(-power);

        }
    }
    public class turret{
        public PIDController controller;

        public double p = .01, i = 0, d = .001;
        public double f = 0;
        public DcMotorEx turret;
        public double current_angle;
        public double limit = 160;
        public double turret_angle;
        public double power;
        public boolean zero = false;
        public boolean shootable = false;
        public double turret_target = 0;
        public double manual_angle = 0;
        public Servo turret_servo_1;
        public Servo turret_servo_2;
        public AnalogInput turret_servo_pos;
        public double turret_pos = 0;
        public double analog_offset = 0;
        public double servo_degrees = 90/.29;
        public double max = 25;
        public double multiplier = 1.2;
        public double a_slow = 1.2;
        public double angle_mod = .9;
        public double target_angle = .5;
        public void initialize(){
            turret_servo_1 = hardwareMap.get(Servo.class, "turret_servo_1");
            turret_servo_2 = hardwareMap.get(Servo.class, "turret_servo_2");
            turret_servo_1.setDirection(Servo.Direction.REVERSE);
            turret_servo_2.setDirection(Servo.Direction.REVERSE);
            //turret_servo_pos = hardwareMap.get(AnalogInput.class, "turret_servo_pos");
        }
        public void turret_move(){

            dead_wheel_calculations();
            if(zero){
                target_angle = manual_angle / servo_degrees + .5;
            }else{
                double angle = Math.toDegrees(follower.getHeading());
                angle = dead_angle + angle;
                target_angle = (angle) / servo_degrees + .5;
            }
            zero = false;
            if((target_angle - .5) * servo_degrees > limit){
                target_angle = (limit - 2) / servo_degrees + .5;
            }else if((target_angle - .5) * servo_degrees < -limit + 60){
                target_angle = (-limit + 60) / servo_degrees + .5;
            }
            if(manual_turret){
                target_angle = .5;
            }
            turret_servo_1.setPosition(target_angle - (4 / servo_degrees) + .0075);
            turret_servo_2.setPosition(target_angle - (4 / servo_degrees) - .0075);


        }
    }
    public double dead_angle = 0;
    public double dead_distance = 0;
    public static double y_mod = -12;
    public static double x_mod = 2;
    public static double speed_shoot = 40;
    public void dead_wheel_calculations(){
        Pose location = new Pose();
        if(color == 0){
            location = new Pose((130 + x_mod) - robot_x, (52.2 + y_mod) - robot_y, robot_heading);

        }else{
             location = new Pose((130 + x_mod) - robot_x, (52.2 + y_mod) + robot_y, robot_heading);

        }
        telemetry.update();
        if(location.getX() <= 0){
            dead_angle = 90;
        }else{
            dead_angle = Math.toDegrees(Math.atan(location.getY() / location.getX()));
        }
        if(color == 0){
            dead_angle *= -1;
        }
        dead_distance = Math.sqrt(Math.pow(location.getX(), 2) + Math.pow(location.getY(), 2));
    }
    public static double spinpower = -.7;
    public class sensors{
        public DigitalChannel apin0;
        public DigitalChannel apin2;
        public boolean auto_intake = false;
        public ElapsedTime intake_offset = new ElapsedTime();
        public ElapsedTime shoot_time_offset = new ElapsedTime();
        public RevColorSensorV3 colorfront1;
        public RevColorSensorV3 colorfront2;
        public RevColorSensorV3 colorback1;
        public double ball_counted = 0;
        public double sorts = 0;
        public RevColorSensorV3 colorback2;
        public DigitalChannel countfront;
        public DigitalChannel countback;
        public double front1 = 0;
        public double front2 = 0;
        public double back1 = 0;
        public double back2 = 0;
        public boolean sorted = false;
        public double sort_step = 0;
        public ElapsedTime sort_time = new ElapsedTime();
        public boolean middleisgreen = false;
        public boolean spin = false;
        public double sort_places = 0;
        public void initialize(){
            apin0 = hardwareMap.get(DigitalChannel.class, "apin1");
            apin2 = hardwareMap.get(DigitalChannel.class, "apin3");
            colorfront1 = hardwareMap.get(RevColorSensorV3.class, "colorfront1");
            colorfront2 = hardwareMap.get(RevColorSensorV3.class, "colorfront2");
            colorback1 = hardwareMap.get(RevColorSensorV3.class, "colorback1");
            colorback2 = hardwareMap.get(RevColorSensorV3.class, "colorback2");
            countfront = hardwareMap.get(DigitalChannel.class, "flapsensefront");
            countback = hardwareMap.get(DigitalChannel.class, "flapsenseback");
            balls.clear();
        }
        public void sense(){
            /*if(balls.size() >= 3){
                JamLight.setPosition(1);
            }else{
                JamLight.setPosition(0);
            }*/
            count();
            front1 = colorfront1.getDistance(DistanceUnit.MM);
            front2 = colorfront2.getDistance(DistanceUnit.MM);
            back1 = colorback1.getDistance(DistanceUnit.MM);
            back2 = colorback2.getDistance(DistanceUnit.MM);
            if((robot_x < 35 && balls.isEmpty()) || apin0.getState() || apin2.getState() || countfront.getState() || countback.getState() || ((colorfront() || colorback()) && balls.size() < 3)) {
                if (balls.size() >= 3) {
                    if(apin0.getState() || apin2.getState()|| countfront.getState() || countback.getState()) {
                        if(!gamepad1.a && teleop && gamepad1.left_trigger < .4){
                            intake.setPower(-.5);
                        }
                    }
                } else {
                    if(!gamepad1.a && gamepad1.left_trigger < .4) {
                        intake.setPower(1);
                    }
                    if (apin0.getState()) {
                        robot_going_forward = true;
                    } else if(apin2.getState()){
                        robot_going_forward = false;
                    }
                }
            }else{
                if(sorted || balls.size() < 3){
                    intake.setPower(gamepad1.right_trigger);
                }

            }
            if(gamepad1.left_trigger > .4 && follower.getVelocity().getMagnitude() < speed_shoot){
                balls.clear();
                shooter.last_shot = shooter.speed;
                flippy_pos = flippy_up;
                intake.setPower(0);
                spindexer.setPower(spin_speed);
            }else if(balls.size() >= 3){
                sort();
            }else{
                sorted = false;
                sort_step = 0;
                moving_steps = 0;
                if(robot_going_forward){
                    if(colorfront() && balls.size() < 3){
                        spindexer.setPower(spinpower);
                    }else{
                        spindexer.setPower(-gamepad1.right_trigger);
                    }
                }else{
                    if(colorback() && balls.size() < 3){
                        spindexer.setPower(spinpower);
                    }else{
                        spindexer.setPower(-gamepad1.right_trigger);
                    }
                }
                flippy_pos = flippy_down;
            }

        }
        public void count(){
            if(!first_count && balls.size() > 0){
                balls.clear();
                first_count = true;
            }
            if(countfront.getState() || countback.getState()){
                ball_counted = 0;
            }else if(!countfront.getState() && !countback.getState() && ball_counted == 0){
                ball_counted = 1;
                intake_offset.reset();
            } else if(ball_counted == 1 && intake_offset.milliseconds() > 25){
                ball_counted = 2;
                balls.add(1.0);
            }

        }
        public void sort(){
            if(!sorted){
                if(sort_step == 0){
                    gamepad1.rumble(1000);
                    sort_time.reset();
                    if(!gamepad1.a && teleop && gamepad1.left_trigger < .4) {
                        intake.setPower(-.5);
                    }
                    flippy_pos = flippy_hold;
                    sort_step = 1;
                }else if(sort_step == 1 && sort_time.milliseconds() >= 500){
                    if(firstisgreen() || lastisgreen()){
                        middleisgreen = false;
                    }else{
                        middleisgreen = true;
                    }
                    if(pattern == 1){
                        pattern1sort();
                    }else if(pattern == 2){
                        pattern2sort();
                    }else{
                        pattern3sort();
                    }
                    sort_step = 2;
                }else if(sort_step == 2){
                    if(sort_places == 0 || sorts == 2){
                        sort_step = 0;
                        sorts = 0;
                        sorted = true;
                        if(teleop) {
                            intake.setPower(0);
                        }
                        sort_time.reset();
                    }else if(sort_places == 1){
                        sort_once();
                    }else if(sort_places == 2){
                        sort_twice();
                    }
                }else if(sort_step == 3 && sort_time.milliseconds() >= 500){
                    sort_step = 1;
                    sorts += 1;
                    moving_steps = 0;
                }

            }else if(balls.size() >= 3){
                spindexer.setPower(0);
                flippy_pos = flippy_hold;
            }else if(balls.size() < 3){
                sorted = false;
                sort_step = 0;
                moving_steps = 0;
                sorts = 0;
            }
        }
        public double moving_steps = 0;
        public void sort_once(){
            if(moving_steps == 0){
                sort_time.reset();
                flippy_pos = flippy_down;
                spindexer.setPower(0);
                moving_steps = .5;
            }if(moving_steps == .5 && sort_time.milliseconds()> 100){
                moving_steps = 1;
                spindexer.setPower(-1);
                sort_time.reset();
            }
            if(moving_steps == 1 && sort_time.milliseconds() > 100){
                flippy_pos = flippy_hold;
                sort_time.reset();
                sort_step = 3;
            }
        }
        public void sort_twice(){
            if(moving_steps == 0){
                sort_time.reset();
                flippy_pos = flippy_down;
                spindexer.setPower(0);
                moving_steps = .5;
            }if(moving_steps == .5 && sort_time.milliseconds()> 100){
                moving_steps = 1;
                spindexer.setPower(-1);
                sort_time.reset();
            }if(moving_steps == 1 && sort_time.milliseconds() > 260){
                flippy_pos = flippy_hold;
                moving_steps = 2;
                sort_time.reset();
                sort_step = 3;
            }
        }
        public void pattern1sort(){
            if(firstisgreen()){
                sort_places = 0;
            }else if(middleisgreen){
                sort_places = 2;
            }else if(lastisgreen()){
                sort_places = 1;
            }
        }
        public void pattern2sort(){
            if(middleisgreen){
                sort_places = 0;
            }else if(lastisgreen()){
                sort_places = 2;
            }else if(firstisgreen()){
                sort_places = 1;
            }
        }
        public void pattern3sort(){
            if(lastisgreen()){
                sort_places = 0;
            }else if(firstisgreen()){
                sort_places = 2;
            }else if(middleisgreen){
                sort_places = 1;
            }
        }

        public void auto_index(){

        }
        public boolean firstisgreen(){
            if(front1 < front2){
                if(colorfront1.getNormalizedColors().green > colorfront1.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }else{
                if(colorfront2.getNormalizedColors().green > colorfront2.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }
        }
        public boolean sensed = false;
        public boolean last_time(){
            if(gamepad1.left_trigger < .4 || colorfront()){
                sensed = false;
            }
            if(!colorfront() && !sensed){
                sensed = true;
                shoot_time_offset.reset();
            }
            if(sensed && shoot_time_offset.milliseconds() > shot_time){
                return true;
            }else{
                return false;
            }

        }
        public boolean lastisgreen(){
            if(back1 < back2){
                if(colorback1.getNormalizedColors().green > colorback1.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }else{
                if(colorback2.getNormalizedColors().green > colorback2.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }
        }

        public boolean colorfront(){
            if(front2 < 140 || front1 < 140){
                return true;
            }else{
                return false;
            }

        }
        public boolean colorback(){
            if(back2 < 140 || back1 < 140){
                return true;
            }else{
                return false;
            }

        }
    }





}
