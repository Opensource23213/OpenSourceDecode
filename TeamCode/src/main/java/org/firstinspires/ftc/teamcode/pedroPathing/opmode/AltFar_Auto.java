package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="AltFar_Auto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class AltFar_Auto extends DecodeLibrary{
    public double forward = 0.25;
    public Path pick_after_stuff;
    public Path wall_scootch;
    public Path wall_scootch2;
    public Path shoot_from_wall;
    public Path first_pick;
    public Path first_shoot;
    public Path second_shoot;
    public Path third_shoot;
    public Path fourth_shoot;
    public Path second_pick;
    public Path third_pick;
    public Path park;
    public Path gate;
    public double shot = 0;
    public double old_turns = 0;
    public double steps = 0;
    public ElapsedTime gate_open = new ElapsedTime();
    public ElapsedTime shooting_time = new ElapsedTime();
    public sorting sorting = new sorting();
    public double sort_pos = 0;
    public double old_pattern = 2;
    public double old_color = 1;
    public boolean shooting = false;
    public double index_steps = 0;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        initialize();
        drive_init_auto();
        cameraCode.limelight.pipelineSwitch(6);
        if(color == 0) {
            blue_init();
            angle_offset = -90;
        }else{
            red_init();
            angle_offset = 90;
        }
        tele_offset = turret.current_angle;
        pattern = 2;
        follower.setMaxPower(1);
        sensors.sorted = true;
    }
    @Override
    public void init_loop(){
        follower.update();
        turret.turret_servo_1.setPosition(.5 - (4 / turret.servo_degrees));
        turret.turret_servo_2.setPosition(.5 - (4 / turret.servo_degrees));
        tele_offset = turret.current_angle;
        cameraCode.camera_calculations();
        if(cameraCode.result.isValid()) {
            if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 23) {
                pattern = 3;
            } else if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 21) {
                pattern = 1;
            }else{
                pattern = 2;
            }
            if (cameraCode.result.getTx() < 0) {
                color = 1;
            } else {
                color = 0;
            }
        }
        if(old_color != color) {
            color = 1;
            teleop = false;
            initialize();
            drive_init_auto();
            cameraCode.limelight.pipelineSwitch(6);
            if (old_color == 1) {
                blue_init();
                angle_offset = -90;
                old_color = 0;
                color = 0;
            } else {
                red_init();
                angle_offset = 90;
                old_color = 1;
            }
        }
        auto_pose = follower.getPose();
        telemetry.addData("color", color);
        telemetry.addData("pattern", pattern);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        auto_pose = follower.getPose();
        cameraCode.camera_calculations();
        if(cameraCode.result.getPipelineIndex() == 6){
            if (color == 1) {
                cameraCode.limelight.pipelineSwitch(5);
            } else {
                cameraCode.limelight.pipelineSwitch(4);
            }
        }
        flippy.setPosition(flippy_pos);
        sensors.auto_index();
        shooter.shooting();
        turret.turret_move();
        sorting.sort();
        shoot();
        if(shooter.speed > 1000){
            shooter.position = .6;
        }else{
            shooter.position = .4;
        }
        if(forward == 6.5){
            if(follower.getPose().getX() < 45){
                follower.setMaxPower(.55);
            }
        }
        if(forward == 7){
            if(abs(follower.getPose().getY()) < 30){
                sorting.sort_balls = true;
                intake.setPower(-1);
                forward = 7.1;
            }
        }
        if(forward == .25){
            if(shooter.shoot1.getVelocity() >= 1100) {
                forward = .3;
            }
        }else if(forward == .3){
            shoot();
            shooting = true;
            if(steps == 2){
                shooting = false;
                flippy_pos = .4;
                robot_going_forward = true;
                spindexer.setPower(-1);
                follower.followPath(third_pick);
                intake.setPower(1);
                follower.setMaxPower(1);
                forward = .45;
                steps = 0;
            }
        }
        else if((follower.atParametricEnd() || !follower.isBusy())) {
            if (forward == .45) {
                intake.setPower(1);
                spindexer.setPower(-1);
                follower.setMaxPowerScaling(1);
                follower.followPath(fourth_shoot);
                follower.setMaxPower(1);
                forward = .5;
            } else if (forward == .5) {
                shoot();
                shooting = true;
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    flippy_pos = flippy_hold;
                    robot_going_forward = true;
                    spindexer.setPower(-1);
                    follower.followPath(first_pick);
                    intake.setPower(1);
                    follower.setMaxPower(1);
                    forward = .75;
                    steps = 0;
                }
            }else if (forward == .75) {
                follower.setMaxPowerScaling(1);
                follower.followPath(gate);
                follower.setMaxPower(1);
                forward = 1;
            }else if(forward == 1){
                if(steps == 0){
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > 1000) {
                    robot_going_forward = true;
                    sorting.sort_balls = true;
                    follower.followPath(second_shoot);
                    follower.setMaxPower(1);
                    forward = 2;
                    steps = 0;
                }
            }else if(forward == 2){
                shoot();
                intake.setPower(0);
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    flippy_pos = flippy_hold;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.setMaxPower(.5);
                    follower.followPath(second_pick);
                    forward = 3;
                    steps = 0;
                }
            }else if(forward == 3){
                sorting.sort_balls = true;
                follower.setMaxPowerScaling(1);
                follower.followPath(third_shoot);
                follower.setMaxPower(1);
                forward = 4;
            }
            else if(forward == 4){
                intake.setPower(0);
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    robot_going_forward = true;
                    flippy_pos = flippy_hold;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(pick_after_stuff);
                    follower.setMaxPower(1);
                    forward = 6.5;
                    steps = 0;
                }
            }else if(forward == 6.5){
                follower.followPath(wall_scootch);
                follower.setMaxPower(1);
                forward = 7;
            }else if(forward == 7.1){
                intake.setPower(0);
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(1);
                    robot_going_forward = true;
                    intake.setPower(1);
                    flippy_pos = flippy_hold;
                    follower.followPath(park);
                    forward = 8;
                    steps = 0;
                }
            }
        }
    }

    public void red_init(){
        first_shoot = new Path(new BezierCurve(new Pose(0,0), new Pose(0,-4), new Pose(72, -5)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-10));
        first_pick = new Path(new BezierCurve(new Pose(65,0), new Pose(50, -3), new Pose(48, -37.5)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(),new Pose(50, -30), new Pose(53, -30), new Pose(53, -41)));
        gate.setConstantHeadingInterpolation(0);
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, -15), new Pose(72, -5)));
        second_shoot.setLinearHeadingInterpolation(0,Math.toRadians(-45));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(74, -36)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, -5)));
        third_shoot.setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(-45));
        third_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(23, -5), new Pose(23, -36)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        fourth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), new Pose(0, 0)));
        fourth_shoot.setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(0));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(40,-30)));
        park.setConstantHeadingInterpolation(Math.toRadians(0));
    }
    public void blue_init(){
        first_pick = new Path(new BezierCurve(new Pose(0,0), new Pose(55, 0), new Pose(48, 40)));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(54, 40)));
        gate.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, 15), new Pose(72, 4)));
        second_shoot.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(90));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(76, 37)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, 4)));
        third_shoot.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135));
        third_pick = new Path(new BezierCurve(new Pose(0,0), new Pose(28, 0), new Pose(23, 38)));
        fourth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), new Pose(0, 0)));
        fourth_shoot.setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(0));
        pick_after_stuff = new Path(new BezierCurve(new Pose(72, 4), new Pose(61,45), new Pose(21, 47), new Pose(6, 47)));
        pick_after_stuff.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(170));
        wall_scootch = new Path(new BezierCurve(new Pose(6, 47), new Pose(18,30), new Pose(8,6)));
        wall_scootch.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(20));
        park = new Path(new BezierLine(new Pose(8,6), new Pose(8,20)));
        park.setConstantHeadingInterpolation(Math.toRadians(20));
    }
    public void shoot(){
        if(shooting){
            if(forward < .75 || forward >= 6.5) {
                if(forward > .3){
                    shooter.speed = 1120;
                }else {
                    shooter.speed = 1100;
                }
            }else{
                shooter.speed = 780;
            }
            if(steps == 0 && (sensors.sorted || forward < .75)){
                if(shooter.speed != 1100 || shooter.speed != 1120 || (shooter.shoot1.getVelocity() >= shooter.speed - 10 && abs(follower.getVelocity().getMagnitude()) < 3)) {
                    shooting_time.reset();
                    steps = .5;
                }
            }else if(steps == .5 && shooting_time.milliseconds() > 500){
                spindexer.setPower(1);
                shooting_time.reset();
                flippy_pos = flippy_up;
                steps = 1;
            }else if(steps == 1 && shooting_time.milliseconds() > 1150){
                steps = 2;
                sensors.sorted = false;
            }
        }else{
            if(forward < .75 || forward >= 6.5) {
                if(forward > .3){
                    if(forward >= 6.5 && abs(follower.getPose().getY()) > 20){
                        turret.zero = true;
                        if (color == 0) {
                            turret.manual_angle = -2;
                        } else {
                            turret.manual_angle = 2;
                        }
                    }
                    shooter.speed = 1120;
                }else {
                    shooter.speed = 1100;
                }
            }else{
                turret.zero = true;
                if(forward <= 2) {
                    if (color == 0) {
                        turret.manual_angle = 35;
                    } else {
                        turret.manual_angle = -35;
                    }
                }else{
                    if (color == 0) {
                        turret.manual_angle = 90;
                    } else {
                        turret.manual_angle = -90;
                    }
                }

                shooter.speed = 780;
            }
        }
    }
    public class sorting{
        public boolean sort_balls = false;
        public void sort(){
            if(sort_balls){
                sensors.sort();
                if(sensors.sorted){
                    sort_balls = false;
                }
            }
        }
    }


}
