package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="Hive_Auto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class HiveAuto extends DecodeLibrary{
    public double forward = 0.25;
    public Path wall_pick;
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
        follower.followPath(first_shoot);
        pattern = 2;
        follower.setMaxPower(1);
        sensors.sorted = true;
    }
    @Override
    public void init_loop(){
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
            initialize();
            cameraCode.limelight.pipelineSwitch(6);
            if (color == 0) {
                blue_init();
                angle_offset = -90;
                old_color = 0;
            } else {
                red_init();
                angle_offset = 90;
                old_color = 1;
            }
            follower.followPath(first_shoot);
        }
        auto_pose = follower.getPose();
        telemetry.addData("color", color);
        telemetry.addData("pattern", pattern);
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
        if(forward == .25){
            turret.zero = true;
            if(color == 1) {
                turret.manual_angle = 30;
            }else{
                turret.manual_angle = -30;
            }
        }else{
            turret.zero = true;
            if(color == 0) {
                turret.manual_angle = 6;
            }else{
                turret.manual_angle = -6;
            }
        }
        turret.turret_move();
        sorting.sort();
        shoot();
        if((forward == .5 || forward == 3 || forward == 1.9)){
            if(forward == 3 && (follower.getPose().getY() < -8 || color == 0) && (follower.getPose().getY() > 8  || color == 1)){
                follower.setMaxPower(.35);
            }else{
                if(follower.getPose().getX() > 20 && forward == 1.9) {
                    sorting.sort_balls = true;
                    forward = 2;
                }
            }

        }else if(forward == 5 && follower.getPose().getX() < 40){
            follower.setMaxPower(.5);
        }
        if(follower.atParametricEnd() || !follower.isBusy()) {
            if (forward == .25) {
                shoot();
                shooting = true;
                if(steps == 2){
                    shooting = false;
                    flippy_pos = flippy_hold;
                    robot_going_forward = true;
                    spindexer.setPower(-1);
                    follower.followPath(wall_pick);
                    intake.setPower(1);
                    follower.setMaxPower(1);
                    forward = .3;
                    steps = 0;
                }
            }else if (forward == .3) {
                flippy_pos = flippy_hold;
                robot_going_forward = true;
                spindexer.setPower(-1);
                follower.followPath(wall_scootch);
                intake.setPower(1);
                follower.setMaxPower(.7);
                forward = .4;
                steps = 0;
            }else if (forward == .4) {
                flippy_pos = flippy_hold;
                robot_going_forward = true;
                spindexer.setPower(-1);
                follower.followPath(wall_scootch2);
                intake.setPower(1);
                follower.setMaxPower(.5);
                forward = .5;
                steps = 0;
            }else if (forward == .5) {
                intake.setPower(1);
                follower.setMaxPowerScaling(1);
                follower.followPath(shoot_from_wall);
                follower.setMaxPower(1);
                forward = 1.9;
            }else if(forward == 2){
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    flippy_pos = flippy_hold;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.setMaxPower(.7);
                    follower.followPath(first_pick);
                    forward = 3;
                    steps = 0;
                }
            }else if(forward == 3){
                sorting.sort_balls = true;
                follower.setMaxPowerScaling(1);
                follower.followPath(second_shoot);
                follower.setMaxPower(1);
                forward = 4;
            }else if(forward == 4){
                shoot();
                shooting = true;
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    flippy_pos = flippy_hold;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(third_pick);
                    follower.setMaxPower(1);
                    forward = 5;
                    steps = 0;
                }
            }else if(forward == 5){
                sorting.sort_balls = true;
                follower.followPath(fourth_shoot);
                follower.setMaxPower(1);
                forward = 6;
            }
            else if(forward == 6){
                intake.setPower(0);
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    flippy_pos = flippy_down;
                    follower.followPath(park);
                    follower.setMaxPower(1);
                    forward = 7;
                    steps = 0;
                }
            }
        }
    }

    public void red_init(){
        first_shoot = new Path(new BezierCurve(new Pose(0,0), new Pose(0,-4), new Pose(72, -5)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-10));
        wall_pick = new Path(new BezierCurve(new Pose(72,-5), new Pose(50, 0), new Pose(20, 0), new Pose(10, 0), new Pose(5, -37.5), new Pose(1, -46)));
        wall_pick.setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(-100));
        wall_scootch = new Path(new BezierCurve(new Pose(1,-46), new Pose(-2, -42), new Pose(3, -46)));
        wall_scootch.setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(-80));
        wall_scootch2 = new Path(new BezierCurve(new Pose(3,-46), new Pose(2, -42), new Pose(1, -46)));
        wall_scootch2.setLinearHeadingInterpolation(Math.toRadians(-120), Math.toRadians(-80));
        shoot_from_wall = wall_pick.getReversed();
        shoot_from_wall.setConstantHeadingInterpolation(Math.toRadians(-45));
        first_pick = new Path(new BezierCurve(new Pose(65,0), new Pose(50, -3), new Pose(48, -36.5)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(),new Pose(50, -30), new Pose(53, -30), new Pose(53, -41)));
        gate.setConstantHeadingInterpolation(0);
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, -15), new Pose(72, -5)));
        second_shoot.setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(-45));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(74, -36)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, -5)));
        third_shoot.setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(-45));
        third_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(21, -5), new Pose(22, -36)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        fourth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), new Pose(72, -5)));
        fourth_shoot.setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(-45));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(40,-30)));
        park.setConstantHeadingInterpolation(Math.toRadians(0));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierCurve(new Pose(0,0), new Pose(0,-4), new Pose(72, -5)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(10));
        wall_pick = new Path(new BezierCurve(new Pose(72,5), new Pose(50, 0), new Pose(20, 0), new Pose(10, 0), new Pose(5, 37.5), new Pose(1, 46)));
        wall_pick.setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(100));
        wall_scootch = new Path(new BezierCurve(new Pose(1,46), new Pose(2, 42), new Pose(3, 46)));
        wall_scootch.setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(80));
        wall_scootch2 = new Path(new BezierCurve(new Pose(3,46), new Pose(2, 42), new Pose(1, 46)));
        wall_scootch2.setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(80));
        shoot_from_wall = wall_pick.getReversed();
        shoot_from_wall.setConstantHeadingInterpolation(Math.toRadians(45));
        first_pick = new Path(new BezierCurve(new Pose(65,0), new Pose(50, 3), new Pose(48, 36.5)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(),new Pose(50, 30), new Pose(53, 30), new Pose(53, 41)));
        gate.setConstantHeadingInterpolation(0);
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, 15), new Pose(72, 5)));
        second_shoot.setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(45));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(74, 36)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, 5)));
        third_shoot.setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(45));
        third_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(21, 5), new Pose(22, 36)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        fourth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), new Pose(72, 5)));
        fourth_shoot.setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(45));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(40,30)));
        park.setConstantHeadingInterpolation(Math.toRadians(0));
    }
    public void shoot(){
        if(shooting){
            shooter.speed = 760;
            if(steps == 0 && shooter.speed_difference < 20 && sensors.sorted){
                shooting_time.reset();
                steps = 1;
                spindexer.setPower(1);
                flippy_pos = flippy_up;
            }else if(steps == 1 && shooting_time.milliseconds() > 2000){
                steps = 2;
                sensors.sorted = false;
            }
        }else{
            shooter.speed = 760;
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
