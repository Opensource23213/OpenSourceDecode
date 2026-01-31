package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CalcDriveTest", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class CalcDriveTest extends DecodeLibrary {
    public boolean move = false;
    @Override
    public void init() {
        color = 0;
        teleop = false;
        initialize();
        drive_init_auto();
        Path path = new Path(new BezierLine(new Pose(0, 0), new Pose(50, 30)));
        path.setConstantHeadingInterpolation(0);
        follower.followPath(path);
    }


    @Override
    public void loop() {
        follower.update();
        if(follower.getPose().getX() > 40 && !move){
            move = true;
            Path path = new Path(new BezierLine(new Pose(40, 25), new Pose(0, 0)));
            path.setConstantHeadingInterpolation(0);
            follower.followPath(path);

        }else if(follower.getPose().getX() < 10 && move){
            move = false;
            Path path = new Path(new BezierLine(new Pose(10, 5), new Pose(50, 30)));
            path.setConstantHeadingInterpolation(0);
            follower.followPath(path);
        }
        double[] var1 = follower.drivetrain.calculateDrive(follower.getCorrectiveVector(), follower.getHeadingVector(), follower.getDriveVector(), follower.poseTracker.getPose().getHeading());
        double FL = var1[0];
        double FR = var1[1];
        double BL = var1[2];
        double BR = var1[3];
        double fwd = (FL + FR) / 2.0;
        double str = (FL - BL) / 2.0;
        double yaw = (BL - FR) / 2.0;
        telemetry.addData("fwd", fwd);
        telemetry.addData("str", str);
        telemetry.addData("yaw", yaw);
        double L = 8.25;
        double W = 13.375;
        double R = Math.sqrt(L*L + W*W);
        double A = str - yaw * L/R;
        double B = str + yaw * L/R;
        double C = fwd - yaw * W/R;
        double D = fwd + yaw * W/R;
        double frs = Math.sqrt(B*B + C*C);
        double fls = Math.sqrt(B*B + D*D);
        double rrs = Math.sqrt(A*A + C*C);
        double rls = Math.sqrt(A*A + D*D);
        double fra = Math.toDegrees(Math.atan2(B,C)) - 180;
        double fla = Math.toDegrees(Math.atan2(B,D)) - 180;
        double rra = Math.toDegrees(Math.atan2(A,C)) - 180;
        double rla = Math.toDegrees(Math.atan2(A,D)) - 180;
        double max = Math.max(fls, frs);
        max = Math.max(max, rrs);
        max = Math.max(max, rls);
        if(max > 1){
            fls = fls / max;
            frs = frs/max;
            rrs = rrs/max;
            rls = rls/max;
        }
        if(fra < 0){
            fra += 360;
        }
        if(fla < 0){
            fla += 360;
        }
        if(rra < 0){
            rra += 360;
        }
        if(rla < 0){
            rla += 360;
        }
        telemetry.addData("Front Right Speed", frs);
        telemetry.addData("Front Left Speed", fls);
        telemetry.addData("Rear Right Speed", rrs);
        telemetry.addData("Rear Left Speed", rls);
        telemetry.addData("Front Right Angle", fra);
        telemetry.addData("Front Left Angle", fla);
        telemetry.addData("Rear Right Angle", rra);
        telemetry.addData("Rear Left Angle", rla);
        telemetry.update();
    }

}