package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="DecodeTeleop", group="ABC Opmode")
public class DecodeTeleop extends DecodeLibrary {
    public boolean start = false;
    double times = 0;
    boolean back = false;
    public static double amps = 2000;
    @Override
    public void init(){
        teleop = true;
        initialize();
        drive_init();
        follower.setPose(auto_pose);

    }

    @Override
    public void loop(){
        if(start){
            cameraCode.camera_calculations();
            drive();
            button1.button();
            button2.button();
            turret.turret_move();
            sensors.sense();
            shooter.shooting();
            flippy.setPosition(flippy_pos);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            if(gamepad1.left_trigger > .4){
                times = 0;
            }
            if(!back && spindexer.getCurrent(CurrentUnit.MILLIAMPS) > amps){
                times += 1;
                back = true;
            }
            if(spindexer.getCurrent(CurrentUnit.MILLIAMPS) < amps){
                back = false;
            }
            if(times == 5){
                button1.nowbutton.add("l1");
                times += 1;
            }
            telemetry.addData("spindexer_draw", spindexer.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("count", times);
            telemetry.update();
        }else if(!gamepad1.atRest()){
            start = true;
        }

    }
}
