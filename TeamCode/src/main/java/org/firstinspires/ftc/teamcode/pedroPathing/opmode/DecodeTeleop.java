package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="DecodeTeleop", group="ABC Opmode")
public class DecodeTeleop extends DecodeLibrary {
    public boolean start = false;
    double times = 0;
    boolean back = false;
    public static double amps = 2000;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry telemetry2 = null;


    @Override
    public void init(){
        teleop = true;
        initialize();
        drive_init();
        follower.setPose(auto_pose);
        telemetry2 = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop(){

        /*telemetry2.addData("front_left", front_left.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry2.addData("front_right", front_right.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry2.addData("rear_left", rear_left.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry2.addData("rear_right", rear_right.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry2.addData("shoot1", shooter.shoot1.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry2.addData("shoot2", shooter.shoot2.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry2.addData("spindexer", spindexer.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry2.addData("Intake", intake.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry2.update();*/
        if(start){
            //cameraCode.camera_calculations();
            drive();
            button1.button();
            button2.button();
            turret.turret_move();
            sensors.sense();
            shooter.shooting();
            flippy.setPosition(flippy_pos);
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
            telemetry.addData("distance", dead_distance * .0254);
            telemetry.update();
        }else if(!gamepad1.atRest()){
            start = true;
        }

    }
}
