package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
@Disabled
@TeleOp(name="CameraTest", group="ABC Opmode")

public class cameratest extends DecodeLibrary{
    public PIDController controller;

    public static double p = 0.001, i = 0, d = 0;

    public static double f = 0;

    public static double power_mod = 35;
    public static double angle_mod = 12;
    @Override
    public void init(){
        initialize();
        controller = new PIDController(p, i, d);
        turret.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }
    @Override
    public void loop(){
        cameraCode.camera_calculations();
        double turret_angle = turret.turret.getCurrentPosition() / cameraCode.ticks_per_degree + (imu.getRobotYawPitchRollAngles().getYaw() - 51);

        double power = (cameraCode.result.getTx() - turret_angle/angle_mod) / power_mod * -1;
        if(power > .3){
            power = .3;
        }else if(power < -.3) {
            power = -.3;
        }
        if(abs(turret.turret.getCurrentPosition() / cameraCode.ticks_per_degree) > 45){
            if(turret.turret.getCurrentPosition() < 0){
                power = .1;
            }else{
                power = -.1;
            }
        }
        turret.turret.setPower(power);
        telemetry.addData("imu",imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("turret_angle", turret.turret.getCurrentPosition() / cameraCode.ticks_per_degree);
        telemetry.addData("real turret_angle", turret_angle );
        telemetry.update();
    }
}
