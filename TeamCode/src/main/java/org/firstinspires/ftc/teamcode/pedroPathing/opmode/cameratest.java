package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Config
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
        shooter.shooting();
        shooter.speed = 6000;
        double power = 0;
        double current_angle = turret.turret.getCurrentPosition() / cameraCode.ticks_per_degree;
        double turret_angle = (current_angle + (imu.getRobotYawPitchRollAngles().getYaw() - 51));
        if(cameraCode.result.isValid() && !gamepad1.touchpad) {
            if(cameraCode.result.getFiducialResults().get(0).getFiducialId() == 24){
                turret_angle = (current_angle + (imu.getRobotYawPitchRollAngles().getYaw() + 51));
            }
            if(abs(turret_angle) < 10){
                power = (cameraCode.result.getTx()) / 40 * -1;
            }else {
                if(cameraCode.result.getFiducialResults().get(0).getFiducialId() == 24){
                    controller.setPID(p, i, d);
                    double pid = controller.calculate(cameraCode.result.getTx() + turret_angle / -20, 0);
                    double ff = Math.cos(Math.toRadians(0)) * f;
                    power = pid + ff;
                }else {
                    controller.setPID(p, i, d);
                    double pid = controller.calculate(cameraCode.result.getTx() - turret_angle / -10, 0);
                    double ff = Math.cos(Math.toRadians(0)) * f;
                    power = pid + ff;
                }
            }
        }else{
            if(gamepad1.touchpad){
                power = current_angle / 90 * -1;
            }else {
                power = 0;
            }
        }

        turret.turret.setPower(power);
    }
}
