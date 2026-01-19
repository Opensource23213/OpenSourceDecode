package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@Config
@TeleOp(name="SwerveTest", group="ABC Opmode")
public class SwerveTest extends DecodeLibrary {
    public static double fr_offset = 1050;
    public static double fl_offset = -1350;
    public static double rr_offset = -900;
    public static double rl_offset = -160;
    public static double p = .001, i = 0, d = .00005;

    public static double f = 0;
    public swerve_drive chassis = new swerve_drive();

    public IMU imu = null;
    @Override
    public void init(){
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        chassis.initialize();
    }
    @Override
    public void loop() {
        chassis.drive();
    }
    public class swerve_drive{
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
        double rls  = 0;
        double fra = 0;
        double fla = 0;
        double rra = 0;
        double rla = 0;
        swerve_pod fr = new swerve_pod();
        swerve_pod fl = new swerve_pod();
        swerve_pod rr = new swerve_pod();
        swerve_pod rl = new swerve_pod();
        public void initialize(){
            fr.initialize("fr_motor", "fr_servo", "fr_analog");
            fl.initialize("fl_motor", "fl_servo", "fl_analog");
            rr.initialize("rr_motor", "rr_servo", "rr_analog");
            rl.initialize("rl_motor", "rl_servo", "rl_analog");
        }
        public class swerve_pod{

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
            public void initialize(String wheel_name, String servo_name, String analog_name){
                controller = new PIDController(p, i, d);
                wheel = hardwareMap.get(DcMotorEx.class, wheel_name);
                wheel.setDirection(DcMotorSimple.Direction.REVERSE);
                pod1servo = hardwareMap.get(CRServo.class, servo_name);
                pod1rotation = hardwareMap.get(AnalogInput.class, analog_name);
                time.reset();
            }
            public void move(double offset, double inversion){
                wheel_pos = (abs(inversion-pod1rotation.getVoltage() / 3.3) * 3600) + offset;
                if(wheel_pos < 0){
                    wheel_pos += 3600;
                }if(wheel_pos > 3600){
                    wheel_pos -= 3600;
                }
                target = position * 10;
                if(abs(target - wheel_pos) > 900 && abs(target - wheel_pos) < 2700){
                    target -= 1800;
                    wheel_power *= -1;
                }
                if(target < 0){
                    target += 3600;
                }
                if(target - wheel_pos > 1800){
                    target -= 3600;
                }else if(target - wheel_pos < -1800){
                    target += 3600;
                }
                controller.setPID(p, i, d);
                double pid = controller.calculate(wheel_pos, target);
                double ff = Math.cos(Math.toRadians(target)) * f;
                power = pid + ff;
                if(inversion == 1) {
                    pod1servo.setPower(-power);
                }else{
                    pod1servo.setPower(power);
                }
                wheel.setPower(wheel_power);
            }
        }
        public void drive(){
            if(abs(gamepad1.left_stick_x) > .1 || abs(gamepad1.left_stick_y) > .1 || abs(gamepad1.right_stick_x) > .1){
                L = 8.25;
                W = 13.375;
                R = Math.sqrt(L*L + W*W);
                double angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double fwd = gamepad1.left_stick_y * Math.cos(angle) + (-gamepad1.left_stick_x) * Math.sin(angle);
                double str = (-gamepad1.left_stick_x) * Math.cos(angle) - gamepad1.left_stick_y * Math.sin(angle);
                A = str - (-gamepad1.right_stick_x) * L/R;
                B = str + (-gamepad1.right_stick_x) * L/R;
                C = fwd - (-gamepad1.right_stick_x) * W/R;
                D = fwd + (-gamepad1.right_stick_x) * W/R;
                frs = Math.sqrt(B*B + C*C);
                fls = Math.sqrt(B*B + D*D);
                rrs = Math.sqrt(A*A + C*C);
                rls = Math.sqrt(A*A + D*D);
                fra = Math.toDegrees(Math.atan2(B,C)) - 180;
                fla = Math.toDegrees(Math.atan2(B,D)) - 180;
                rra = Math.toDegrees(Math.atan2(A,C)) - 180;
                rla = Math.toDegrees(Math.atan2(A,D)) - 180;
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
                fr.position = fra;
                fr.wheel_power = frs;
                fl.position = fla;
                fl.wheel_power = fls;
                rr.position = rra;
                rr.wheel_power = rrs;
                rl.position = rla;
                rl.wheel_power = rls;
            }else{
                frs = 0;
                fls = 0;
                rrs = 0;
                rls = 0;
                fr.wheel_power = frs;
                fl.wheel_power = fls;
                rr.wheel_power = rrs;
                rl.wheel_power = rls;
            }
            fr.move(fr_offset, 1);
            fl.move(fl_offset, 1);
            rr.move(rr_offset, 1);
            rl.move(rl_offset, 1);

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



}



