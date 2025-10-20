package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.NaN;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Vector;

public class Launcher{
    double Launch_angle =  Math.toRadians(45);
    Telemetry telemetry;
    double wheel_radius_meters = .036;
    DcMotor leftLaunchMotor , rightLaunchMotor ;
    public Launcher(HardwareMap hardwareMap, Telemetry itelemetry) {
        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
        this.configure_motors(leftLaunchMotor, rightLaunchMotor);
        telemetry = itelemetry;
    }
    void configure_motors(DcMotor leftLaunchMotor, DcMotor rightLaunchMotor) {
        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLaunchMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public double [] FireAtY(float target_pos_y_in, float distance_in) {
        double velocity = distance_to_velocity(distance_in/12, Launch_angle, target_pos_y_in/12,  0);
        double motor_power = velocity_to_motor_power(velocity);
        leftLaunchMotor.setPower(.65);
        rightLaunchMotor.setPower(.65);
        return new double[]{motor_power, velocity};
    }
    public double distance_to_velocity (double x_pos_feet, double theta, double target_pos_feet, double tolerance){
        double target_pos = 0.3048*target_pos_feet ;
        double x_pos = 0.3048*x_pos_feet ;
        double Mag_V = 0;
        double numerator = (7 * x_pos)/Math.cos(theta);
        double denominator = Math.sqrt(10) * Math.sqrt(x_pos*Math.tan(theta)-target_pos-tolerance);
        if (denominator == 0) {
            telemetry.addLine("Division By Zero Error -- Target too far or too close");
            return 0;
        }
        Mag_V = numerator / denominator;
        return Mag_V;
    };

    public void reset() {
        rightLaunchMotor.setPower((0));
        leftLaunchMotor.setPower((0));
    };
    double velocity_to_motor_power (double velocity) {
        return Math.min(0.10857*velocity, 1);
    }
}
