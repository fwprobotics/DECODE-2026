package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Vector;

public class Launcher extends  Subsystem {

    double Launch_angle =  Math.toRadians(45);
    double g = -9.8;
    double wheel_radius_meters = .036;
    double y_0 = .2;
    double MAX_FIRING_RANGE = 100000;
    double Launch_Angle;
    DcMotor LaunchMotor ;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        LaunchMotor = hardwareMap.dcMotor.get("Launch");
        this.configure_motors(LaunchMotor);
    }
    void configure_motors(DcMotor LaunchMotor) {
        LaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LaunchMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void FireAtY(double target_pos_y, double distance) {
        double velocity = distance_to_velocity(distance, target_pos_y,  0);
        double motor_power = velocity_to_motor_power(velocity);
        LaunchMotor.setPower(motor_power);

    }
    public void FireAtPower(float power) {
        LaunchMotor.setPower(power);
    }
    public Action FireAtYAction(double target_pos_y, double distance) {
        return telemetryPacket -> {
            this.FireAtY(in_to_m(target_pos_y), in_to_m(distance));
            return false;
        };
    }

    public Action FireAtPowerAction(double power) {
        return telemetryPacket -> {
            this.FireAtPower((float) power);
            return false;
        };
    }

     double distance_to_velocity (double x_pos, double target_pos, double tolerance){
         double c = y_0 + target_pos+tolerance - x_pos;
         double v_squared = (this.g * c * Math.pow(x_pos, 2)) / (2*Math.pow(Math.cos(Launch_Angle),2));
         telemetry.addData("V_SQUARED", v_squared);
         if (v_squared <= 0) {
            return 0;
        }
        double Mag_V = Math.sqrt(v_squared);

        return Mag_V;
    };
    public Action reset() {
        return TelemetryPacket -> {
            LaunchMotor.setPower((0));
            return false;
        };
    };
    double velocity_to_motor_power (double velocity) {
        return velocity/MAX_FIRING_RANGE;
    }

    public double in_to_m(double quantity) {
    return quantity*0.0254;
    }
}
