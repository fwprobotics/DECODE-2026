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
    double Launch_Angle;
    DcMotor LaunchMotor ;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        LaunchMotor = hardwareMap.dcMotor.get("Launch");
        this.configure_motors(LaunchMotor);
//        stopperServo = hardwareMap.servo.get("stopperServo");
    }
    void configure_motors(DcMotor LaunchMotor) {
        LaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LaunchMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void FireAtY(double target_pos_y_in, double distance_in) {
        double velocity = distance_to_velocity(distance_in, target_pos_y_in,  0);
        telemetry.addData("VELOCITY", velocity);
        double motor_power = velocity_to_motor_power(velocity);
        LaunchMotor.setPower(motor_power);
        telemetry.addData("MOTOR POWER", motor_power);

    }
    public void FireAtPower(float power) {
        LaunchMotor.setPower(power);
    }
    public Action FireAtYAction(double target_pos_y_in, double distance_in) {
        return telemetryPacket -> {
            this.FireAtY(target_pos_y_in, distance_in);
            return false;
        };
    }

    public Action FireAtPowerAction(double power) {
        return telemetryPacket -> {
            this.FireAtPower((float) power);
            return false;
        };
    }

     double distance_to_velocity (double x_pos_feet, double target_pos_feet, double tolerance){
         // double c = y_0 + (target_pos_feet+tolerance)*0.3048 - Math.tan(Launch_Angle)*(x_pos_feet*0.3048);
         double c = y_0 + target_pos_feet+tolerance - x_pos_feet;

         double v_squared = (this.g * c * Math.pow(x_pos_feet, 2)) / (2*Math.pow(Math.cos(Launch_Angle),2));
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
        return 0.01307*2*velocity;
    }

}
