package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Vector;

public class Launcher extends  Subsystem{
    public enum FiringState {
        LOADED(0.0),
        FIRING(1.0);

        public double pos;
        FiringState(double pos) {
            this.pos = pos;
        }
    }
    double Launch_angle =  Math.toRadians(45);
    Telemetry telemetry;
    Servo stopperServo;
    double wheel_radius_meters = .036;
    DcMotor leftLaunchMotor , rightLaunchMotor ;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
        this.configure_motors(leftLaunchMotor, rightLaunchMotor);
        stopperServo = hardwareMap.servo.get("stopperServo");
    }
    void configure_motors(DcMotor leftLaunchMotor, DcMotor rightLaunchMotor) {
        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLaunchMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void FireAtY(double target_pos_y_in, double distance_in) {
        double velocity = distance_to_velocity(distance_in/12, Launch_angle, target_pos_y_in/12,  0);
        double motor_power = velocity_to_motor_power(velocity);
        leftLaunchMotor.setPower(velocity);
        rightLaunchMotor.setPower(velocity);


//            leftLaunchMotor.setPower(0);
//            rightLaunchMotor.setPower(0);
    }
    public void FireAtPower(float power) {
        leftLaunchMotor.setPower(power);
        rightLaunchMotor.setPower(power);
//            leftLaunchMotor.setPower(0);
//            rightLaunchMotor.setPower(0);
    }
    public Action FireAtYAction(double target_pos_y_in, double distance_in) {
        return telemetryPacket -> {
            this.FireAtY(target_pos_y_in, distance_in);
            return false;
        };
    }
     double distance_to_velocity (double x_pos_feet, double theta, double target_pos_feet, double tolerance){
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

    public Action reset() {
        return TelemetryPacket -> {
            rightLaunchMotor.setPower((0));
            leftLaunchMotor.setPower((0));
            return false;
        };
    };
    double velocity_to_motor_power (double velocity) {
        return Math.min(0.10857*velocity, 1);
    }

    public Action setFiringStateAction(FiringState state) {
        return TelemetryPacket -> {
            setFiringState(state);
        return false;};
    }
    public void setFiringState(FiringState state) {
        stopperServo.setPosition(state.pos);
    }
    public void quickFire() {
        stopperServo.setPosition(FiringState.FIRING.pos);
        new SleepAction(.5);
        stopperServo.setPosition(FiringState.LOADED.pos);

    }

}
