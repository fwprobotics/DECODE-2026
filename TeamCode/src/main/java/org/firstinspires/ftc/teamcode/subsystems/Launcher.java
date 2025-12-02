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

public class Launcher extends  Subsystem {
    public enum FiringState {
        LOADED(1),
        FIRING(.25);
        public final double pos;
        FiringState(double pos) {
            this.pos = pos;
        }
    }
    double Launch_angle =  Math.toRadians(45);
    double g = -9.8;
    Telemetry telemetry;
    Servo stopperServo;
    double wheel_radius_meters = .036;
    double y_0 = .2;
    double Launch_Angle;
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
        double velocity = distance_to_velocity(distance_in/12, target_pos_y_in/12,  .5);
        double motor_power = velocity_to_motor_power(velocity);
        leftLaunchMotor.setPower(motor_power);
        rightLaunchMotor.setPower(motor_power);
    }
    public void FireAtPower(float power) {
        leftLaunchMotor.setPower(power);
        rightLaunchMotor.setPower(power);
    }
    public Action FireAtYAction(double target_pos_y_in, double distance_in) {
        return telemetryPacket -> {
            this.FireAtY(target_pos_y_in, distance_in);
            return false;
        };
    }
     double distance_to_velocity (double x_pos_feet, double target_pos_feet, double tolerance){
        double c = y_0 + (target_pos_feet+tolerance)*0.3048 - Math.tan(Launch_Angle)*(x_pos_feet*0.3048);
        double v_squared = (this.g * c * Math.pow(x_pos_feet*0.3048, 2)) / (2*Math.pow(Math.cos(Launch_Angle),2));
        if (v_squared <= 0) {
            return 0;
        }
        double Mag_V = Math.sqrt(v_squared);

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
        return 0.0012922*Math.pow(velocity,2)-0.198256*velocity+8.10605;
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
    public Action quickFireAction() {
        return TelemetryPacket ->{
            this.quickFire();
            return false;
        };
    };

}
