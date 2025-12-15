
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.FieldTrajectoryPlanner;
import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.util.ToggleButton;
// Want to toggle between this and normal driving

public class Drivetrain {
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public LinearOpMode OpMode;
    public Telemetry realTelemetry;
    public IMU imu;
    private boolean inputButtonPressed;
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor.class);

    @Config
    public static class TeleOpDTConstants {
        //Biases so we don't go too fast
        public static double turning_modifier = 0.70;
        //        public static double y_modifier = 0.95;
//        public static double x_modifier = 0.85;
        public static double speedFactor = .8;
        public static double power_modifier = 0.6;
        public static double lift_up_modifier = 0.2;

    };
    public Drivetrain(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){
        OpMode = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);

        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    public void joystickMovement(double leftStickY, double leftStickX, double rightStickX, double rightstickY, boolean slowModeControl, boolean fieldRelativeToggle, boolean boostButton) {
        double frontRightVal;
        double frontLeftVal;
        double backLeftVal;
        double backRightVal;

        double slowModeMult = slowModeControl ? 0.4 : 1;
        double boostModeMult = boostButton ? 1.5 : 1;
        if (fieldRelativeToggle) {
            double LeftStickAngle;

            if (leftStickX == 0 && leftStickY == 0) {
                LeftStickAngle = 0;
            } else {
                LeftStickAngle = Math.atan2(leftStickY, -leftStickX) + Math.PI/2;
            }
            double RobotAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle ;
            realTelemetry.addData("ANGLE", RobotAngle);
            realTelemetry.addData("LEFT STICK ANGLE", LeftStickAngle);

            double NewLeftAngle = LeftStickAngle - RobotAngle + Math.PI/4;
            double magnitude = Math.sqrt(Math.pow(cubeInput(leftStickY, TeleOpDTConstants.speedFactor), 2.0) + Math.pow(cubeInput(leftStickX, TeleOpDTConstants.speedFactor), 2.0));
            double LeftX = Math.cos(NewLeftAngle) * magnitude;
            double LeftY = Math.sin(NewLeftAngle) * magnitude;
            double RightX = rightStickX;
            frontLeftVal = -((-RightX) + LeftX);
            frontRightVal = -((LeftY + RightX));
            backLeftVal = -(((LeftY - RightX)));
            backRightVal = -(((RightX) + LeftX));
      }
    else {


        double LeftY = cubeInput(-leftStickY, TeleOpDTConstants.speedFactor);
        double LeftX = cubeInput(-leftStickX, TeleOpDTConstants.speedFactor);
        double RightX = cubeInput(-rightStickX, TeleOpDTConstants.speedFactor);

        if (LeftY > 0.10 && LeftX < 0.10) {
            LeftX = 0;
        } else if (LeftX > 0.10 && LeftY < 0.10) {
            LeftY = 0;
        }
        frontLeftVal = cubeInput(((LeftY - RightX) - LeftX), TeleOpDTConstants.speedFactor);
        frontRightVal = cubeInput(((LeftY + RightX) + LeftX), TeleOpDTConstants.speedFactor);
        backLeftVal = cubeInput(((LeftY - RightX) + LeftX), TeleOpDTConstants.speedFactor);
        backRightVal = cubeInput(((LeftY + RightX) - LeftX), TeleOpDTConstants.speedFactor);


    }

        frontLeftDrive.setPower(frontLeftVal * slowModeMult * boostModeMult * TeleOpDTConstants.power_modifier);
        frontRightDrive.setPower(frontRightVal * slowModeMult * boostModeMult* TeleOpDTConstants.power_modifier);
        backLeftDrive.setPower(backLeftVal * slowModeMult * boostModeMult * TeleOpDTConstants.power_modifier);
        backRightDrive.setPower(backRightVal * slowModeMult  * boostModeMult  * TeleOpDTConstants.power_modifier);

    };


    double cubeInput (double input, double factor) {
        double cubedComponent = factor * Math.pow(input,3 );
        double linearComponent = input * (1 - factor);
        return cubedComponent + linearComponent;
    };
};
