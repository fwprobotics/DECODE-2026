
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
//import org.firstinspires.ftc.teamcode.util.ToggleButton;
// Want to toggle between this and normal driving

public class Drivetrain {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    public LinearOpMode OpMode;
    public Telemetry realTelemetry;
    public IMU imu;

    private boolean inputButtonPressed;
//    public ToggleButton fieldRelativeDrive;


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

    }


    public Drivetrain(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        OpMode = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);
//        fieldRelativeDrive = new ToggleButton(false);

        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));


        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //  OpMode.idle();
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    //This is the teleop drive formulas
    public void joystickMovement(double leftStickY, double leftStickX, double rightStickX, double rightstickY, boolean slowModeControl, boolean fieldRelativeToggle, boolean boostButton){

//        double RightStickAngle;

        //toggles between field relative drive(aka forward is your forward(default) vs forward is robot forward)
//        fieldRelativeDrive.toggle(fieldRelativeToggle);
        double frontRightVal;
        double frontLeftVal;
        double backLeftVal;
        double backRightVal;

        double slowModeMult = slowModeControl ? 0.4 : 1;
        double boostModeMult = boostButton ? 1.5 : 1;

            double LeftY = cubeInput(-leftStickY, TeleOpDTConstants.speedFactor);
            double LeftX = cubeInput(-leftStickX, TeleOpDTConstants.speedFactor);
            double RightX = cubeInput(-rightStickX, TeleOpDTConstants.speedFactor);

            if (LeftY > 0.10 && LeftX <0.10) {
                LeftX = 0;
            } else if (LeftX > 0.10 && LeftY < 0.10) {
                LeftY = 0;
            }
            //     if (slowModeControl) {
            frontLeftVal = ((LeftY - RightX) - LeftX);
            frontRightVal = ((LeftY + RightX) + LeftX);
            backLeftVal = ((LeftY - RightX) + LeftX);
            backRightVal = ((LeftY + RightX) - LeftX);
//            } else {
//                frontLeftVal = cubeInput(((LeftY - RightX) - LeftX), TeleOpDTConstants.speedFactor);
//                frontRightVal = cubeInput(((LeftY + RightX) + LeftX), TeleOpDTConstants.speedFactor);
//                backLeftVal = cubeInput(((LeftY - RightX) + LeftX), TeleOpDTConstants.speedFactor);
//                backRightVal = cubeInput(((LeftY + RightX) - LeftX), TeleOpDTConstants.speedFactor);
//            }
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -leftStickY,
//                            -leftStickX,
//                            -rightStickX
//                    )
//            );
//            drive.update();

            //   realTelemetry.addData("LeftX", LeftX);
            //  realTelemetry.addData("RightX", RightX);
            // realTelemetry.addData("LeftY", LeftY);


        frontLeftDrive.setPower(frontLeftVal * slowModeMult * boostModeMult * TeleOpDTConstants.power_modifier);
        frontRightDrive.setPower(frontRightVal * slowModeMult * boostModeMult* TeleOpDTConstants.power_modifier);
        backLeftDrive.setPower(backLeftVal * slowModeMult * boostModeMult * TeleOpDTConstants.power_modifier);
        backRightDrive.setPower(backRightVal * slowModeMult  * boostModeMult  * TeleOpDTConstants.power_modifier);

        realTelemetry.addData("Front Left", frontLeftDrive.getCurrentPosition());
        realTelemetry.addData("Back Left", backLeftDrive.getCurrentPosition());
        realTelemetry.addData("Front Right", frontRightDrive.getCurrentPosition());
        realTelemetry.addData("Back Right", backRightDrive.getCurrentPosition());
//        realTelemetry.addData("Toggle Field Relative", fieldRelativeDrive.state());
        realTelemetry.addData("Slow Mode Multiplier", slowModeMult);


    }

    double cubeInput (double input, double factor) {
        double cubedComponent = factor * Math.pow(input,3 );
        double linearComponent = input * (1 - factor);
        return cubedComponent + linearComponent;
    }

}
