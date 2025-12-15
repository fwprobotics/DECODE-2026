package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TeleopActionRunner;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.StereoCamera;

import java.util.Arrays;
import java.util.Objects;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.subsystems.Wrist;
//import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        TeleopActionRunner actionRunner = new TeleopActionRunner();
        Robot robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDWALL, false);
        byte firing_pattern = 0;
        boolean fieldRelative = true;
        drivetrain.imu.resetYaw();
        while (!gamepad1.dpad_left) {
            if (gamepad1.x) {
                fieldRelative = false;
            }
        }
        waitForStart();
        while (!isStopRequested()) {
            double d_x = 0;
            if (gamepad1.right_trigger>.5) {
                 d_x = robot.stereoCamera.findAngleToAprilTag();
            }
            telemetry.addData("steering", d_x);
            drivetrain.joystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x+d_x, gamepad1.right_stick_y, gamepad1.left_bumper, fieldRelative, gamepad1.right_bumper);
            robot.launcher.FireAtPower(gamepad2.left_trigger);
            if (gamepad2.left_bumper) {
                robot.launcher.setFiringState(Launcher.FiringState.FIRING);
            }
            if (gamepad2.right_bumper) {
                robot.launcher.setFiringState(Launcher.FiringState.LOADED);
            }
            if (gamepad2.a ) {
               robot.launcher.FireAtY(1, 1.83);
            }
            if (gamepad2.b) {
                robot.launcher.FireAtPower(.6F);
            }
            if (gamepad2.y) {
                robot.launcher.FireAtPower(.7F);
            }
            if (gamepad1.y) {
                drivetrain.imu.resetYaw();
            }
            if (gamepad2.x) {
                robot.launcher.FireAtPower(.8F);
            }
            if (gamepad1.x) {
                robot.intake.runIntake();
            }
             if (gamepad1.b) {
                 robot.intake.reset();
            }
             if (gamepad1.a) {
                 robot.intake.reverseIntake();
             }

             if (gamepad1.dpad_left) {
                 telemetry.addData("April Tag: ",robot.leftCamera.find_april_tag());
                 if (robot.leftCamera.find_april_tag() == null) {continue;};
                     double d_0 = robot.stereoCamera.compute_x_distance();
                 telemetry.addData("Fire at time",robot.leftCamera.find_april_tag().id );
                 telemetry.addData("X Distance Stero", d_0);

                 double d_1 = Camera.GetDistanceFromArea(robot.leftCamera.getAreaOfAprilTag());
                 double d_2 = Camera.GetDistanceFromArea(robot.rightCamera.getAreaOfAprilTag());
             }


            telemetry.update();
        }
    }
    public void TurnToAprilTag(Robot robot, Drivetrain drivetrain) {
        double d_x = robot.stereoCamera.findAngleToAprilTag();
        double RightX = d_x;
        double frontLeftVal = RightX;
        double frontRightVal = -RightX;
        double backLeftVal = RightX;
        double backRightVal = -RightX;
        telemetry.addData("d_x", d_x);
        drivetrain.frontLeftDrive.setPower(frontLeftVal);
        drivetrain.frontRightDrive.setPower(frontRightVal);
        drivetrain.backLeftDrive.setPower(backLeftVal );
        drivetrain.backRightDrive.setPower(backRightVal );
    }
}

