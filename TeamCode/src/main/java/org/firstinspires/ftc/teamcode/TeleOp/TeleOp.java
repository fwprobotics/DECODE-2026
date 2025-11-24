package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TeleopActionRunner;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.StereoCamera;

import java.util.Arrays;
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
        waitForStart();
        while (!isStopRequested()) {
            drivetrain.joystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_trigger >= .5, false, gamepad1.right_trigger >= .5);
            robot.launcher.FireAtPower(gamepad2.left_trigger);
            if (gamepad2.left_bumper) {
                robot.launcher.setFiringState(Launcher.FiringState.FIRING);
            }
            if (gamepad2.right_bumper) {
                robot.launcher.setFiringState(Launcher.FiringState.LOADED);
            }
            if (gamepad2.a ) {
               robot.launcher.FireAtY(.65);
            }
            if (gamepad2.b) {
                robot.launcher.FireAtY(.5);
            }
            if (gamepad2.y) {
                robot.launcher.FireAtY(1);
            }
            if (gamepad2.x) {
                robot.launcher.quickFire();
            }
            if (gamepad1.dpad_up) {
                robot.intake.runIntake();
            }
             if (gamepad1.a) {
                 robot.intake.reset();
            }
             if (gamepad1.dpad_down) {
                 robot.intake.reverseIntake();
             }
             if (gamepad1.dpad_right) {
                 robot.leftCamera.find_april_tag();
                 robot.rightCamera.find_april_tag();
             }
             if (gamepad1.dpad_left) {
                 telemetry.addData("X LeftDistance",robot.leftCamera.getAreaOfAprilTag());
                 telemetry.addData("X RightDistance",robot.rightCamera.getAreaOfAprilTag());
                 telemetry.addData("Estimated distance Right",Camera.GetDistanceFromArea(robot.leftCamera.getAreaOfAprilTag()));
                 telemetry.addData("Estimated distance Left",Camera.GetDistanceFromArea(robot.rightCamera.getAreaOfAprilTag()));

             }
            telemetry.update();
        }
    }
}