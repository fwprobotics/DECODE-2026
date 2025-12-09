package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@Autonomous
public class BackUpAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.AutoPos autoPos = Robot.AutoPos.BLUEWALL;
        while (!gamepad1.dpad_left) {
            if (gamepad1.dpad_down) {
                autoPos = Robot.AutoPos.REDWALL;
            } else if (gamepad1.dpad_up) {
                autoPos = Robot.AutoPos.BLUEWALL;
            }
            if (gamepad1.dpad_left) {
                autoPos = Robot.AutoPos.REDBASKET;
            } else if (gamepad1.dpad_right) {
                autoPos = Robot.AutoPos.BLUEBASKET;
            }
            telemetry.addData("starting pos", autoPos);
            telemetry.update();
        }
        telemetry.addData("ready for ", autoPos);
        telemetry.update();
        Robot robot = new Robot(hardwareMap, telemetry, autoPos, false);
        Actions.runBlocking(robot.launcher.setFiringStateAction(Launcher.FiringState.LOADED));
        Action autoAction = robot.createTrajectoryPlanner()
//                .stepToShot()
//                .fireWholeMagazine()
                .lineWithBackWall()
                .builder.build();

        waitForStart();

        Actions.runBlocking(autoAction);
    }
}
