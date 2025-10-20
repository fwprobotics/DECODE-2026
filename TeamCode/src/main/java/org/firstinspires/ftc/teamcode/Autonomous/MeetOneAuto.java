//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//public class MeetOneAuto {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
////        Robot.AutoPos autoPos = Robot.AutoPos.REDNET;
////        while (!gamepad1.a) {
////            if (gamepad1.dpad_down) {
////                autoPos = Robot.AutoPos.REDNET;
////            } else if (gamepad1.dpad_up) {
////                autoPos = Robot.AutoPos.REDHUMAN;
////            } else if (gamepad1.dpad_left) {
////                autoPos = Robot.AutoPos.BLUENET;
////            } else if (gamepad1.dpad_right) {
////                autoPos = Robot.AutoPos.BLUEHUMAN;
////            }
////            telemetry.addData("starting pos", autoPos);
////            telemetry.update();
////        }
//        Robot robot = new Robot(hardwareMap, telemetry, autoPos, false);
//
//        Action autoAction = robot.createTrajectoryPlanner()
//                .dropSpecimen()
//                .pickNeutral(0)
//                .dropNet()
//                .pickNeutral(1)
//                .dropNet()
//                .pickNeutral(2)
//                .dropNet()
//                .park()
//                //  .ascend()
//                .builder.build();
//
//        Actions.runBlocking(robot.robotAction(Robot.RobotStates.DEFAULT));
//
//        while (!gamepad1.touchpad) {
//            if (gamepad1.y) {
//                robot.claw.setPosition(Claw.ClawStates.OPEN);
//            } else if (gamepad1.b) {
//                robot.claw.setPosition(Claw.ClawStates.CLOSE);
//            }
//        }
//
//        waitForStart();
//
//        Actions.runBlocking(autoAction);
//    }
//}
