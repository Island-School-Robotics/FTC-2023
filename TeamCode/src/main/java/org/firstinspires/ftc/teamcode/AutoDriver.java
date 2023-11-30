

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


@Autonomous(name = "Spike's Auto Drive", group = "LinearOpMode")
public final class AutoDriver extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        DcMotorSimple conveyorBelt = hardwareMap.get(DcMotorSimple.class, "beltMotor");
        conveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);

        int stripeLineToGoTo = 0;
        int conveyorBeltMotion = 1;


        //while(!isStarted()&&!gamepad1.x&&!gamepad1.y&&!gamepad1.a) {
       //     stripeLineToGoTo = gamepad1.x ? 1 : gamepad1.y ? 2 : 3;
       // }
       while(!isStarted()) {
           if(gamepad1.x) {
               stripeLineToGoTo = 1;
                break;
          }
           else if(gamepad1.y) {
               stripeLineToGoTo =2;
               break;
           }
            else if(gamepad1.a) {
                stripeLineToGoTo =3;
                break;
            }
        }
        waitForStart();
        //int conveyorBeltMotion = 1;

       // drive.updatePoseEstimate();
        //conveyorBelt.setPower(0);
        //conveyorBelt.setPower(conveyorBeltMotion);
        //Actions.runBlocking(
        //        drive.actionBuilder(drive.pose)
        //                .strafeTo(new Vector2d(24, 0))
        //                .turnTo(Math.toRadians(180))
         //               .build());
        //drive.updatePoseEstimate();
        //conveyorBelt.setPower(-1);
       // try {
        //    Thread.sleep(500);
        //} catch (InterruptedException e) {
       //    Thread.currentThread().interrupt();
       // }
       // Actions.runBlocking(
         //       drive.actionBuilder(drive.pose)
         //               .strafeTo(new Vector2d(30, 0))
          //              .build());

        //Actions.runBlocking(
           //     drive.actionBuilder(drive.pose)
            //            .strafeTo(new Vector2d(36, 0))
           //             .strafeTo(new Vector2d(28, 48))
           //             .build());
        switch (stripeLineToGoTo) {
            case 1:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(24, 0))
                                .build()
                );
                drive.updatePoseEstimate();
                conveyorBelt.setPower(-1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .turnTo(Math.toRadians(270))
                                .strafeTo(new Vector2d(24, 0))
                                .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(24,0))
                                .turnTo(Math.toRadians(270))
                                .build()
                );
                drive.updatePoseEstimate();
                conveyorBelt.setPower(-1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(24,0))
                                .build()
                );
                drive.updatePoseEstimate();
                break;
            case 3:  Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(24,0))
                            .turnTo(Math.toRadians(90))
                            .build()
            );
                drive.updatePoseEstimate();
                conveyorBelt.setPower(-1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .turnTo(Math.toRadians(180))
                                .strafeTo(new Vector2d(24,0))
                                .build()
                );
                drive.updatePoseEstimate();
                break;
        }
    }
}
