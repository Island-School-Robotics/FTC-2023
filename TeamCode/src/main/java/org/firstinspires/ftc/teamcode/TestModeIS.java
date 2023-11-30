

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name="TestModeIS", group = "Linear OpMode")
public final class TestModeIS extends LinearOpMode {
    public static double DISTANCE = 64;

    private final double LAUNCH_NOGO = 0.80;
    private final double LAUNCH_GO = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo launchServo = hardwareMap.servo.get("launch");


        launchServo.setPosition(LAUNCH_NOGO);
        DcMotorSimple conveyorBelt = hardwareMap.get(DcMotorSimple.class, "beltMotor");
        conveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
        double movementX, movementY, angle;


        waitForStart();
        int conveyorBeltMotion = 0;

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            conveyorBeltMotion = (gamepad1.right_bumper) ? -1 : (gamepad1.left_bumper) ? 1 : (gamepad1.dpad_down) ? 0: conveyorBeltMotion;
            conveyorBelt.setPower(conveyorBeltMotion);

            launchServo.setPosition(gamepad1.x?LAUNCH_GO:LAUNCH_NOGO);

            movementX = -1*gamepad1.left_stick_y;
            movementY = -1*gamepad1.left_stick_x;
            angle = -1*gamepad1.right_stick_x;

            Vector2d innerVector = new Vector2d(movementX, movementY);

            innerVector = (innerVector.angleCast().times(drive.pose.heading.inverse())).vec();
                    //innerVector.angleCast().minus(drive.pose.heading);
            // makes robot spin in a circle
            drive.setDrivePowers(new PoseVelocity2d(innerVector, angle));




        }
    }
}
