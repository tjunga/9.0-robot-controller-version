package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.drive.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class RedRoadrunnerLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(this);
        Drivetrain drivetrain = new Drivetrain(this);
        drivetrain.initDrivetrain(hardwareMap);
        mech.initMechanisms(hardwareMap);
        mech.initEncoders();

        Pose2d startPose = new Pose2d(-34,-60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        double pixelLocation = 270;


        TrajectorySequence purplepixel = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(5)
                .back(1)
                .lineToLinearHeading(new Pose2d(-34, -7, Math.toRadians(pixelLocation)))
                .build();
        TrajectorySequence yellowPixel = drive.trajectorySequenceBuilder(purplepixel.end())
                .lineToLinearHeading(new Pose2d(47, -38, Math.toRadians(0)))
                .build();
        TrajectorySequence whitePixel = drive.trajectorySequenceBuilder(yellowPixel.end())
                .back(85)
                .splineToLinearHeading(new Pose2d(43, -35, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(whitePixel.end())
                .strafeRight(24)
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(park1.end())
                .forward(10)
                .build();


        waitForStart();
        mech.closeClaws();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(purplepixel);
        mech.purplePosition();
        drive.followTrajectorySequence(yellowPixel);
        mech.autoscoringPosition();


    }
}