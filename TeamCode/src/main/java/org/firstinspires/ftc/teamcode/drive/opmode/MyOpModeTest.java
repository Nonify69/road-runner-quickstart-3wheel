package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous (group = "drive")
public class MyOpModeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-64, -36, 0);
        drive.setPoseEstimate(startPose);

        Vector2d junctionPose = new Vector2d(-12, -24);
        Vector2d conePose = new Vector2d(-12, -60);

        TrajectoryVelocityConstraint splineVelConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(DriveConstants.slowerSplineVel),
                new AngularVelocityConstraint(16)
        ));

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(splineVelConstraint)
                .lineToSplineHeading(new Pose2d(-12, -36, Math.toRadians(-90)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(
                        conePose,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.slowerMoveVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(
                        junctionPose,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.slowerMoveVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(
                        conePose,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.slowerMoveVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);
        drive.followTrajectory(traj2);
        sleep(500);
        for (int i = 0; i < 6; i++) {
            drive.followTrajectory(traj3);
            sleep(1000);
            drive.followTrajectory(traj4);
            sleep(500);
        }
    }
}
