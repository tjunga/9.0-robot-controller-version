package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(-34,60, Math.toRadians(90)))
                                .back(50)
                                .waitSeconds(3)
                                .turn(Math.toRadians(90))
                                .forward(18)
                                .waitSeconds(2)
                                .back(85)
                                .splineToLinearHeading(new Pose2d(47, 38, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(3)
                                .strafeLeft(20)
                                .forward(10)
                                .build()
                );
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(-34,-60, Math.toRadians(270)))
                                .back(1)
                                .splineToLinearHeading(new Pose2d(-34, -10, Math.toRadians(270)), Math.toRadians(0))
                                .waitSeconds(3)
                                .turn(Math.toRadians(-90))
                                .forward(18)
                                .waitSeconds(2)
                                .back(95)
                                .splineToLinearHeading(new Pose2d(47, -38, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(3)
                                .strafeRight(20)
                                .forward(10)
                                .build()
                );
        RoadRunnerBotEntity myThirdBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(10,-60, Math.toRadians(270)))
                                .back(1)
                                .lineToLinearHeading(new Pose2d(10, -10, Math.toRadians(270)))
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(47, -38, Math.toRadians(0)))
                                .waitSeconds(2)
                                .back(50)
                                .lineToLinearHeading(new Pose2d(-55, -28, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47, -38, Math.toRadians(0)))
                                .build()
                );

        RoadRunnerBotEntity auto = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(-34,-60, Math.toRadians(270)))
                                .back(1)
                                .lineToLinearHeading(new Pose2d(-34, -40, Math.toRadians(90)))
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(-55, -28, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47, -38, Math.toRadians(0)))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-55, -28, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47, -38, Math.toRadians(0)))
                                .waitSeconds(2)
                                .build()
                );
        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(auto)
                .addEntity(mySecondBot)
                .start();
    }
}
/* blue far nut auto
    .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(-34,60, Math.toRadians(90)))
                                 .back(50)
                                .waitSeconds(4)
                                .turn(Math.toRadians(90))
                                .forward(18)
                                .waitSeconds(2)
                                .back(85)
                                .splineToSplineHeading(new Pose2d(47, 38, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(5)
                                .strafeLeft(20)
                                .back(10)
                                .build()
 */
