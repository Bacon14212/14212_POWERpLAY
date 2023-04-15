package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16,16)
                .setConstraints(55, 55, 15, 15, 15)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -70, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(-35, -48, Math.toRadians(270)))
                                .splineTo(new Vector2d(-18, -40), Math.toRadians(0))
                                .splineTo(new Vector2d(-4, -28), Math.toRadians(45))
                                .waitSeconds(0.3)

                                .lineToLinearHeading(new Pose2d(-12, -16, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-59, -15, Math.toRadians(180)))
                                .waitSeconds(0.2)


                                .lineToLinearHeading(new Pose2d(-30, -15, Math.toRadians(180)))
                                .splineTo(new Vector2d(-5, -18), Math.toRadians(-45))
                                .waitSeconds(0.3)
                                .lineToSplineHeading(new Pose2d(-12, -17, Math.toRadians(-90)))

                                .build());












/*
                                .lineToSplineHeading(new Pose2d(-35, 0, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-31, -16, Math.toRadians(135)))



                                .waitSeconds(0.3)
                                //Second Cone
                                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, -13, Math.toRadians(180)))

                                .waitSeconds(0.3)

                                .lineToLinearHeading(new Pose2d(-45, -13, Math.toRadians(180)))
                                .splineTo(new Vector2d(-31, -16), Math.toRadians(315))
                                .waitSeconds(0.3)
                                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, -13, Math.toRadians(180)))

                                .waitSeconds(0.3)

                                .lineToLinearHeading(new Pose2d(-45, -13, Math.toRadians(180)))
                                .splineTo(new Vector2d(-31, -16), Math.toRadians(315))
                                .waitSeconds(0.3)

                                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, -13, Math.toRadians(180)))

                                .waitSeconds(0.3)

                                .lineToLinearHeading(new Pose2d(-45, -13, Math.toRadians(180)))
                                .splineTo(new Vector2d(-31, -16), Math.toRadians(315))
                                .waitSeconds(0.3)

                                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, -13, Math.toRadians(180)))

                                .waitSeconds(0.3)

                                .lineToLinearHeading(new Pose2d(-45, -13, Math.toRadians(180)))
                                .splineTo(new Vector2d(-31, -16), Math.toRadians(315))
                                .waitSeconds(0.3)

                                .splineTo(new Vector2d(-45, -13), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, -13, Math.toRadians(180)))

                                .waitSeconds(0.3)

                                .lineToLinearHeading(new Pose2d(-45, -13, Math.toRadians(180)))
                                .splineTo(new Vector2d(-31, -16), Math.toRadians(315))
                                .waitSeconds(0.3)*/








                                /*
                                .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(180)))

                                .splineToConstantHeading(new Vector2d(-61, -13), Math.toRadians(0))
                                .waitSeconds(0.4)
                                .splineTo(new Vector2d(-40, -13), Math.toRadians(0))

                                .splineTo(new Vector2d(-29, -5), Math.toRadians(45))
                                .waitSeconds(0.3)
                                .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(0)))

                                .splineToConstantHeading(new Vector2d(-61, -13), Math.toRadians(0))
                                .waitSeconds(0.4)
                                .splineTo(new Vector2d(-40, -13), Math.toRadians(0))

                                .splineTo(new Vector2d(-29, -5), Math.toRadians(45))
                                .waitSeconds(0.3)
                                .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(0)))

                                .splineToConstantHeading(new Vector2d(-61, -13), Math.toRadians(0))
                                .waitSeconds(0.4)
                                .splineTo(new Vector2d(-40, -13), Math.toRadians(0))

                                .splineTo(new Vector2d(-29, -5), Math.toRadians(45))
                                .waitSeconds(0.3)
                                .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(0)))

                                .splineToConstantHeading(new Vector2d(-61, -13), Math.toRadians(0))
                                .waitSeconds(0.4)
                                .splineTo(new Vector2d(-40, -13), Math.toRadians(0))

                                .splineTo(new Vector2d(-29, -5), Math.toRadians(45))
                                .waitSeconds(0.3)




*/








                                /*
                                .splineTo(new Vector2d(35, -21), Math.toRadians(90))

                                .splineTo(new Vector2d(27, -3), Math.toRadians(315))

                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .turn(Math.toRadians(-135))

                                .lineToSplineHeading(new Pose2d(57, -10, Math.toRadians(0)))

                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .lineToLinearHeading(new Pose2d(27, -3, Math.toRadians(315)))


                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(0)))

                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .lineToLinearHeading(new Pose2d(27.5, -4, Math.toRadians(315)))


                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(0)))

                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .lineToLinearHeading(new Pose2d(28, -4, Math.toRadians(315)))


                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(0)))

                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .lineToLinearHeading(new Pose2d(28, -4, Math.toRadians(315)))

                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(0)))



                                 */



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}