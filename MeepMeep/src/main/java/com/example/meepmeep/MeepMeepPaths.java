package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;

import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepPaths {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity redBackdrop = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 17.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.95)
                .build();
        RoadRunnerBotEntity blueBackdrop = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 17.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.95)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        redBackdrop.runAction(redBackdrop.getDrive().actionBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                //.splineToSplineHeading(new Pose2d(5, -35, Math.toRadians(140)),Math.toRadians(140))
                //center
                .splineToSplineHeading(new Pose2d(17, -27, Math.toRadians(140)),Math.toRadians(140))
                //right
                //.splineTo(new Vector2d(29, -35),Math.toRadians(140))
                .setReversed(true)
                .splineTo(new Vector2d(38, -36), Math.toRadians(0))
                .splineTo(new Vector2d(58, -61), Math.toRadians(0))
                .build());
        blueBackdrop.runAction(blueBackdrop.getDrive().actionBuilder(new Pose2d(14, 61, Math.toRadians(-90)))
                //left
                //.splineToSplineHeading(new Pose2d(5, 35, Math.toRadians(-140)),Math.toRadians(-140))
                //center
                //.splineToSplineHeading(new Pose2d(17, 27, Math.toRadians(-140)),Math.toRadians(-140))
                //right
                .splineTo(new Vector2d(29, 35),Math.toRadians(-140))
                .setReversed(true)
                .splineTo(new Vector2d(38, 36), Math.toRadians(0))
                .splineTo(new Vector2d(58, 61), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBackdrop)
                .addEntity(blueBackdrop)
                .start();
    }
}