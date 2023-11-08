package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setDimensions(15,14)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                                .forward(30)
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {}) // Drop pixel
                                .waitSeconds(0.7)
                                .back(3)
                                .lineToLinearHeading(new Pose2d(50-5,-62+30-3,Math.toRadians(0)))
                                .forward(5)
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {}) // Drop other pixel
                                .waitSeconds(0.7)
                                .splineTo(new Vector2d(2,6),Math.toRadians(0))
                                .build()
                );

        // Set field image
//        Image img = null;
//        try { img = ImageIO.read(new File("C:\\Users\\jevgo\\Documents\\field-2023-juice-dark.png")); }
//        catch (IOException e) {}

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}