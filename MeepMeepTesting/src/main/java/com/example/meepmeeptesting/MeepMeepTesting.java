package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.core.colorscheme.ColorManager;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.entity.*;
import com.noahbres.meepmeep.core.ui.WindowFrame;
import com.noahbres.meepmeep.core.util.FieldUtil;
import com.noahbres.meepmeep.core.util.LoopManager;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.ui.TrajectoryProgressSliderMaster;
import java.awt.*;
import java.awt.datatransfer.StringSelection;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.border.EtchedBorder;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900); // window size, and contuine this for the rest of the format of the window

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -70, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, -33), Rotation2d.fromDouble(Math.toRadians(90)))
                //first place
                .waitSeconds(1.25)
                .lineToY(-44) //back up

                .strafeTo(new Vector2d(34, -44)) //don't run into sub
                .strafeTo(new Vector2d(34, -5)) //don't run into sample!
                //first sample push
                .strafeTo(new Vector2d(45,-5))
                .waitSeconds(.1)
                .lineToY(-50)
                .lineToY(-15)
                //second sample push
                .strafeTo(new Vector2d(54,-12))
                .waitSeconds(.1)
                .lineToY(-50)
                .lineToY(-15)
                //third sample push
                .strafeTo(new Vector2d(62,-12))
                .waitSeconds(.1)
                .lineToY(-50)
                .waitSeconds(.1)
                .setTangent(Math.toRadians(180)) //rotation positioning
                .lineToXSplineHeading(36,Math.toRadians(270)) //line up for pickup
                .setTangent(Math.toRadians(90))
                .lineToY(-62)
                //pick up first
                .strafeToLinearHeading(new Vector2d(10,-32), Rotation2d.fromDouble(Math.toRadians(90)))
                //place first
                .waitSeconds(1.25)
                .strafeToLinearHeading(new Vector2d(36,-52), Rotation2d.fromDouble(Math.toRadians(270))) //line up for pickup
                .waitSeconds(.1)
                //pick up second
                .lineToY(-62)
                .strafeToLinearHeading(new Vector2d(6,-32), Rotation2d.fromDouble(Math.toRadians(90)))
                //place second
                .waitSeconds(1.25)
                .strafeTo(new Vector2d(50,-62)) //parking

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}