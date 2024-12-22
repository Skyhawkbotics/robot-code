package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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
                .lineToY(-33)
                        .waitSeconds(2)
                        .lineToY(-40)
                .strafeTo(new Vector2d(50,-40))
                    .waitSeconds(2)
                    .turnTo(Math.toRadians(270))
                    .lineToY(-45)
                    .waitSeconds(2)
                    .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(58,-40))
                    .waitSeconds(2)
                    .turnTo(Math.toRadians(270))
                    .lineToY(-45)
                    .waitSeconds(2)
                    .turnTo(Math.toRadians(90))

                .strafeTo(new Vector2d(40,-25))
                    .turnTo(Math.toRadians(0))
                    .lineToX(58)
                    .waitSeconds(2)
                    .turnTo(Math.toRadians(270))
                    .lineToY(-45)
                    .waitSeconds(2)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(58,-65))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}