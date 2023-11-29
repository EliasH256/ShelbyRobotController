package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.time.LocalDateTime;

import static com.example.meepmeeptesting.Field.StartPos.*;
import static com.example.meepmeeptesting.Field.Highways.*;


public class MeepMeepTesting
{
     private static final float X_WIDTH = 141.0f;
     private static final float Y_WIDTH = 141.0f;
     /* package */ static final float N_WALL_Y = Y_WIDTH/2.0f;
     /* package */ static final float E_WALL_X = X_WIDTH/2.0f;
     /* package */ static final float S_WALL_Y = -N_WALL_Y;
     /* package */ static final float W_WALL_X = -E_WALL_X;

     protected static final float tileWidth        = 23.5f;
     protected static final float oneTile          = tileWidth;
     protected static final float halfTile         = tileWidth/2.0f;
     protected static final float oneAndHalfTile   = 3.0f*tileWidth/2.0f;
     protected static final float halfField        = X_WIDTH/2.0f;
     protected static final float quadField        = X_WIDTH/4.0f;
     protected static final float mmTargetHeight   = 6.0f;

     public static void main(String[] args)
     {
         // TODO: If you experience poor performance, enable this flag
         // System.setProperty("sun.java2d.opengl", "true");

         // Declare a MeepMeep instance
         // With a field size of 800 pixels

         RobotConstants.init(RobotConstants.Chassis.B7252);
         Field.Highways[] highwaysIn = {CENTER, WALL,WALL, DOOR,WALL, CENTER,WALL, DOOR};
         Field.Highways[] pixelsIn = {DOOR, CENTER,WALL, DOOR,WALL, DOOR,WALL, DOOR};
         CenterStageRoute ffr = new CenterStageRoute(START_BACKDROP, WALL, Field.Alliance.RED, Field.Route.REPEAT_4PT_JUNCTION,Field.FirstLocation.PIXEL_DOOR, Route.TeamElement.RIGHT, highwaysIn, pixelsIn ) ;
         TrajectorySequence seq = ffr.fullSeq;

         MeepMeep meepMeep = new MeepMeep(600);

         DefaultBotBuilder myBot = new DefaultBotBuilder(meepMeep)
                 .setColorScheme(new ColorSchemeRedDark())
                 .setDimensions(RobotConstants.BOT_WID, RobotConstants.BOT_LEN)
                 .setConstraints(50, 50,Math.toRadians(180), Math.toRadians(180),12);

         RoadRunnerBotEntity rrBotEntity = myBot.build();
         rrBotEntity.followTrajectorySequence(seq);

//       PPlayRoute ffr2 = new PPlayRoute(START_RIGHT, CENTER_PARK, Field.Alliance.BLUE);
//       TrajectorySequence seq2 = ffr2.fullSeq;				 

//         DefaultBotBuilder myBot2 = new DefaultBotBuilder(meepMeep)
//                 .setColorScheme(new ColorSchemeRedDark())
//                 .setDimensions(18, 20)
//                 .setConstraints(50, 50,Math.toRadians(180), Math.toRadians(180),12);


//         RoadRunnerBotEntity rrBotEntity2 = myBot2.build();
//         rrBotEntity2.followTrajectorySequence(seq2);

         System.out.println("STARTING at " +LocalDateTime.now());

         meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                 .setDarkMode(true)
                 .setBackgroundAlpha(0.95f)
                 .addEntity(rrBotEntity)
                 //s.addEntity(rrBotEntity2)
                 .start(); /* Executes the sequences */
       }
}