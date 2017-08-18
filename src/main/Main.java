package main;

import java.io.IOException;

import planner.BeliefState;
import planner.Controller;
import planner.Planner;
import commands.ConfigurationCoordinate;
import cv.CameraModel;

/*
 * This project was created by Roger Ballard, almost entirely on 3/6/2017 and 3/7/2017 (With the exception of what got transferred from the first project.)
 */

public class Main {
    public static final double TARGET_HEIGHT = -3150;
    
    public static void main (String[] args) throws IOException {
        double[][] configurationBounds = new double[][] {{-9000, 9000}, {-11750, 11750}, {-13500, 13500}, {-10125, 10125}, {-9000 - ConfigurationCoordinate.HAND_OFFSET, 9000 - ConfigurationCoordinate.HAND_OFFSET}};
        
        boolean superMode = true;
        
        if (superMode) {
            //Set a searching bound that definitely covers everything inside the cage. It looks impressive when the robot finds something all the way out there that it can't reach.
            //This is especially true when you place the stick in the metal of the cage itself.
            BeliefState belief = new BeliefState(new double[150][150], 100, 0, 0);
            new Controller(new Planner(1, 50, 5, 3, configurationBounds, 5, belief, new CameraModel(), TARGET_HEIGHT, false), args[0]).run();
        } else {
            BeliefState belief = new BeliefState(new double[100][100], 100, 0, 0);
            new Controller(new Planner(1, 100, 5, 3, configurationBounds, 5, belief, new CameraModel(), TARGET_HEIGHT, false), args[0]).run();
        }
    }
}
