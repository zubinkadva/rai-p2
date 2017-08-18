package tests;

import main.Main;
import planner.BeliefState;
import planner.Controller;
import planner.Planner;
import visualization.BeliefVisualizer;

import commands.ConfigurationCoordinate;

import cv.CameraModel;

public class BeliefInitializationTest {
    public static void main (String[] args) {
        double[][] configurationBounds = new double[][] {{-9000, 9000}, {-11750, 11750}, {-13500, 13500}, {-10125, 10125}, {-9000 - ConfigurationCoordinate.HAND_OFFSET, 9000 - ConfigurationCoordinate.HAND_OFFSET}};
        BeliefState belief = new BeliefState(new double[100][100], 100, 0, 0);
        Planner planner = new Planner(1, 100, 5, 3, configurationBounds, 5, belief, new CameraModel(), Main.TARGET_HEIGHT, false);
        
        Controller.initializeFirst(planner.beliefState);
        BeliefVisualizer.drawBeliefState(planner.beliefState, 5, "FindFirst Initial Belief; score = " + planner.beliefState.getScore());
        Controller.initializeSecond(planner.beliefState, new double[] {0, 0});
        BeliefVisualizer.drawBeliefState(planner.beliefState, 5, "FindSecond Initial Belief; score = " + planner.beliefState.getScore());
        Controller.initializeThird(planner.beliefState, new double[] {0, 0}, new double[] {-Controller.INTERPOINT_DISTANCE, 0});
        BeliefVisualizer.drawBeliefState(planner.beliefState, 5, "FindThird Initial Belief; score = " + planner.beliefState.getScore());
    }
}
