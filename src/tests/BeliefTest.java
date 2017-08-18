package tests;

import planner.BeliefState;
import visualization.BeliefVisualizer;

import commands.ConfigurationCoordinate;
import commands.WorkspaceCoordinate;

import cv.CameraModel;

public class BeliefTest {
    public static void main (String[] args) {
        double[][] probabilities = new double[100][100];
        double binLength = 100; //1cm bins
        for (int y = 0; y < probabilities.length; y++) {
            for (int x = 0; x < probabilities[y].length; x++) {
                probabilities[y][x] = 1;
            }
        }
        BeliefState belief = new BeliefState(probabilities, binLength, 0, 0);
        belief.normalize();
        
        BeliefVisualizer.drawBeliefState(belief, 5, "beginning");
        
        for (int y = 0; y < probabilities.length; y++) {
            for (int x = 0; x < probabilities[y].length; x++) {
                if (belief.yOffset + binLength * y > 0 && belief.xOffset + binLength * x > 0) {
                    probabilities[y][x] = 0;
                }
            }
        }
        belief.normalize();
        
        BeliefVisualizer.drawBeliefState(belief, 5, "After removing invalid");
        
        ConfigurationCoordinate a = new ConfigurationCoordinate(9000, 0, 9000, 9000, 0);
        belief.updateUnseen(a, new CameraModel().getVisibleRange(a, -3150));
        BeliefVisualizer.drawBeliefState(belief, 5, "After belief update 1");
        
        ConfigurationCoordinate b = new ConfigurationCoordinate(4500, 0, 9000, 9000, 0);
        belief.updateUnseen(b, new CameraModel().getVisibleRange(b, -3150));
        BeliefVisualizer.drawBeliefState(belief, 5, "After belief update 2");
        
        ConfigurationCoordinate c = new ConfigurationCoordinate(13500, 0, 9000, 7000, 0);
        belief.updateUnseen(c, new CameraModel().getVisibleRange(c, -3150));
        BeliefVisualizer.drawBeliefState(belief, 5, "After belief update 3");
        
        ConfigurationCoordinate d = new ConfigurationCoordinate(new WorkspaceCoordinate(-100, -100, -3000, 0, 0));
        belief.updateUnseen(d, new CameraModel().getVisibleRange(d, -3150));
        BeliefVisualizer.drawBeliefState(belief, 5, "After belief update 4");

        ConfigurationCoordinate e = new ConfigurationCoordinate(new WorkspaceCoordinate(-500, -100, -2800, -500, -6000));
        belief.updateUnseen(e, new CameraModel().getVisibleRange(e, -3150));
        BeliefVisualizer.drawBeliefState(belief, 5, "After belief update 5");

        ConfigurationCoordinate f = new ConfigurationCoordinate(new WorkspaceCoordinate(0, -3000, 0, -9000, -4500));
        belief.updateUnseen(f, new CameraModel().getVisibleRange(f, -3150));
        BeliefVisualizer.drawBeliefState(belief, 5, "After belief update 6");
    }
}
