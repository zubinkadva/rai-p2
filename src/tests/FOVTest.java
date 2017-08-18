package tests;

import geometry.Vector3;
import planner.BeliefState;
import visualization.BeliefVisualizer;

import commands.ConfigurationCoordinate;

import cv.CameraModel;
import cv.VisibleRange;

public class FOVTest {
    public static void main (String[] args) {
        ConfigurationCoordinate configuration = new ConfigurationCoordinate(-9000, 0, 9000, 9000, -4500);
        //ConfigurationCoordinate configuration = new ConfigurationCoordinate(68889, 4352, -222, 9917, 908);
        CameraModel cameraModel = new CameraModel();
        double targetHeight = -3150;
        VisibleRange visibleRange = cameraModel.getVisibleRange(configuration, targetHeight);
        
        BeliefState belief = new BeliefState(new double[100][100], 100, 0, 0);
        for (int y = 0; y < 100; y++) {
            for (int x = 0; x < 100; x++) {
                belief.locationProbabilities[y][x] = 1;
            }
        }
        belief.normalize();
        
        BeliefVisualizer.drawBeliefState(belief, 5, "before");
        
        belief.updateUnseen(configuration, visibleRange);
        
        BeliefVisualizer.drawBeliefState(belief, 5, "after");
        
        System.out.println(Vector3.UP.afterRotation(cameraModel.getCameraOrientation(configuration)));
        System.out.println(configuration.getUnescapedCommandString());
        
        System.out.println();
    }
}
