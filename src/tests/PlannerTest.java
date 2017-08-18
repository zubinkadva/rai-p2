package tests;

import java.util.Random;

import planner.BeliefState;
import planner.Planner;
import visualization.BeliefVisualizer;

import commands.ConfigurationCoordinate;

import cv.CameraModel;
import cv.VisibleRange;

public class PlannerTest {
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
                double yPos = belief.yOffset + binLength * y;
                double xPos = belief.xOffset + binLength * x;
                if ((yPos > 500 && xPos > 500) ||
                        (yPos < 750 && yPos > -750 && xPos < 750 && xPos > -750)) {
                    probabilities[y][x] = 0;
                }
            }
        }
        belief.normalize();
        
        BeliefVisualizer.drawBeliefState(belief, 5, "after removing invalid");
        
        CameraModel cameraModel = new CameraModel();
        double[][] configurationBounds = new double[][] {{-9000, 9000}, {-11750, 11750}, {-13500, 13500}, {-10125, 10125}, {-9000 - ConfigurationCoordinate.HAND_OFFSET, 9000 - ConfigurationCoordinate.HAND_OFFSET}};

        //Planner planner = new Planner(1, 5, 6, 3, configurationBounds, 5, belief, cameraModel, -3150, false);
        //Planner planner = new Planner(1, 100, 6, 3, configurationBounds, 5, belief, cameraModel, -3150, false);
        //Planner planner = new Planner(1, 50, 5, 3, configurationBounds, 5, belief, cameraModel, -3150, false);
        //Planner planner = new Planner(1, 20, 5, 3, configurationBounds, 10, belief, cameraModel, -3150, false);
        Planner planner = new Planner(1, 100, 5, 3, configurationBounds, 5, belief, cameraModel, -3150, false);
        
        double[] targetPosition = belief.samplePosition();
        
        Random rng = new Random();
        
        for (int x = 1; x <= 10; x++) {
            ConfigurationCoordinate nextStep = planner.getNextStep();
            double uncertainty = cameraModel.getUncertainty(nextStep, targetPosition, -3150);
            double[] observedPosition = targetPosition.clone();
            for (int i = 0; i < observedPosition.length; i++) observedPosition[i] += rng.nextGaussian() * uncertainty / 2; //A rough approximation, but good enough.
            System.out.println(nextStep.getUnescapedCommandString());
            VisibleRange visibleRange = cameraModel.getVisibleRange(nextStep, -3150);
            //belief.updateUnseen(cameraModel.getVisibleRange(nextStep, -3150));
            if (visibleRange.contains(targetPosition)) {
                belief.updateBelief(targetPosition, nextStep, cameraModel, -3150);
            } else {
                belief.updateUnseen(nextStep, cameraModel.getVisibleRange(nextStep, -3150));
            }
            System.out.println(belief.getScore());
            BeliefVisualizer.drawBeliefState(belief, 5, "after belief update " + x);
        }
    }
}
