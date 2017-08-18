package planner;

import static util.Utilities.downloadImage;
import static util.Utilities.sendCommand;
import geometry.Vector2;

import java.io.IOException;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import visualization.SegmentVisualizer;
import visualization.BeliefVisualizer;
import commands.CaptureCommand;
import commands.ConfigurationCoordinate;
import commands.HomeCommand;
import commands.WorkspaceCoordinate;
import cv.BlobIdentifier;
import cv.VisibleRange;

public class Controller {
    public static final double SUCCESS_THRESHOLD = -2;
    public static final double MAX_VERIFICATION_RADIUS = 4760;
    public static final double VERIFICATION_HEIGHT = -1500;
    public static final double INTERPOINT_DISTANCE = 1950;
    public static final double INTERPOINT_SIGMA = 250;
    
    Planner planner;
    String password;
    
    public Controller(Planner planner, String password) {
        this.planner = planner;
        this.password = password;
    }
    
    public void run() throws IOException {
        sendCommand(new HomeCommand(), password);

        initializeFirst(planner.beliefState);
        BeliefVisualizer.drawBeliefState(planner.beliefState, 5, "FindFirst Initial Belief; score = " + planner.beliefState.getScore());
        
        findPoint("FindFirst");
        takeVerificationImage("FindFirst");
        
        double[] firstPoint = planner.beliefState.getHighestPosition();
        System.out.printf("%nFound interesting point at %s%n%n", new Vector2(firstPoint[1], firstPoint[0]));
        
        initializeSecond(planner.beliefState, firstPoint);
        BeliefVisualizer.drawBeliefState(planner.beliefState, 5, "FindSecond Initial Belief; score = " + planner.beliefState.getScore());
        
        findPoint("FindSecond");
        takeVerificationImage("FindSecond");
        
        double[] secondPoint = planner.beliefState.getHighestPosition();
        System.out.printf("%nFound interesting point at %s%n%n", new Vector2(secondPoint[1], secondPoint[0]));
        
        initializeThird(planner.beliefState, firstPoint, secondPoint);
        BeliefVisualizer.drawBeliefState(planner.beliefState, 5, "FindThird Initial Belief; score = " + planner.beliefState.getScore());
        
        findPoint("FindThird");
        takeVerificationImage("FindThird");
        
        double[] thirdPoint = planner.beliefState.getHighestPosition();
        System.out.printf("%nFound interesting point at %s%n%n", new Vector2(thirdPoint[1], thirdPoint[0]));
        
        writeResults(firstPoint, secondPoint, thirdPoint);

        sendCommand(new HomeCommand(), password);
    }
    
    public static void initializeFirst(BeliefState belief) {
        double[][] probs = belief.locationProbabilities;
        double binLength = belief.binLength;
        for (int y = 0; y < probs.length; y++) {
            for (int x = 0; x < probs[y].length; x++) {
                double yPos = belief.yOffset + binLength * y;
                double xPos = belief.xOffset + binLength * x;
                if ((yPos > 1000 && yPos < 6000 && xPos > 1000 && xPos < 6000) ||
                        (yPos < 1000 && yPos > -1000 && xPos < 1000 && xPos > -1000)) {
                    probs[y][x] = 0.01d;
                } else {
                    probs[y][x] = 1;
                }
            }
        }
        belief.normalize();
    }
    
    public static void initializeSecond(BeliefState belief, double[] firstPosition) {
        double negTwoSigSquared = -2 * INTERPOINT_SIGMA * INTERPOINT_SIGMA;
        
        double[][] probs = belief.locationProbabilities;
        double binLength = belief.binLength;
        for (int y = 0; y < probs.length; y++) {
            for (int x = 0; x < probs[y].length; x++) {
                double yPos = belief.yOffset + binLength * y;
                double xPos = belief.xOffset + binLength * x;
                double yDiff = yPos - firstPosition[0];
                double xDiff = xPos - firstPosition[1];
                double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
                double distanceOffset = Math.abs(distance - INTERPOINT_DISTANCE);
                probs[y][x] = Math.exp(distanceOffset * distanceOffset / negTwoSigSquared);
                if (distance < INTERPOINT_DISTANCE / 2) probs[y][x] = 0;
            }
        }
        belief.normalize();
    }
    
    public static void initializeThird(BeliefState belief, double[] firstPosition, double[] secondPosition) {
        double negTwoSigSquared = -2 * (2 * INTERPOINT_SIGMA) * (2 * INTERPOINT_SIGMA);
        
        Vector2 firstVec = new Vector2(firstPosition[1], firstPosition[0]);
        Vector2 secondVec = new Vector2(secondPosition[1], secondPosition[0]);
        
        Vector2 a = firstVec.add(firstVec.sub(secondVec));
        Vector2 b = secondVec.add(secondVec.sub(firstVec));
        
        double[][] probs = belief.locationProbabilities;
        double binLength = belief.binLength;
        for (int y = 0; y < probs.length; y++) {
            for (int x = 0; x < probs[y].length; x++) {
                double yPos = belief.yOffset + binLength * y;
                double xPos = belief.xOffset + binLength * x;
                
                double yDiffA = yPos - a.y;
                double xDiffA = xPos - a.x;
                double distanceA = Math.sqrt(xDiffA * xDiffA + yDiffA * yDiffA);
                probs[y][x] = Math.exp(distanceA * distanceA / negTwoSigSquared);

                double yDiffB = yPos - b.y;
                double xDiffB = xPos - b.x;
                double distanceB = Math.sqrt(xDiffB * xDiffB + yDiffB * yDiffB);
                probs[y][x] += Math.exp(distanceB * distanceB / negTwoSigSquared);
                
                if (firstVec.distanceTo(new Vector2(xPos, yPos)) < INTERPOINT_DISTANCE / 4 || secondVec.distanceTo(new Vector2(xPos, yPos)) < INTERPOINT_DISTANCE / 4) probs[y][x] = 0;
            }
        }
        belief.normalize();
    }
    
    public boolean isInBounds(int[] point) {
        double[][] grid = planner.beliefState.locationProbabilities;
        return (point[0] >= 0 && point[0] < grid.length && point[1] >= 0 && point[1] < grid[0].length);
    }
    
    public boolean isInBounds(double[] point) {
        return isInBounds(toIndex(point));
    }
    
    public int[] toIndex(double[] point) {
        return new int[] {(int)Math.round((point[0] - planner.beliefState.yOffset) / planner.beliefState.binLength),
                (int)Math.round((point[1] - planner.beliefState.xOffset) / planner.beliefState.binLength)};
    }
    
    public void findPoint(String title) throws IOException {
        int captureIndex = 1;
        while (planner.beliefState.getScore() < SUCCESS_THRESHOLD) {
            ConfigurationCoordinate nextStep = planner.getNextStep();
            sendCommand(nextStep, password);
            captureAndUpdateBelief(title + " Image" + captureIndex, nextStep);
            captureIndex++;
        }
    }
    
    public void captureAndUpdateBelief(String title, ConfigurationCoordinate configuration) throws IOException {
        CaptureCommand capture = new CaptureCommand();
        sendCommand(capture, password);
        String filename = downloadImage(capture);
        SegmentVisualizer.displaySegmentation(title, filename);
        VisibleRange visibleRange = planner.cameraModel.getVisibleRange(configuration, planner.targetHeight);
        List<double[]> imageCenterLocations = BlobIdentifier.getCenterLocations(filename);
        if (imageCenterLocations.isEmpty()) {
            planner.beliefState.updateUnseen(configuration, visibleRange);
        } else {
            List<double[]> tableCenterLocations = imageCenterLocations.stream().map(dA -> planner.cameraModel.mapToTarget(dA, configuration, planner.targetHeight)).filter(dA -> dA != null).filter(this::isInBounds).collect(Collectors.toList());
            Optional<double[]> mostLikelyLocation = tableCenterLocations.stream().filter(dA -> {int[] iA = toIndex(dA); return dA[2] * planner.beliefState.locationProbabilities[iA[0]][iA[1]] != 0;}).max(Comparator.comparingDouble(dA -> {int[] iA = toIndex(dA); return dA[2] * planner.beliefState.locationProbabilities[iA[0]][iA[1]];}));
            if (mostLikelyLocation.isPresent()) {
                planner.beliefState.updateBelief(mostLikelyLocation.get(), configuration, planner.cameraModel, planner.targetHeight);
            } else {
                planner.beliefState.updateUnseen(configuration, visibleRange);
            }
        }
        BeliefVisualizer.drawBeliefState(planner.beliefState, 5, title + " Belief; score = " + planner.beliefState.getScore());
    }
    
    public void takeVerificationImage(String title) throws IOException {
        double[] highestPosition = planner.beliefState.getHighestPosition();
        double distance = new Vector2(highestPosition[1], highestPosition[0]).norm();
        if (distance <= MAX_VERIFICATION_RADIUS && (highestPosition[0] > 2200 || highestPosition[0] < -2200 || highestPosition[1] > 2200 || highestPosition[1] < -2200)) {
            ConfigurationCoordinate configuration = new ConfigurationCoordinate(new WorkspaceCoordinate(highestPosition[1], highestPosition[0], VERIFICATION_HEIGHT, -9000, 0));
            sendCommand(configuration, password);
            captureAndUpdateBelief(title + " Verification", configuration);
            sendCommand(new CaptureCommand(), password);
        } else {
            System.out.printf("Point [%f, %f] not reachable to take a close-up verification image.%n", highestPosition[1], highestPosition[0]);
        }
    }
    
    public static void writeResults(double[] point1, double[] point2, double[] point3) {
        Vector2 vec1 = new Vector2(point1[1], point1[0]);
        Vector2 vec2 = new Vector2(point2[1], point2[0]);
        Vector2 vec3 = new Vector2(point3[1], point3[0]);
        double vec1MiddleScore = middleScore(vec1, vec2, vec3);
        double vec2MiddleScore = middleScore(vec2, vec1, vec3);
        double vec3MiddleScore = middleScore(vec3, vec1, vec2);
        
        Vector2 mid;
        Vector2 a;
        Vector2 b;
        
        if (vec1MiddleScore > vec2MiddleScore && vec1MiddleScore > vec3MiddleScore) { //vec1 is the middle
            mid = vec1;
            a = vec2;
            b = vec3;
        } else if (vec2MiddleScore > vec3MiddleScore) { //vec2 is the middle
            mid = vec2;
            a = vec1;
            b = vec3;
        } else { //vec3 is the middle
            mid = vec3;
            a = vec1;
            b = vec2;
        }
        
        System.out.printf("Found the three interesting points of the object:%n%s%n%s%n%s%n%n", a, mid, b);
        System.out.printf("The center of the object is at:%n%s%n%n", mid);
        Vector2 difference = b.sub(a);
        double angle = Math.atan2(difference.y, difference.x);
        angle %= 2 * Math.PI;
        if (angle <= -Math.PI) angle += 2 * Math.PI;
        if (angle > Math.PI) angle -= 2 * Math.PI;
        double degrees = angle * 180 / Math.PI;
        System.out.printf("The object is oriented at %f degrees relative to the robot coordinate system.%n", degrees);
    }
    
    static double middleScore(Vector2 mid, Vector2 a, Vector2 b) {
        Vector2 firstLeg = mid.sub(a);
        Vector2 secondLeg = b.sub(mid);
        Vector2 difference = secondLeg.sub(firstLeg);
        return -difference.norm();
    }
}
