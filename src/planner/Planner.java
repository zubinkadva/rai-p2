package planner;

import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.stream.Collectors;

import commands.ConfigurationCoordinate;
import commands.WorkspaceCoordinate;

import cv.CameraModel;
import cv.VisibleRange;

public class Planner {
    public int timestepLookahead;
    public int beamWidth;
    public int beamSearchDepth;
    public int beamBranchingFactor; //per dimension
    public double[][] configurationBounds; //TODO: add workspace bound checks, as well. //[waist, shoulder, elbow, wrist, handTwist]
    public int evidenceSampleCount;
    public BeliefState beliefState;
    public CameraModel cameraModel;
    public double targetHeight; //targetHeight ~= -3150
    public boolean reuseScoresOptimization;
    
    public Planner(int timestepLookahead, int beamWidth, int beamSearchDepth, int beamBranchingFactor, double[][] configurationBounds, int evidenceSampleCount, BeliefState beliefState, CameraModel cameraModel, double targetHeight, boolean reuseScoresOptimization) {
        if (beamBranchingFactor % 2 == 0) throw new IllegalArgumentException("Beam branching factor must be odd");
        
        this.timestepLookahead = timestepLookahead;
        this.beamWidth = beamWidth;
        this.beamSearchDepth = beamSearchDepth;
        this.beamBranchingFactor = beamBranchingFactor;
        this.configurationBounds = configurationBounds;
        this.evidenceSampleCount = evidenceSampleCount;
        this.beliefState = beliefState;
        this.cameraModel = cameraModel;
        this.targetHeight = targetHeight;
        this.reuseScoresOptimization = reuseScoresOptimization;
    }
    
    //After profiling, I have determined that ~80% of the CPU time is spent in updateBelief and updateUnseen. Therefore, these are good candidates for parallelization.
    //Because of the small size of these methods, and the number of times they get called by getPlannedActingRange, I have determined that this is a good candidate function for parallelization.

    /* Original code:
    public ScoredActingRange getPlannedActingRange(int remainingTimeLookahead, double[][] searchBounds, BeliefState currentBelief) {
        if (remainingTimeLookahead == 0) return new ScoredActingRange(null, currentBelief.getScore());
        
        List<ScoredActingRange> bestPlans = new LinkedList<ScoredActingRange>();
        bestPlans.add(getScoredActingRange(remainingTimeLookahead, searchBounds, currentBelief));
        
        for (int depth = 0; depth < beamSearchDepth; depth++) {
            for (int branchingDimension = 0; branchingDimension < searchBounds.length; branchingDimension++) {
                List<ScoredActingRange> childPlans = new LinkedList<ScoredActingRange>();
                for (ScoredActingRange parent : bestPlans) {
                    double[][][] splitSearchBounds = splitSearchBounds(parent.actingRange, branchingDimension, beamBranchingFactor);
                    for (int x = 0; x < splitSearchBounds.length; x++) {
                        double[][] childSearchBounds = splitSearchBounds[x];
                        if (x == splitSearchBounds.length / 2 && reuseScoresOptimization) { //Don't recompute the score of a centerpoint if it is already known. //TODO: Think some more about whether this will cause more errors, because scores that were randomly sampled to be higher will remain and stay randomly higher.
                            childPlans.add(new ScoredActingRange(childSearchBounds, parent.expectedUtility));
                        } else {
                            childPlans.add(getScoredActingRange(remainingTimeLookahead, childSearchBounds, currentBelief));
                        }
                    }
                }
                //TODO: Examine if this tends to eliminate any branches too early on
                bestPlans = getTopN(childPlans.stream().filter(sar -> Planner.isInBounds(sar.getPlannedAction().plannedCommand)).collect(Collectors.toList()), beamWidth);
            }
        }
        
        return bestPlans.stream().max(Comparator.comparingDouble(sar -> sar.expectedUtility)).get();
    }*/

    public ScoredActingRange getPlannedActingRange(int remainingTimeLookahead, double[][] searchBounds, BeliefState currentBelief) {
        if (remainingTimeLookahead == 0) return new ScoredActingRange(null, currentBelief.getScore());
        
        List<ScoredActingRange> bestPlans = new LinkedList<ScoredActingRange>();
        bestPlans.add(getScoredActingRange(remainingTimeLookahead, searchBounds, currentBelief));
        
        for (int depth = 0; depth < beamSearchDepth; depth++) {
            for (int branchingDimension = 0; branchingDimension < searchBounds.length; branchingDimension++) {
                int chosenDimension = branchingDimension;
                List<ScoredActingRange> childPlans = 
                bestPlans.stream().parallel().flatMap(parent -> {
                    double[][][] splitSearchBounds = splitSearchBounds(parent.actingRange, chosenDimension, beamBranchingFactor);
                    List<ScoredActingRange> personalChildren = new LinkedList<ScoredActingRange>();
                    for (int x = 0; x < splitSearchBounds.length; x++) {
                        double[][] childSearchBounds = splitSearchBounds[x];
                        if (x == splitSearchBounds.length / 2 && reuseScoresOptimization) { //Don't recompute the score of a centerpoint if it is already known. //TODO: Think some more about whether this will cause more errors, because scores that were randomly sampled to be higher will remain and stay randomly higher.
                            personalChildren.add(new ScoredActingRange(childSearchBounds, parent.expectedUtility));
                        } else {
                            personalChildren.add(getScoredActingRange(remainingTimeLookahead, childSearchBounds, currentBelief));
                        }
                    }
                    return personalChildren.stream();
                }).collect(Collectors.toList());
                //TODO: Examine if this tends to eliminate any branches too early on
                bestPlans = getTopN(childPlans.stream().filter(sar -> Planner.isInBounds(sar.getPlannedAction().plannedCommand)).collect(Collectors.toList()), beamWidth);
            }
        }
        
        return bestPlans.stream().max(Comparator.comparingDouble(sar -> sar.expectedUtility)).get();
    }
    
    public ScoredActingRange getPlannedActingRange(int remainingTimeLookahead, BeliefState currentBelief) {
        return getPlannedActingRange(remainingTimeLookahead, configurationBounds, currentBelief);
    }
    
    public ScoredActingRange getPlannedActingRange() {
        return getPlannedActingRange(timestepLookahead, beliefState);
    }
    
    public ConfigurationCoordinate getNextStep() {
        return getPlannedActingRange().getCenter();
    }
    
    public double getExpectedScore(int remainingTimeLookahead, ConfigurationCoordinate configuration, BeliefState currentBelief) {
        VisibleRange visibleRange = cameraModel.getVisibleRange(configuration, targetHeight);
        
        double sumScore = 0;
        for (int e = 0; e < evidenceSampleCount; e++) {
            double[] sampledPosition = currentBelief.samplePosition();
            BeliefState resultantBelief = new BeliefState(currentBelief);
            if (visibleRange.contains(sampledPosition)) {
                resultantBelief.updateBelief(sampledPosition, configuration, cameraModel, targetHeight);
            } else {
                resultantBelief.updateUnseen(configuration, visibleRange);
            }
            ScoredActingRange resultantPlan = getPlannedActingRange(remainingTimeLookahead - 1, resultantBelief);
            sumScore += resultantPlan.expectedUtility;
        }
        return sumScore / evidenceSampleCount;
    }
    
    public ScoredActingRange getScoredActingRange(int remainingTimeLookahead, double[][] actingRange, BeliefState currentBelief) {
        return new ScoredActingRange(actingRange, getExpectedScore(remainingTimeLookahead, ScoredActingRange.getCenterOfActingRange(actingRange), currentBelief));
    }
    
    public <T extends Comparable<T>> List<T> getTopN(List<T> original, int n, Comparator<T> comparator) {
        PriorityQueue<T> heap = new PriorityQueue<T>(comparator);
        for (T elem : original) {
            heap.add(elem);
            if (heap.size() > n) heap.poll();
        }
        return new LinkedList<T>(heap);
    }
    
    public <T extends Comparable<T>> List<T> getTopN(List<T> original, int n) {
        return getTopN(original, n, Comparator.naturalOrder());
    }

    public double[][][] splitSearchBounds(double[][] searchBounds, int splitDimension, int splitCount) {
        double[][][] splitSearchBounds = new double[splitCount][searchBounds.length][];
        for (int x = 0; x < splitCount; x++) {
            for (int d = 0; d < searchBounds.length; d++) {
                if (d == splitDimension) {
                    splitSearchBounds[x][d] = splitRange(searchBounds[d], x, splitCount);
                } else {
                    splitSearchBounds[x][d] = searchBounds[d];
                }
            }
        }
        return splitSearchBounds;
    }
    
    public double[] splitRange(double[] range, int index, int splitCount) {
        double length = range[1] - range[0];
        double segmentLength = length / splitCount;
        return new double[] {range[0] + (segmentLength * index), range[0] + (segmentLength * (index + 1))};
    }
    
    public static boolean isInBounds(ConfigurationCoordinate configuration) {
        //The robot-controlling server doesn't have a good idea of the calibration of the robot; I have to convert to its view to check and see if a command will be valid or not before I send it.
        ConfigurationCoordinate boundsConfiguration = new ConfigurationCoordinate(configuration.waist * ConfigurationCoordinate.WAIST_MULTIPLE, configuration.shoulder, configuration.elbow, configuration.wrist, configuration.hand + ConfigurationCoordinate.HAND_OFFSET);
        WorkspaceCoordinate workspace = new WorkspaceCoordinate(boundsConfiguration);
        if (workspace.x <= -4800 || workspace.x >= 4800) return false;
        if (workspace.y <= -4800 || workspace.y >= 4800) return false;
        if (workspace.z <= -1800) return false;
        if (workspace.z <= 0 && workspace.x >= -2200 && workspace.x <= 2200 && workspace.y >= -2200 && workspace.y <= 2200) return false;
        return true;
    }
}
