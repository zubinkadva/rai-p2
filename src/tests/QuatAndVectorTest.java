package tests;

import planner.BeliefState;
import visualization.BeliefVisualizer;
import geometry.Quaternion;
import geometry.Vector3;
import commands.AbstractCoordinate;
import commands.ConfigurationCoordinate;
import cv.CameraModel;

public class QuatAndVectorTest {
    public static void main(String[] args) {
        Vector3 up = new Vector3(0, 0, 1);
        System.out.println(up);
        Quaternion rotateOverYBy90 = Quaternion.fromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);
        System.out.println(rotateOverYBy90);
        Vector3 left = up.afterRotation(rotateOverYBy90);
        System.out.println(left);
        Quaternion rotateOverYByNeg90 = rotateOverYBy90.conjugate();
        System.out.println(rotateOverYByNeg90);
        Vector3 upRecovered = left.afterRotation(rotateOverYByNeg90);
        System.out.println(upRecovered);
        
        System.out.println();
        
        Vector3 a = new Vector3(1, 2, 3);
        System.out.println(a);
        Quaternion testRot = Quaternion.fromAxisAngle(new Vector3(1, 1, 1), 2 * Math.PI / 3);
        System.out.println(testRot);
        Vector3 b = a.afterRotation(testRot);
        System.out.println(b);
        
        System.out.println();
        
        //WorkspaceCoordinate w = new WorkspaceCoordinate(-2000, 1200, -1000, -9000, 9000);
        //ConfigurationCoordinate c = new ConfigurationCoordinate(w);
        //c = new ConfigurationCoordinate(0, 0, 9000, 0, 0);
        ConfigurationCoordinate c = new ConfigurationCoordinate(9000, 0, 16000, 0, 0);
        Vector3 position = Vector3.ZERO;
        Vector3 direction = Vector3.UP;
        Quaternion orientation = Quaternion.IDENTITY;
        
        System.out.println("Starting:");
        System.out.printf("position: %s%n", position);
        System.out.printf("direction: %s%n", direction);
        System.out.printf("orientation: %s%n", orientation);
        System.out.printf("orientation2: %s%n", Vector3.FORWARD.afterRotation(orientation));
        System.out.println();
        
        Quaternion waist = Quaternion.fromAxisAngle(Vector3.DOWN, AbstractCoordinate.roboToRadians(c.waist));
        orientation = orientation.mult(waist);
        direction = Vector3.UP.afterRotation(orientation);
        position = position.add(direction.withScale(0));
        
        System.out.printf("After waist: (%f)%n", c.waist);
        System.out.printf("position: %s%n", position);
        System.out.printf("direction: %s%n", direction);
        System.out.printf("orientation: %s%n", orientation);
        System.out.printf("orientation2: %s%n", Vector3.FORWARD.afterRotation(orientation));
        System.out.println();
        
        Quaternion shoulder = Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(c.shoulder));
        orientation = orientation.mult(shoulder);
        direction = Vector3.UP.afterRotation(orientation);
        position = position.add(direction.withScale(2500));
        
        System.out.printf("After shoulder: (%f)%n", c.shoulder);
        System.out.printf("position: %s%n", position);
        System.out.printf("direction: %s%n", direction);
        System.out.printf("orientation: %s%n", orientation);
        System.out.printf("orientation2: %s%n", Vector3.FORWARD.afterRotation(orientation));
        System.out.println();
        
        Quaternion elbow = Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(c.elbow));
        orientation = orientation.mult(elbow);
        direction = Vector3.UP.afterRotation(orientation);
        position = position.add(direction.withScale(2500));
        
        System.out.printf("After elbow: (%f)%n", c.elbow);
        System.out.printf("position: %s%n", position);
        System.out.printf("direction: %s%n", direction);
        System.out.printf("orientation: %s%n", orientation);
        System.out.printf("orientation2: %s%n", Vector3.FORWARD.afterRotation(orientation));
        System.out.println();
        
        Quaternion wrist = Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(c.wrist));
        orientation = orientation.mult(wrist);
        direction = Vector3.UP.afterRotation(orientation);
        position = position.add(direction.withScale(1300));
        
        System.out.printf("After wrist: (%f)%n", c.wrist);
        System.out.printf("position: %s%n", position);
        System.out.printf("direction: %s%n", direction);
        System.out.printf("orientation: %s%n", orientation);
        System.out.printf("orientation2: %s%n", Vector3.FORWARD.afterRotation(orientation));
        System.out.println();
        
        Quaternion handTwist = Quaternion.fromAxisAngle(Vector3.UP, AbstractCoordinate.roboToRadians(c.hand));
        orientation = orientation.mult(handTwist);
        direction = Vector3.UP.afterRotation(orientation);
        position = position.add(direction.withScale(0));
        
        System.out.printf("After hand twist: (%f)%n", c.hand);
        System.out.printf("position: %s%n", position);
        System.out.printf("direction: %s%n", direction);
        System.out.printf("orientation: %s%n", orientation);
        System.out.printf("orientation2: %s%n", Vector3.FORWARD.afterRotation(orientation));
        System.out.println();
        
        System.out.println(new CameraModel().getCameraOrientation(c));
        System.out.println(
                Quaternion.fromAxisAngle(Vector3.DOWN, AbstractCoordinate.roboToRadians(c.waist)).mult(
                Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(c.shoulder))).mult(
                Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(c.elbow))).mult(
                Quaternion.fromAxisAngle(Vector3.LEFT, AbstractCoordinate.roboToRadians(c.wrist))).mult(
                Quaternion.fromAxisAngle(Vector3.UP, AbstractCoordinate.roboToRadians(c.hand))));
        System.out.println(Quaternion.IDENTITY.mult(waist).mult(shoulder).mult(elbow).mult(wrist).mult(handTwist));
        System.out.println(c.getUnescapedCommandString());
        
        BeliefState belief = new BeliefState(new double[100][100], 100, 0, 0);
        for (int y = 0; y < 100; y++) {
            for (int x = 0; x < 100; x++) {
                belief.locationProbabilities[y][x] = 1;
            }
        }
        belief.normalize();
        
        belief.updateUnseen(c, new CameraModel().getVisibleRange(c, -3150));

        BeliefVisualizer.drawBeliefState(belief, 5, "after");
    }
}
