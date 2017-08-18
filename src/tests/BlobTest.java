package tests;

import static util.Utilities.sendCommand;
import static util.Utilities.downloadImage;

import java.io.IOException;

import commands.CaptureCommand;
import commands.Command;
import commands.ConfigurationCoordinate;
import cv.BlobIdentifier;

public class BlobTest {
    public static void main(String[] args) throws IOException {
        String password = args[0];
        Command command = new ConfigurationCoordinate(-9000, 0, 9000, 9000, 0);
        System.out.printf("Sending command: %s%n", command.getUnescapedCommandString());
        sendCommand(command, password);
        CaptureCommand capture = new CaptureCommand();
        System.out.printf("Sending command: %s%n", capture.getUnescapedCommandString());
        sendCommand(capture, password);
        System.out.println("Downloading image");
        String fileName = downloadImage(capture);
        System.out.println("Finding blob centers");
        for (double[] center : BlobIdentifier.getCenterLocations(fileName)) {
            System.out.printf("x: %f y: %f size: %d%n", center[1], center[0], (int)center[2]);
        }
    }
}
