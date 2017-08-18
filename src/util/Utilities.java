package util;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.channels.Channels;
import java.nio.channels.FileChannel;
import java.nio.channels.ReadableByteChannel;

import commands.CaptureCommand;
import commands.Command;
//Author: Roger Ballard
//Class: Robotics & AI, Spring 2017

public class Utilities {
    public static String htmlEscape(String string) {
        return string.replace(" ", "%20");
    }
    
    public static int sendCommand(Command command, String password) throws IOException {
        System.out.printf("Sending %s%n", command.getUnescapedCommandString());
        String urlString = String.format("http://debatedecide.fit.edu/robot.php?o=369&m=Y&p=%s&c=%s", password, command.getCommandString());
        URL url = new URL(urlString);
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        int responseCode = connection.getResponseCode();
        return responseCode;
    }
    
    public static String downloadImage(CaptureCommand captureCommand) throws IOException {
        return downloadImage(captureCommand.id);
    }
    
    public static String downloadImage(int imageId) throws IOException {
        String fileName = String.format("%d.bmp", imageId);
        System.out.printf("Downloading Image: \"%s\"%n", fileName);
        downloadFile("http://debatedecide.fit.edu/robot/" + fileName, "images/" + fileName);
        return "images/" + fileName;
    }
    
    //Credit to Holger of http://stackoverflow.com/questions/18872611/download-file-from-server-in-java
    //for the main downloading logic; now somewhat modified
    public static void downloadFile(String sourceUrlString, String destinationPathString) throws IOException {
        URL url = new URL(sourceUrlString);
        InputStream inputStream = url.openStream();
        ReadableByteChannel inputChannel = Channels.newChannel(inputStream);
        
        try(FileOutputStream outputStream = new FileOutputStream(destinationPathString)) {
            FileChannel outputChannel = outputStream.getChannel();
            outputChannel.transferFrom(inputChannel, 0, Long.MAX_VALUE);
        }
    }
}
