package visualization;

import java.util.Arrays;

import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.GrayF32;
import planner.BeliefState;

public class BeliefVisualizer {
    public static void drawBeliefState(BeliefState beliefState, int scaleFactor, String title) {
        double[][] values = beliefState.locationProbabilities;
        double max = Arrays.stream(values).flatMapToDouble(dA -> Arrays.stream(dA)).max().getAsDouble();
        double scale = 255 / max;
        int height = values.length * scaleFactor;
        int width = values[0].length * scaleFactor;
        GrayF32 image = new GrayF32(width, height);
        for( int y = 0; y < height; y++ ) {
            for( int x = 0; x < width; x++ ) {
                image.set(y, x, (float)(values[y / scaleFactor][x / scaleFactor] * scale));
            }
        }
 
        ShowImages.showWindow(image, title);
        UtilImageIO.saveImage(image, "savedVisualizations/" + title + ".bmp");
    }
}
