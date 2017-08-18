package commands;
//Author: Roger Ballard
//Class: Robotics & AI, Spring 2017

//Note: Not thread-safe! (Not that it needs to be for this project.)
public class CaptureCommand extends AbstractCommand {
    public final static CaptureCommand DELETE_COMMAND = new CaptureCommand(-1);
    protected static int nextId = 0;
    
    public final int id;
    
    public CaptureCommand() {
        this(nextId);
        nextId++;
    }
    
    protected CaptureCommand(int id) {
        this.id = id;
    }

    @Override
    public String getUnescapedCommandString() {
        return String.format("%d CAPTURE", id);
    }
}
