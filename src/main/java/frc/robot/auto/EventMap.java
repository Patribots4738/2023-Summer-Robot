package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj2.command.Command;

public class EventMap {
    private PPRamseteCommand baseCommand;
    private HashMap<String, Command> eventMap = new HashMap<>();

    private FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(getPathEventCommand(), null, eventMap);
    
    public EventMap(PPRamseteCommand command, Command[] events){
        this.baseCommand = command;
        this.eventMap = addEvents(events);
    }

    private HashMap<String, Command> addEvents(Command[] events){
        HashMap<String, Command> eventMap = new HashMap<>();

        for(int i = 0; i < events.length; i++){
            eventMap.put("marker" + (i+1), events[i]);
        }

        return eventMap;
    }

    public FollowPathWithEvents getPathEventCommand(){
      return null;
    }
}
