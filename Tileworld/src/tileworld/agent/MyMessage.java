package tileworld.agent;

import sim.util.Int2D;

public class MyMessage extends Message{

    private Int2D coordinate;

    public MyMessage(String from, String to, MessageType type, Int2D coordinate) {
        super(from, to, type.toString());
        this.coordinate = coordinate;
    }

    public Int2D getCoordinate() {
        return coordinate;
    }

    public String getType() {
        return this.getMessage();
    }
}

enum MessageType {
    AGENT,
    FUEL_STATION,
    TARGET
}