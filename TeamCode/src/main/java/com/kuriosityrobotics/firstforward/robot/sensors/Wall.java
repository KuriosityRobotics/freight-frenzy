package com.kuriosityrobotics.firstforward.robot.sensors;

import static com.kuriosityrobotics.firstforward.robot.util.Constants.Field.FULL_FIELD;

import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.Segment;
import org.apache.commons.geometry.euclidean.twod.Vector2D;

public enum Wall {
    LEFT(Lines.segmentFromPoints(Vector2D.of(0, 0), Vector2D.of(0, FULL_FIELD), DistanceSensorLocaliser.p)),
    RIGHT(Lines.segmentFromPoints(Vector2D.of(FULL_FIELD, 0), Vector2D.of(FULL_FIELD, FULL_FIELD), DistanceSensorLocaliser.p)),
    FRONT(Lines.segmentFromPoints(Vector2D.of(0, FULL_FIELD), Vector2D.of(FULL_FIELD, FULL_FIELD), DistanceSensorLocaliser.p)),
    BACK(Lines.segmentFromPoints(Vector2D.of(0, 0), Vector2D.of(FULL_FIELD, 0), DistanceSensorLocaliser.p));

    private final Segment segment;

    Wall(Segment segment) {
        this.segment = segment;
    }

    public Segment getSegment() {
        return segment;
    }
}
