package roadgraph;

import java.util.ArrayList;
import java.util.function.Consumer;

import geography.GeographicPoint;

public class PointTracker {
	ArrayList<GeographicPoint> points;
	
	public PointTracker() {
		points = new ArrayList<GeographicPoint>();
	}

    public void acceptPoint(geography.GeographicPoint point) {
    	points.add(point);

//        System.out.println("accepted point : " + point);
    }

    public ArrayList<GeographicPoint> getPoints() {
    	return points;
    }
	
	
}
