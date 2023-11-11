package roadgraph;

import java.util.Set;
import java.util.HashMap;
import java.util.HashSet;

import geography.GeographicPoint;


// TODO: Auto-generated Javadoc
/**
 * The Class MapNode.
 */
public class MapNode {
	
	/** The edges HashMap. The GeographicPoint is the "to" location for an
	 *  edge that is an out neighbor of this MapNode. The MapEdge contains
	 *  the rest of the edge-specific data. A HashMap is used for search efficiency 
	 *  (vs iteration through an ArrayList or LinkedList). Note that this
	 *  requires that only one path from this node to any other node (dictated
	 *  by the HashMap implementation);
	 */
	HashMap<GeographicPoint,MapEdge> edges;
	private GeographicPoint location;
	private double distance;
	/**
	 * MapNode constructor. Must instantiate (ie, "new") the edges HashMap
	 */
	public MapNode() 
	{
		edges = new HashMap<GeographicPoint, MapEdge>();
	}
	
	public MapNode(GeographicPoint location) 
	{
		edges = new HashMap<GeographicPoint, MapEdge>();
		this.location = location;
	}
	
	public GeographicPoint getLocation() {
        return location;
    }
	
	/**
	 * Gets the neighbors.
	 *
	 * @return the set of neighbor MapNodes
	 */
	public Set<MapNode> getNeighbors() {
		Set<MapNode> neighbors = new HashSet<MapNode>();
        for (GeographicPoint nLoc : edges.keySet()) {
            MapNode nNode = new MapNode(nLoc);
            neighbors.add(nNode);
        }
        return neighbors;
	}
	
	/**
	 * Gets the edges.
	 *
	 * @return the edges
	 */
	public HashMap<GeographicPoint, MapEdge> getEdges()
	{
		return edges;
	}
	
	/**
	 * Gets the Geographic points of all neighboring points as a Set
	 *
	 * @return the neighbor points
	 */
	public Set<GeographicPoint> getNeighborPoints() 
	{
		return edges.keySet();
	}
	
	/**
	 * Gets the edge associated with the specified "to" Geographic Point,
	 * if the point is not null and exists in the edges HashMap; otherwise,
	 * returns null;
	 *
	 * @param to  the GeographicPoint of a neighboring node.
	 * @return the edge or null
	 */
	public MapEdge getEdge(GeographicPoint to)
	{
		if (to != null && edges.containsKey(to)) {
            return edges.get(to);
        }
        return null;
	}
	
	
	
	/**
	 * Adds the edge to the edges HashMap. Returns true UNLESS
	 * the "to" GeographicPoint already exists in the edges HashMap
	 *
	 * @param to the GeographicPoint of the neighboring node.
	 * @param roadName the road name
	 * @param roadType the road type
	 * @param length the length of the road
	 */
	public boolean addEdge(GeographicPoint to, String roadName, String roadType, double length)
	{
		if (to != null && !edges.containsKey(to)) {
            MapEdge edge = new MapEdge(roadName, roadType, length);
            edges.put(to, edge);
            return true;
        }
		// to point already exists or is null
        return false;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

}
