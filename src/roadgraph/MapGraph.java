/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private int numVertices;
	private List<MapEdge> edges;
	private int numEdges;
	private HashMap<GeographicPoint,MapNode> vertices;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor
		vertices = new HashMap<GeographicPoint, MapNode>();
		int numVertices = 0;
		this.numVertices = numVertices;
		
		int numEdges = 0;
		this.numEdges = numEdges;
		edges = new ArrayList<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method
		return vertices.values().size();
	}
	

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method.
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (ONLY if the node
	 * was already in the graph, or the parameter is null).
	 */
	
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method
		if (location == null) {
			return false;
		}
		MapNode mNode = vertices.get(location);
		if (mNode == null) {
			mNode = new MapNode(location);
			vertices.put(location, mNode);
			return true;
		} else {
			// Node already exists at location 
			return false;
		}
		
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2. This will perform the required 
	 * error checking, get the MapNode associated with the "from" point and call the
	 * addEdge() method of the MapNode instance.
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	
	
	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName, String roadType, double length) {
		MapNode node1 = vertices.get(pt1);
		MapNode node2 = vertices.get(pt2);

		// check nodes are valid
		if (node1 == null) {
			throw new NullPointerException("pt1:" + pt1 + "not found");
		}
		if (node2 == null) {
			throw new NullPointerException("pt2:" + pt2 + "not found");
		}

		MapEdge edge = new MapEdge(roadName, roadType, length);
		edges.add(edge);
		node1.addEdge(pt2, roadName, roadType, length);
	}
	
	
	/** Find the path from start to goal using breadth first search - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search - Called by MapApp
	 *  Use of helper methods is encouraged
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
	    Set<GeographicPoint> visited = new HashSet<>();
	    Queue<GeographicPoint> queue = new LinkedList<>();
	    HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
	    parentMap.put(start, null);
	    queue.add(start);

	    while (!queue.isEmpty()) {
	        GeographicPoint current = queue.poll();
	        nodeSearched.accept(current);

	        if (current.equals(goal)) {
	            return constructPath(parentMap, goal);
	        }

	        visited.add(current);
	        //add all unvisited neighbors to the queue and mark them as visited. Update the parent map
	        addUnvisitedNeighbors(current, visited, queue, parentMap);
	    }

	    return null;
	}
	
	
	private void addUnvisitedNeighbors(GeographicPoint current, Set<GeographicPoint> visited,
			Queue<GeographicPoint> queue,
			HashMap<GeographicPoint, GeographicPoint> parentMap) {
		List<GeographicPoint> neighbors = getNeighbors(current);

		for (GeographicPoint neighbor : neighbors) {
			if (!visited.contains(neighbor)) {
				queue.add(neighbor);
				visited.add(neighbor);
				parentMap.put(neighbor, current);
			}
		}
	}

    private List<GeographicPoint> constructPath(Map<GeographicPoint, GeographicPoint> parentMap, GeographicPoint goal) {
        ArrayList<GeographicPoint> path;
        path = new ArrayList<GeographicPoint>();
        GeographicPoint current = goal;

        while (current != null) {
            path.add(0, current);
            current = parentMap.get(current);
        }

        return path;
    }
    
    private List<GeographicPoint> getNeighbors(GeographicPoint current) {
        List<GeographicPoint> neighbors = new ArrayList<GeographicPoint>();

        // get the MapNode corresponding to the current GeographicPoint
        MapNode cNode = vertices.get(current);

        if (cNode != null) {
            // Get neighboring points from the current node
            Set<GeographicPoint> neighborPoints = cNode.getNeighborPoints();

            // Add the neighboring points to the neighbors list
            for (GeographicPoint neighborPoint : neighborPoints) {
                neighbors.add(neighborPoint);
            }
        }

        return neighbors;
    }
	
	
	


	// DO NOT CODE ANYTHING BELOW THIS LINE UNTIL PART2

	/** Find the path from start to goal using Dijkstra's algorithm - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm - Called by MapApp
	 *  Use of helper methods is encouraged
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method with Part 2

		// Hook for visualization.  See writeup. Note that this may actually need to be
		// located in a helper method.
		 PriorityQueue<QueueElement> frontier = new PriorityQueue<>();
		    Set<GeographicPoint> visited = new HashSet<>();
		    Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		    Map<GeographicPoint, Double> distanceMap = new HashMap<>();

		    initializeMaps(start, distanceMap);
		    addToFrontier(start, frontier);

		    while (!frontier.isEmpty()) {
		        QueueElement currentElement = frontier.poll();
		        GeographicPoint current = currentElement.getPoint();
		        nodeSearched.accept(current);

		        if (current.equals(goal)) {
		            return constructPath(parentMap, current);
		        }

		        visited.add(current);
		        exploreNeighbors(current, visited, frontier, parentMap, distanceMap);
		    }

		    return null;
		 }
	
	private void initializeMaps(GeographicPoint start, Map<GeographicPoint, Double> distanceMap) {
	    for (GeographicPoint point : getAllNodes()) {
	        distanceMap.put(point, Double.POSITIVE_INFINITY);
	    }
	    distanceMap.put(start, 0.0);
	}
	
	private void addToFrontier(GeographicPoint start, PriorityQueue<QueueElement> frontier) {
	    frontier.add(new QueueElement(start, 0.0));
	}
	
	private void exploreNeighbors(GeographicPoint current, Set<GeographicPoint> visited,
            PriorityQueue<QueueElement> frontier, Map<GeographicPoint, GeographicPoint> parentMap,
            Map<GeographicPoint, Double> distanceMap) {
		for (GeographicPoint neighbor : getNeighbors(current)) {
			if (!visited.contains(neighbor)) {
				double newDistance = distanceMap.get(current) + current.distance(neighbor);

				if (newDistance < distanceMap.get(neighbor)) {
					distanceMap.put(neighbor, newDistance);
					parentMap.put(neighbor, current);
					frontier.add(new QueueElement(neighbor, newDistance));
				}
			}
		}
	}


	/** Find the path from start to goal using A-Star search - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	
	/** Find the path from start to goal using A-Star search - Called by MapApp
	 *  Use of helper methods is encouraged
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
	    // Initialize data structures
	    Map<GeographicPoint, Double> distanceMap = new HashMap<>();
	    Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
	    PriorityQueue<QueueElement> queue = new PriorityQueue<>();
	    Set<GeographicPoint> visited = new HashSet<>();

	    // Initialize distance from start to each node with a large value except for start node (0)
	    initializeMaps(start, distanceMap);

	    // Add start node to the priority queue with priority distance
	    queue.offer(new QueueElement(start, 0.0, goal));

	    while (!queue.isEmpty()) {
	        QueueElement currentElement = queue.poll();
	        double distFromStart = currentElement.getDistFromStart();
	        GeographicPoint current = currentElement.getPoint();
	        nodeSearched.accept(current);

	        if (!visited.contains(current)) {
	            visited.add(current);

	            // Check if the goal node has been reached
	            if (current.equals(goal)) {
	                return constructPath(parentMap, current);
	            }

	            exploreNeighbors(current, distFromStart, goal, distanceMap, parentMap, queue);
	        }
	    }

	    // No path found from start to goal
	    return null;
	}


	/**
	 * 
	 * @param current
	 * @param distFromStart
	 * @param goal
	 * @param distanceMap
	 * @param parentMap
	 * @param queue
	 */
	private void exploreNeighbors(GeographicPoint current, double distFromStart, GeographicPoint goal,
	                              Map<GeographicPoint, Double> distanceMap,
	                              Map<GeographicPoint, GeographicPoint> parentMap,
	                              PriorityQueue<QueueElement> queue) {
	    for (GeographicPoint neighbor : getNeighbors(current)) {
	        double distanceFromStart = distFromStart + vertices.get(current).getEdges().get(neighbor).getRoadLength();
	        double heuristic = neighbor.distance(goal);
	        double priorityDistance = distanceFromStart + heuristic;

	        if (priorityDistance < distanceMap.get(neighbor)) {
	            distanceMap.put(neighbor, priorityDistance);
	            parentMap.put(neighbor, current);
	            queue.offer(new QueueElement(neighbor, distanceFromStart, goal));
	        }
	    }
	}


	/**
	 * returns all the nodes
	 * @return
	 */
	public Set<GeographicPoint> getAllNodes() {
	    Set<GeographicPoint> nodes = new HashSet<>();

	    for (GeographicPoint startPoint : vertices.keySet()) {
	        nodes.add(startPoint);
	        MapNode mapNode = vertices.get(startPoint);
	        for (GeographicPoint neighbor : mapNode.getNeighborPoints()) {
	            nodes.add(neighbor);
	        }
	    }

	    return nodes;
	}
	
	

	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should visit 13 nodes and AStar should visit 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should visit 37 nodes and AStar should visit 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
	}
	
}
