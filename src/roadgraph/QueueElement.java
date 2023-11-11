package roadgraph;

import geography.GeographicPoint;


/** Class to hold an element of the PriorityQueue. Relates a point to distance from start and straightline
 * distance to goal.
 * 
 * @author Mr. Murray
 *
 */
public class QueueElement implements Comparable<QueueElement> {
	
	private GeographicPoint point;
    private double distFromStart;
    private double priorityDistance;
    
    
    /** Constructor for Dijkstra search
     * @param pt - current point
     * @param d1 - distance from start to this pt on current path
     */
    public QueueElement(GeographicPoint pt, double d1) 
    {
    	this.point = pt;
    	this.distFromStart = d1;
    	this.priorityDistance = this.distFromStart;
    }
    
    
    /** Constructor for A-star search
     * @param pt - current point
     * @param d1 - distance from start to this pt on current path
     * @param goal - goalNode
     */
    public QueueElement(GeographicPoint pt, double d1, GeographicPoint goal) 
    {
    	this.point = pt;
    	this.distFromStart = d1;
    	this.priorityDistance = this.distFromStart + pt.distance(goal);
    }
    
    /** gets the current point
     * @return
     */
    public GeographicPoint getPoint()
    {
    	return point;
    }
    
    /** Gets the best known distance from start at the time this QueueElement
     *  was created.
     * @return
     */
    public double getDistFromStart() 
    {
    	return distFromStart;
    }
    
    
    /** Gets the best known distance from start at the time this QueueElement
     *  was created.
     * @return
     */
    public double getPriorityDistance() 
    {
    	return priorityDistance;
    }
    
    
    
    /** Comparator function for the priority queue for Dijkstra. 
     *
     */
    public int compareTo(QueueElement o )
    {
    	return Double.compare(this.priorityDistance,o.getPriorityDistance());
    }

}
