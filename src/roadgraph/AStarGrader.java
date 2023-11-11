package roadgraph;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC Development Team
 * Grader for Module 4, Part 2.
 */
public class AStarGrader implements Runnable {
    public String feedback;

    public int correct;

    private static final int TESTS = 4;

    // SET THIS TO TRUE IF YOU NEED TO USE THE DEBUGGER!!!!
    private static final boolean DEBUG = false;

    /** Format readable feedback */
    public static String printOutput(double score, String feedback) {
        return "Score: " + score + "\nFeedback: " + feedback;
    }

    /** Format test number and description */
    public static String appendFeedback(int num, String test) {
        return "\n** Test #" + num + ": " + test + "...";
    }

    public static void main(String[] args) {
        AStarGrader grader = new AStarGrader();

        if (DEBUG) {
        	// if you are trying to debug in eclipse, set DEBUG to true
        	// eclipse doesn't like debugging multiple threads...
        	grader.run();
        } else {
        	// Infinite loop detection
        	Thread thread = new Thread(grader);
        	thread.start();

        	// Allow it to run for 10 seconds
        	long endTime = System.currentTimeMillis() + 10000;
        	boolean infinite = false;
        	while(thread.isAlive()) {
        		if (System.currentTimeMillis() > endTime) {
        			// Stop the thread if it takes too long
        			thread.stop();
        			infinite = true;
        			break;
        		}
        	}
        	if (infinite) {
        		System.out.println(printOutput((double)grader.correct / TESTS, grader.feedback + "\nYour program entered an infinite loop."));
        	}
        }
    }

    /** Run a test case on an adjacency list and adjacency matrix.
     * @param i The graph number
     * @param file The file to read from
     * @param desc A description of the graph
     * @param start The point to start from
     * @param end The point to end at
     */
    public void runTest(int i, String file, String desc, GeographicPoint start, GeographicPoint end) {
        MapGraph graph = new MapGraph();

        feedback += "\n\n" + desc;

        GraphLoader.loadRoadMap("data/graders/mod4/" + file, graph);
        CorrectAnswer corr = new CorrectAnswer("data/graders/mod4/" + file + ".answer", false);

        judge(i, graph, corr, start, end);
    }

    /** Compare the user's result with the right answer.
     * @param i The graph number
     * @param result The user's graph
     * @param corr The correct answer
     * @param start The point to start from
     * @param end The point to end at
     */
    public void judge(int i, MapGraph result, CorrectAnswer corr, GeographicPoint start, GeographicPoint end) {
    	// Correct if paths are same length and have the same elements
        feedback += appendFeedback(i, "Running A* from (" + start.getX() + ", " + start.getY() + ") to (" + end.getX() + ", " + end.getY() + ")");
        PointTracker pt = new PointTracker();
        Consumer<geography.GeographicPoint> nodeAccepter = pt::acceptPoint;
        List<GeographicPoint> path = result.aStarSearch(start, end, nodeAccepter);
        ArrayList<GeographicPoint> points = pt.getPoints();
        int numVisited = points.size();
        if (path == null) {
            if (corr.path == null) {
                feedback += "PASSED.";
                correct++;
            } else {
                feedback += "FAILED. Your implementation returned null; expected \n" + printPath(corr.path) + ".";
            }
        } else if (path.size() != corr.path.size() || !corr.path.containsAll(path)) {
            feedback += "FAILED. Expected: \n" + printPath(corr.path) + "Got: \n" + printPath(path);
            if (path.size() != corr.path.size()) {
                feedback += "Your result has size " + path.size() + "; expected " + corr.path.size() + ".";
            } else {
                feedback += "Correct size, but incorrect path.";
            }
        } else if (((corr.visited.size()-1) != numVisited) && (corr.visited.size() != numVisited)) {
        	feedback += "FAILED. Expected: "+corr.visited.size()+" nodes visited. Got "+numVisited+" nodes visited\n";
        	feedback += printVisitedPoints(points,corr.visited);
//        } else if (!checkVisitedPoints(points,corr.visited)) {
//        	feedback += "FAILED. \nA mismatch was detected in nodes visited by the search.\n";
//        	feedback += printVisitedPoints(points,corr.visited);
        } else {
            feedback += "PASSED.\n";
            feedback += "Visited correct number of nodes.\n";
            //feedback += "Visited correct nodes in correct order.\n";
//        	feedback += printVisitedPoints(points,corr.visited);
            correct++;
        }
    }

    public boolean checkVisitedPoints(List<GeographicPoint> actual, List<GeographicPoint> expected) {
    	for (int i = 0; i < expected.size(); i++) {
    		if (i != actual.size()) {
    			if (!actual.get(i).equals(expected.get(i))) return false;
    		}
    	}
    	return true;
    }
    
    public String printVisitedPoints(List<GeographicPoint> actual,List<GeographicPoint> expected) {
    	String ret = "";
    	int length = Math.max(actual.size(),expected.size());
    	for (int i = 0; i<length; i++) {
    		ret += String.format("%-34s ","Expected: "+((i<expected.size())?expected.get(i).printPoint():""));
    		ret += String.format("%s","Actual: "+((i<actual.size())?actual.get(i).printPoint():""));
    		if ((i>= actual.size()) || (i>=expected.size())) {
    			ret += "*\n";
    		} else {
    			ret += (actual.get(i).equals(expected.get(i)))?"":"*";
    			ret += "\n";
    		}
    	}
    	return ret;
    }
    
    
    /** Print a search path in readable form */
    public String printPath(List<GeographicPoint> path) {
        String ret = "";
        for (GeographicPoint point : path) {
            ret += point + "\n";
        }
        return ret;
    }

    /** Run the grader */
    public void run() {
        feedback = "";

        correct = 0;

        try {
            runTest(1, "map1.txt", "MAP: Straight line (-3 <- -2 <- -1 <- 0 -> 1 -> 2-> 3 ->...)", new GeographicPoint(0, 0), new GeographicPoint(6, 6));

            runTest(2, "map2.txt", "MAP: Example map from the writeup", new GeographicPoint(7, 3), new GeographicPoint(4, -1));

            runTest(3, "map3.txt", "MAP: Right triangle (with a little detour)", new GeographicPoint(0, 0), new GeographicPoint(0, 4));

            runTest(4, "ucsd.map", "UCSD MAP: Intersections around UCSD", new GeographicPoint(32.8709815, -117.2434254), new GeographicPoint(32.8742087, -117.2381344));

            if (correct == TESTS)
                feedback = "All tests passed. Great job!" + feedback;
            else
                feedback = "Some tests failed. Check your code for errors, then try again:" + feedback;

        } catch (Exception e) {
            feedback += "\nError during runtime: " + e;
            e.printStackTrace();
        }
            
        System.out.println(printOutput((double)correct / TESTS, feedback));
    }
}
