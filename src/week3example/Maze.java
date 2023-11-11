/**
 * A class that represents a maze to navigate through.
 */
package week3example;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Queue;
import java.util.Stack;

// TODO: Auto-generated Javadoc
/**
 * A class that represents a 2D maze, represented using a graph.  
 * 
 * @author UCSD Intermediate Programming MOOC Team
 *
 */
public class Maze {
	
	/** The cells. */
	private MazeNode[][] cells;
	
	/** The width. */
	private int width;
	
	/** The height. */
	private int height;

	/** The default size. */
	private final int DEFAULT_SIZE = 10;

	/**
	 *  
	 * Create a new empty maze with default size 10x10.
	 */
	public Maze() {

		cells = new MazeNode[DEFAULT_SIZE][DEFAULT_SIZE];
		this.width = DEFAULT_SIZE;
		this.height = DEFAULT_SIZE;
	}

	/**
	 *  
	 * Create a new empty Maze with specified height and width.
	 *
	 * @param width the width
	 * @param height the height
	 */
	public Maze(int width, int height) {
		cells = new MazeNode[height][width];
		this.width = width;
		this.height = height;
	}

	/**
	 * Reset the maze to have the given height and width.
	 *
	 * @param width The width of the maze
	 * @param height The height of the maze
	 */
	public void initialize(int width, int height) {
		cells = new MazeNode[height][width];
		this.width = width;
		this.height = height;

	}

	/**
	 * Add a graph node (i.e. not a wall) at the given location.
	 * Any grid entry that doesn't contain a node is interpreted as a wall.
	 * @param row  The row where the node exists
	 * @param col  The column where the node exists
	 */
	public void addNode(int row, int col) {
		cells[row][col] = new MazeNode(row, col);
	}

	/**
	 * Link the nodes that are adjacent (and not null) to each other with an
	 * edge. There is an edge between any two adjacent nodes up, down, left or
	 * right. This is the code that defines the search order (up, left, down, right)!
	 */
	public void linkEdges() {
		int numRows = cells.length;
		for (int row = 0; row < numRows; row++) {
			int numCols = cells[row].length;
			for (int col = 0; col < numCols; col++) {
				if (cells[row][col] != null) {
					if (row > 0 && cells[row - 1][col] != null) {
						cells[row][col].addNeighbor(cells[row - 1][col]);
					}
					if (col > 0 && cells[row][col - 1] != null) {
						cells[row][col].addNeighbor(cells[row][col - 1]);
					}
					if (row < numRows - 1 && cells[row + 1][col] != null) {
						cells[row][col].addNeighbor(cells[row + 1][col]);
					}
					if (col < numCols - 1 && cells[row][col + 1] != null) {
						cells[row][col].addNeighbor(cells[row][col + 1]);
					}
				}
			}
		}
	}

	/**
	 * Print the maze grid to the screen.
	 */
	public void printMaze() {
		for (int r = 0; r < height; r++) {
			for (int c = 0; c < width; c++) {
				if (cells[r][c] == null) {
					System.out.print('*');
				} else {
					System.out.print(cells[r][c].getDisplayChar());
				}
			}
			System.out.print("\n");
		}

	}

	/**
	 * Change the display of the maze so that it will print the 
	 * path found from start to goal.
	 * 
	 * NOTE: This method could use redesigning so that it did not expose
	 * the MazeNode class to the outside world.
	 * 
	 * @param path A path of MazeNodes from start to goal.
	 */
	public void setPath(List<MazeNode> path) {
		int index = 0;
		for (MazeNode n : path) {
			if (index == 0) {
				n.setDisplayChar(MazeNode.START);
			} else if (index == path.size() - 1) {
				n.setDisplayChar(MazeNode.GOAL);
			} else {
				n.setDisplayChar(MazeNode.PATH);
			}
			index++;
		}

	}

	/**
	 * Clear (reset) the maze so that it will not disply a path
	 * from start to goal.
	 */
	public void clearPath() {
		for (int r = 0; r < cells.length; r++) {
			for (int c = 0; c < cells[r].length; c++) {
				MazeNode n = cells[r][c];
				if (n != null) {
					n.setDisplayChar(MazeNode.EMPTY);
				}
			}
		}
	}
	
	/**
	 * Check that start and goal maze nodes are valid (not null).
	 *
	 * @param start the start
	 * @param goal the goal
	 * @return true, if successful
	 */
	private boolean checkStartGoal(MazeNode start, MazeNode goal) {
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return false;
		}
		return true;
	}
	
	/**
	 * Reconstructs the path from start -> goal using the parentMap and
	 * working backwards
	 *
	 * @param start the start node
	 * @param goal the goal node
	 * @param parentMap the parent map that links a node to its "parent"
	 * @return path the path from start to goal, represented as a linked list of MazeNodes.
	 */
	private LinkedList<MazeNode> reconstructPath(MazeNode start, MazeNode goal, HashMap<MazeNode,MazeNode> parentMap) {
		// reconstruct the path
		LinkedList<MazeNode> path = new LinkedList<MazeNode>();
		MazeNode curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;		
	}
	
	/**
	 * Helper method to execute the DFS Search algorithm. Searches the graph
	 * until either the toExplore Stack is empty OR a path is found. Note that 
	 * updates to visited
	 *
	 * @param start the start
	 * @param goal the goal
	 * @param parentMap the HashMap that relates each node to its predecessor (or parent)
	 * @return true, if path from start -> goal was found
	 */
	private boolean dfsSearch(MazeNode start, MazeNode goal, HashMap<MazeNode,MazeNode> parentMap) {
		HashSet<MazeNode> visited = new HashSet<MazeNode>();
		Stack<MazeNode> toExplore = new Stack<MazeNode>();

		toExplore.push(start);
		visited.add(start);
		boolean found = false;

		// Do the search
		while (!toExplore.empty()) {
			MazeNode curr = toExplore.pop();
			found = (curr == goal);
			if (found) break;
			List<MazeNode> neighbors = curr.getNeighbors();
			
			// Make sure you understand how the iterator is being used here 
			// to meet the DFS implementation requirements
			ListIterator<MazeNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MazeNode next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.push(next);
				}
			}
		}
				
		return found;
	}

	/** depth first search from (startRow,startCol) to (endRow,endCol)
	 *  
	 * @param startRow  The row of the starting position
	 * @param startCol  The column of the starting position
	 * @param endRow The row of the end position
	 * @param endCol The column of the end position
	 * @return the path from starting position to ending position, or
	 * an empty list if there is no path.
	 */
	public List<MazeNode> dfs(int startRow, int startCol, int endRow, int endCol) {
		
		// Initialize everything
		MazeNode start = cells[startRow][startCol];
		MazeNode goal = cells[endRow][endCol];
		LinkedList<MazeNode> path = new LinkedList<MazeNode>();

		if (!checkStartGoal(start,goal)) return path;

		HashMap<MazeNode, MazeNode> parentMap = new HashMap<MazeNode, MazeNode>();	

		boolean found = dfsSearch(start, goal,parentMap);

		if (!found) {
			System.out.println("No path exists");
		} else {
			path = reconstructPath(start, goal, parentMap);
		}
		return path;

	}

	/**
	 * Helper method to execute the BFS Search algorithm. Searches the graph
	 * until either the toExplore Stack is empty OR a path is found. Note that 
	 * updates to visited
	 *
	 * @param start the start
	 * @param goal the goal
	 * @param parentMap the HashMap that relates each node to its predecessor (or parent)
	 * @return true, if path from start -> goal was found
	 */
	private boolean bfsSearch(MazeNode start, MazeNode goal, HashMap<MazeNode,MazeNode> parentMap) {
		HashSet<MazeNode> visited = new HashSet<MazeNode>();
		Queue<MazeNode> toExplore = new LinkedList<MazeNode>();
		toExplore.add(start);
		visited.add(start);
		boolean found = false;

		// Do the search
		while (!toExplore.isEmpty()) {
			MazeNode curr = toExplore.poll();
			found = (curr == goal);
			if (found) break;
			List<MazeNode> neighbors = curr.getNeighbors();
			for (MazeNode next : neighbors) {
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}

		return found;
	}

	/** breadth first search from (startRow,startCol) to (endRow,endCol)
	 * 
	 * 
	 * @param startRow  The row of the starting position
	 * @param startCol  The column of the starting position
	 * @param endRow The row of the end position
	 * @param endCol The column of the end position
	 * @return the path from starting position to ending position, or
	 * an empty list if there is no path.
	 */
	public List<MazeNode> bfs(int startRow, int startCol, int endRow, int endCol) {
		MazeNode start = cells[startRow][startCol];
		MazeNode goal = cells[endRow][endCol];
		LinkedList<MazeNode> path = new LinkedList<MazeNode>();

		if (!checkStartGoal(start,goal)) return path;

		HashMap<MazeNode, MazeNode> parentMap = new HashMap<MazeNode, MazeNode>();
		
		boolean found = bfsSearch(start, goal,parentMap);

		if (!found) {
			System.out.println("No path exists");
		} else {
			path = reconstructPath(start, goal, parentMap);
		}
		return path;
}

	/**
	 * The main method.
	 *
	 * @param args the arguments
	 */
	public static void main(String[] args) {
		String mazeFile = "";
		Maze maze;
		List<MazeNode> path;
		
		mazeFile = "data/mazes/maze1.maze";
		maze = new Maze();
		MazeLoader.loadMaze(mazeFile, maze);
		System.out.println("\nMaze:\n");
		maze.printMaze();
		path = maze.dfs(3, 3, 2, 0);
		maze.setPath(path);
		System.out.println("\nDFS Search:\n");
		maze.printMaze();
		maze.clearPath();
		maze.setPath(maze.bfs(3, 3, 2, 0));
		System.out.println("\nBFS Search:\n");
		maze.printMaze();
	}
}




