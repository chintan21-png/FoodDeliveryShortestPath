import java.util.*;

class Graph {
    // Public variables to represent the number of locations (nodes) and adjacency list
    public int vertices; // Number of locations (nodes)
    public List<List<Node>> adjList; // Adjacency list for graph representation

    // Constructor
    public Graph(int vertices) {
        this.vertices = vertices;
        adjList = new ArrayList<>(vertices); // Initialize adjacency list
        for (int i = 0; i < vertices; i++) {
            adjList.add(new ArrayList<>()); // Create an empty list for each vertex
        }
    }

    // Add an edge between two locations
    public void addEdge(int source, int destination, int weight) {
        adjList.get(source).add(new Node(destination, weight));
        adjList.get(destination).add(new Node(source, weight)); // Undirected graph
    }

    // Dijkstra's Algorithm for Shortest Path
    public void dijkstra(int source, int destination) {
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.weight));
        int[] distance = new int[vertices];
        int[] parent = new int[vertices]; // To store the path

        // Initialize distances to infinity
        Arrays.fill(distance, Integer.MAX_VALUE);
        Arrays.fill(parent, -1);
        distance[source] = 0;

        pq.offer(new Node(source, 0));

        while (!pq.isEmpty()) {
            Node current = pq.poll();
            int u = current.vertex;

            for (Node neighbor : adjList.get(u)) {
                int v = neighbor.vertex;
                int weight = neighbor.weight;

                // If a shorter path is found
                if (distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                    parent[v] = u;
                    pq.offer(new Node(v, distance[v]));
                }
            }
        }

        // Print the shortest path
        printPath(source, destination, distance, parent);
    }

    // Print the shortest path
    private void printPath(int source, int destination, int[] distance, int[] parent) {
        if (distance[destination] == Integer.MAX_VALUE) {
            System.out.println("No path found from Restaurant to Customer.");
            return;
        }

        System.out.println("Shortest Delivery Path Distance: " + distance[destination] + " km");

        List<Integer> path = new ArrayList<>();
        for (int at = destination; at != -1; at = parent[at]) {
            path.add(at);
        }
        Collections.reverse(path);

        System.out.print("Optimal Delivery Route: ");
        for (int i = 0; i < path.size(); i++) {
            System.out.print("Location " + path.get(i));
            if (i < path.size() - 1) System.out.print(" -> ");
        }
        System.out.println();
    }

    // Node class representing a location (vertex) in the graph
    static class Node {
        int vertex, weight;

        Node(int vertex, int weight) {
            this.vertex = vertex;
            this.weight = weight;
        }
    }
}

// Main Class
public class FoodDeliveryShortestPath {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        System.out.println("Enter number of locations (nodes): ");
        int locations = scanner.nextInt();
        Graph graph = new Graph(locations);

        System.out.println("Enter number of roads (edges): ");
        int roads = scanner.nextInt();

        System.out.println("Enter edges (source destination distance): ");
        for (int i = 0; i < roads; i++) {
            int source = scanner.nextInt();
            int destination = scanner.nextInt();
            int distance = scanner.nextInt();
            graph.addEdge(source, destination, distance);
        }

        System.out.println("Enter restaurant location (start node): ");
        int restaurant = scanner.nextInt();

        System.out.println("Enter customer location (destination node): ");
        int customer = scanner.nextInt();

        graph.dijkstra(restaurant, customer);
    }
}
