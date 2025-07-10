package com.graphapp;

import java.util.*;

public class GraphAlgorithms {

    // === Dijkstra ===
    public static Map<String, Integer> dijkstra(Graph graph, String start) {
        Map<String, Integer> distances = new HashMap<>();
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));
        Set<String> visited = new HashSet<>();

        for (String v : graph.getVertices()) {
            distances.put(v, Integer.MAX_VALUE);
        }
        distances.put(start, 0);
        pq.add(new Node(start, 0));

        while (!pq.isEmpty()) {
            Node current = pq.poll();
            if (!visited.add(current.name))
                continue;

            for (Graph.Edge edge : graph.getNeighbors(current.name)) {
                if (visited.contains(edge.to))
                    continue;

                int newDist = distances.get(current.name) + edge.weight;
                if (newDist < distances.get(edge.to)) {
                    distances.put(edge.to, newDist);
                    pq.add(new Node(edge.to, newDist));
                }
            }
        }
        return distances;
    }

    // === BFS ===
    public static List<String> bfs(Graph graph, String start) {
        List<String> visited = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        Queue<String> queue = new LinkedList<>();

        queue.add(start);
        seen.add(start);

        while (!queue.isEmpty()) {
            String current = queue.poll();
            visited.add(current);
            for (Graph.Edge edge : graph.getNeighbors(current)) {
                if (!seen.contains(edge.to)) {
                    seen.add(edge.to);
                    queue.add(edge.to);
                }
            }
        }
        return visited;
    }

    // === DFS ===
    public static List<String> dfs(Graph graph, String start) {
        List<String> visited = new ArrayList<>();
        Set<String> visitedSet = new HashSet<>();
        dfsHelper(graph, start, visited, visitedSet);
        return visited;
    }

    private static void dfsHelper(Graph graph, String v, List<String> visited, Set<String> visitedSet) {
        visited.add(v);
        visitedSet.add(v);
        for (Graph.Edge e : graph.getNeighbors(v)) {
            if (!visitedSet.contains(e.to)) {
                dfsHelper(graph, e.to, visited, visitedSet);
            }
        }
    }

    // === Connected Components (undirected only) ===
    public static List<List<String>> connectedComponents(Graph graph) {
        if (graph.isDirected())
            throw new IllegalArgumentException("Connected components require undirected graph.");
        List<List<String>> components = new ArrayList<>();
        Set<String> visited = new HashSet<>();

        for (String v : graph.getVertices()) {
            if (!visited.contains(v)) {
                List<String> component = new ArrayList<>();
                dfsHelper(graph, v, component, visited);
                components.add(component);
            }
        }
        return components;
    }

    // === Cycle Detection (undirected only) ===
    public static boolean hasCycle(Graph graph) {
        if (graph.isDirected())
            throw new IllegalArgumentException("Cycle detection here assumes undirected graph.");

        Set<String> visited = new HashSet<>();
        for (String v : graph.getVertices()) {
            if (!visited.contains(v) && hasCycleDFS(graph, v, null, visited)) {
                return true;
            }
        }
        return false;
    }

    private static boolean hasCycleDFS(Graph graph, String current, String parent, Set<String> visited) {
        visited.add(current);
        for (Graph.Edge e : graph.getNeighbors(current)) {
            if (!visited.contains(e.to)) {
                if (hasCycleDFS(graph, e.to, current, visited))
                    return true;
            } else if (!e.to.equals(parent)) {
                return true;
            }
        }
        return false;
    }

    // === Topological Sort ===
    public static List<String> topologicalSort(Graph graph) {
        if (!graph.isDirected())
            throw new IllegalArgumentException("Topological sort requires directed graph");

        Map<String, Integer> inDegree = new HashMap<>();
        for (String v : graph.getVertices())
            inDegree.put(v, 0);
        for (String from : graph.getVertices()) {
            for (Graph.Edge e : graph.getNeighbors(from)) {
                inDegree.put(e.to, inDegree.get(e.to) + 1);
            }
        }

        Queue<String> queue = new LinkedList<>();
        for (String v : inDegree.keySet()) {
            if (inDegree.get(v) == 0)
                queue.add(v);
        }

        List<String> sorted = new ArrayList<>();
        while (!queue.isEmpty()) {
            String node = queue.poll();
            sorted.add(node);
            for (Graph.Edge e : graph.getNeighbors(node)) {
                inDegree.put(e.to, inDegree.get(e.to) - 1);
                if (inDegree.get(e.to) == 0)
                    queue.add(e.to);
            }
        }

        return sorted.size() == graph.getVertices().size() ? sorted : null; // null = cycle detected
    }

    // === Floyd-Warshall ===
    public static Map<String, Map<String, Integer>> floydWarshall(Graph graph) {
        List<String> vertices = new ArrayList<>(graph.getVertices());
        int n = vertices.size();
        Map<String, Map<String, Integer>> dist = new HashMap<>();

        // Init
        for (String v1 : vertices) {
            Map<String, Integer> row = new HashMap<>();
            for (String v2 : vertices) {
                row.put(v2, v1.equals(v2) ? 0 : Integer.MAX_VALUE);
            }
            dist.put(v1, row);
        }

        // Add edges
        for (String from : graph.getVertices()) {
            for (Graph.Edge edge : graph.getNeighbors(from)) {
                dist.get(from).put(edge.to, edge.weight);
            }
        }

        // Floyd-Warshall
        for (String k : vertices) {
            for (String i : vertices) {
                for (String j : vertices) {
                    int ik = dist.get(i).get(k);
                    int kj = dist.get(k).get(j);
                    if (ik != Integer.MAX_VALUE && kj != Integer.MAX_VALUE) {
                        dist.get(i).put(j, Math.min(dist.get(i).get(j), ik + kj));
                    }
                }
            }
        }
        return dist;
    }

    // === Kruskal's MST (undirected only) ===
    public static List<Graph.EdgeRecord> kruskalMST(Graph graph) {
        if (graph.isDirected())
            throw new IllegalArgumentException("Kruskal's MST works only on undirected graphs.");

        List<Graph.EdgeRecord> edges = graph.getAllEdges();
        edges.sort(Comparator.comparingInt(e -> e.weight));

        Map<String, String> parent = new HashMap<>();
        for (String v : graph.getVertices())
            parent.put(v, v);

        List<Graph.EdgeRecord> mst = new ArrayList<>();

        for (Graph.EdgeRecord e : edges) {
            String root1 = find(parent, e.from);
            String root2 = find(parent, e.to);
            if (!root1.equals(root2)) {
                mst.add(e);
                parent.put(root1, root2);
            }
        }

        return mst;
    }

    private static String find(Map<String, String> parent, String v) {
        if (!parent.get(v).equals(v)) {
            parent.put(v, find(parent, parent.get(v)));
        }
        return parent.get(v);
    }

    private static class Node {
        String name;
        int distance;

        Node(String name, int distance) {
            this.name = name;
            this.distance = distance;
        }
    }
}