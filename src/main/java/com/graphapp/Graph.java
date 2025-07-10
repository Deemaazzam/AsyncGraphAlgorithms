package com.graphapp;

import java.util.*;

public class Graph {
    private final Map<String, List<Edge>> adjList = new HashMap<>();
    private boolean directed;

    public Graph(boolean directed) {
        this.directed = directed;
    }

    // Add a vertex
    public void addVertex(String v) {
        adjList.putIfAbsent(v, new ArrayList<>());
    }

    public void setDirected(boolean is) {
        this.directed = is;
    }

    // Remove a vertex and all edges connected to it
    public void removeVertex(String v) {
        adjList.remove(v);
        for (List<Edge> edges : adjList.values()) {
            edges.removeIf(e -> e.to.equals(v));
        }
    }

    public boolean containsVertex(String v) {
        return adjList.containsKey(v);
    }

    // Add an edge with default weight 1
    public void addEdge(String from, String to) {
        addEdge(from, to, 1);
    }

    // Add an edge with weight (uses internal directed flag)
    public void addEdge(String from, String to, int weight) {
        addVertex(from);
        addVertex(to);
        adjList.get(from).add(new Edge(to, weight));
        if (!directed) {
            adjList.get(to).add(new Edge(from, weight));
        }
    }

    // Add edge with explicit direction
    public void addEdge(String from, String to, int weight, boolean isDirected) {
        addVertex(from);
        addVertex(to);
        adjList.get(from).add(new Edge(to, weight));
        if (!isDirected) {
            adjList.get(to).add(new Edge(from, weight));
        }
    }

    // Remove edge (uses internal directed flag)
    public void removeEdge(String from, String to) {
        List<Edge> list = adjList.get(from);
        if (list != null) {
            list.removeIf(e -> e.to.equals(to));
        }
        if (!directed) {
            List<Edge> reverse = adjList.get(to);
            if (reverse != null) {
                reverse.removeIf(e -> e.to.equals(from));
            }
        }
    }

    // Remove edge with explicit direction
    public void removeEdge(String from, String to, boolean isDirected) {
        List<Edge> list = adjList.get(from);
        if (list != null) {
            list.removeIf(e -> e.to.equals(to));
        }
        if (!isDirected) {
            List<Edge> reverse = adjList.get(to);
            if (reverse != null) {
                reverse.removeIf(e -> e.to.equals(from));
            }
        }
    }

    // Get vertices
    public Set<String> getVertices() {
        return adjList.keySet();
    }

    // Get neighbors of a vertex
    public List<Edge> getNeighbors(String v) {
        return adjList.getOrDefault(v, Collections.emptyList());
    }

    public boolean isDirected() {
        return directed;
    }

    // Return all edges
    public List<EdgeRecord> getAllEdges() {
        List<EdgeRecord> edges = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        for (String from : adjList.keySet()) {
            for (Edge e : adjList.get(from)) {
                String to = e.to;
                String key = from + "-" + to;
                if (directed || !seen.contains(key)) {
                    edges.add(new EdgeRecord(from, to, e.weight));
                    if (!directed) {
                        seen.add(to + "-" + from);
                    }
                }
            }
        }
        return edges;
    }

    public static class Edge {
        public final String to;
        public final int weight;

        public Edge(String to, int weight) {
            this.to = to;
            this.weight = weight;
        }
    }

    public static class EdgeRecord {
        public final String from;
        public final String to;
        public final int weight;

        public EdgeRecord(String from, String to, int weight) {
            this.from = from;
            this.to = to;
            this.weight = weight;
        }

        @Override
        public String toString() {
            return String.format("%s %s %s [%d]", from, "->", to, weight);
        }
    }
}
