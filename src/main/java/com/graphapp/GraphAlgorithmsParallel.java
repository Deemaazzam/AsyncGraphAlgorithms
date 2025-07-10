package com.graphapp;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import com.graphapp.ParallelDijkstraWorker;

public class GraphAlgorithmsParallel {
    private static final int THREAD_COUNT = Runtime.getRuntime().availableProcessors();
    private static final ExecutorService executor = Executors.newFixedThreadPool(THREAD_COUNT);

    // === Dijkstra with CompletableFuture ===
    public static Map<String, Integer> dijkstraParallelMultiQueue(Graph graph, String start)
            throws InterruptedException {
        List<String> vertices = new ArrayList<>(graph.getVertices());
        int n = vertices.size();
        int numThreads = Runtime.getRuntime().availableProcessors();

        MultiQueues multiQueues = new MultiQueues(numThreads);

        int[] distances = new int[n];
        Offer[] offers = new Offer[n];
        ReentrantLock[] distancesLocks = new ReentrantLock[n];
        ReentrantLock[] offersLocks = new ReentrantLock[n];
        AtomicBoolean[] done = new AtomicBoolean[numThreads];

        // Create a map from vertex name to its index for quick lookup
        Map<String, Integer> vertexIndices = new HashMap<>();
        for (int i = 0; i < n; i++) {
            vertexIndices.put(vertices.get(i), i);
        }

        for (int i = 0; i < n; i++) {
            distances[i] = Integer.MAX_VALUE;
            distancesLocks[i] = new ReentrantLock();
            offersLocks[i] = new ReentrantLock();
            offers[i] = null;
        }

        for (int i = 0; i < numThreads; i++) {
            done[i] = new AtomicBoolean(false);
        }

        int startIndex = vertexIndices.get(start);
        distances[startIndex] = 0;
        multiQueues.insert(vertices.get(startIndex), 0);

        ExecutorService executorService = Executors.newFixedThreadPool(numThreads);

        for (int i = 0; i < numThreads; i++) {
            executorService.submit(new ParallelDijkstraWorker(
                    multiQueues, graph, distances, distancesLocks, offersLocks, offers, done, i, numThreads,
                    vertexIndices));
        }

        executorService.shutdown();
        executorService.awaitTermination(1, TimeUnit.HOURS);

        Map<String, Integer> distMap = new ConcurrentHashMap<>();
        for (int i = 0; i < n; i++) {
            distMap.put(vertices.get(i), distances[i]);
        }

        return distMap;
    }

    // === BFS with CompletableFuture ===
    public static List<String> bfs(Graph graph, String start) {
        List<String> visited = Collections.synchronizedList(new ArrayList<>());
        Set<String> seen = ConcurrentHashMap.newKeySet();
        Queue<String> queue = new ConcurrentLinkedQueue<>();

        queue.add(start);
        seen.add(start);

        while (!queue.isEmpty()) {
            String current = queue.poll();
            visited.add(current);

            List<CompletableFuture<Void>> futures = new ArrayList<>();
            for (Graph.Edge edge : graph.getNeighbors(current)) {
                CompletableFuture<Void> future = CompletableFuture.runAsync(() -> {
                    if (seen.add(edge.to)) { // true if newly added
                        queue.add(edge.to);
                    }
                }, executor);
                futures.add(future);
            }

            CompletableFuture.allOf(futures.toArray(new CompletableFuture[0])).join();
        }
        return visited;
    }

    // === DFS with CompletableFuture ===
    public static List<String> dfs(Graph graph, String start) {
        List<String> visited = Collections.synchronizedList(new ArrayList<>());
        Set<String> visitedSet = ConcurrentHashMap.newKeySet();
        dfsHelperParallel(graph, start, visited, visitedSet).join();
        return visited;
    }

    private static CompletableFuture<Void> dfsHelperParallel(Graph graph, String v, List<String> visited,
            Set<String> visitedSet) {
        if (!visitedSet.add(v))
            return CompletableFuture.completedFuture(null);

        visited.add(v);

        List<CompletableFuture<Void>> futures = new ArrayList<>();
        for (Graph.Edge e : graph.getNeighbors(v)) {
            if (!visitedSet.contains(e.to)) {
                futures.add(CompletableFuture.runAsync(() -> dfsHelperParallel(graph, e.to, visited, visitedSet).join(),
                        executor));
            }
        }
        return CompletableFuture.allOf(futures.toArray(new CompletableFuture[0]));
    }

    // === Connected Components (undirected only) with CompletableFuture ===
    public static List<List<String>> connectedComponents(Graph graph) {
        if (graph.isDirected())
            throw new IllegalArgumentException("Connected components require undirected graph.");

        List<List<String>> components = Collections.synchronizedList(new ArrayList<>());
        Set<String> visited = ConcurrentHashMap.newKeySet();

        List<CompletableFuture<Void>> futures = new ArrayList<>();
        for (String v : graph.getVertices()) {
            if (!visited.contains(v)) {
                futures.add(CompletableFuture.runAsync(() -> {
                    List<String> component = Collections.synchronizedList(new ArrayList<>());
                    dfsHelper(graph, v, component, visited);
                    components.add(component);
                }, executor));
            }
        }
        CompletableFuture.allOf(futures.toArray(new CompletableFuture[0])).join();
        return components;
    }

    private static void dfsHelper(Graph graph, String v, List<String> component, Set<String> visited) {
        if (!visited.add(v))
            return;
        component.add(v);
        for (Graph.Edge e : graph.getNeighbors(v)) {
            dfsHelper(graph, e.to, component, visited);
        }
    }

    // === Cycle Detection (undirected only) with CompletableFuture ===
    public static boolean hasCycle(Graph graph) {
        if (graph.isDirected())
            throw new IllegalArgumentException("Cycle detection here assumes undirected graph.");

        Set<String> visited = ConcurrentHashMap.newKeySet();
        AtomicBoolean cycleFound = new AtomicBoolean(false);

        List<CompletableFuture<Void>> futures = new ArrayList<>();
        for (String v : graph.getVertices()) {
            if (!visited.contains(v)) {
                futures.add(CompletableFuture.runAsync(() -> {
                    if (hasCycleDFS(graph, v, null, visited)) {
                        cycleFound.set(true);
                    }
                }, executor));
            }
        }
        CompletableFuture.allOf(futures.toArray(new CompletableFuture[0])).join();
        return cycleFound.get();
    }

    private static boolean hasCycleDFS(Graph graph, String current, String parent, Set<String> visited) {
        if (!visited.add(current))
            return false;

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

    // === Topological Sort with CompletableFuture ===
    public static List<String> topologicalSort(Graph graph) {
        if (!graph.isDirected())
            throw new IllegalArgumentException("Topological sort requires directed graph");

        Map<String, Integer> inDegree = new ConcurrentHashMap<>();
        for (String v : graph.getVertices())
            inDegree.put(v, 0);
        for (String from : graph.getVertices()) {
            for (Graph.Edge e : graph.getNeighbors(from)) {
                inDegree.compute(e.to, (key, val) -> val == null ? 1 : val + 1);
            }
        }

        Queue<String> queue = new ConcurrentLinkedQueue<>();
        for (String v : inDegree.keySet()) {
            if (inDegree.get(v) == 0)
                queue.add(v);
        }

        List<String> sorted = Collections.synchronizedList(new ArrayList<>());
        while (!queue.isEmpty()) {
            String node = queue.poll();
            sorted.add(node);

            List<CompletableFuture<Void>> futures = new ArrayList<>();
            for (Graph.Edge e : graph.getNeighbors(node)) {
                futures.add(CompletableFuture.runAsync(() -> {
                    inDegree.computeIfPresent(e.to, (key, val) -> val - 1);
                    if (inDegree.get(e.to) == 0) {
                        queue.add(e.to);
                    }
                }, executor));
            }
            CompletableFuture.allOf(futures.toArray(new CompletableFuture[0])).join();
        }

        return sorted.size() == graph.getVertices().size() ? sorted : null; // null means cycle detected
    }

    // === Floyd-Warshall with CompletableFuture ===
    public static Map<String, Map<String, Integer>> floydWarshall(Graph graph) {
        List<String> vertices = new ArrayList<>(graph.getVertices());
        int n = vertices.size();
        Map<String, Map<String, Integer>> dist = new ConcurrentHashMap<>();

        // Init
        for (String v1 : vertices) {
            Map<String, Integer> row = new ConcurrentHashMap<>();
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

        // Floyd-Warshall parallelized on i-loop
        for (String k : vertices) {
            List<CompletableFuture<Void>> futures = new ArrayList<>();
            for (String i : vertices) {
                futures.add(CompletableFuture.runAsync(() -> {
                    for (String j : vertices) {
                        int ik = dist.get(i).get(k);
                        int kj = dist.get(k).get(j);
                        if (ik != Integer.MAX_VALUE && kj != Integer.MAX_VALUE) {
                            dist.get(i).compute(j, (key, oldVal) -> Math.min(oldVal, ik + kj));
                        }
                    }
                }, executor));
            }
            CompletableFuture.allOf(futures.toArray(new CompletableFuture[0])).join();
        }
        return dist;
    }

    // === Kruskal's MST (undirected only) ===
    public static List<Graph.EdgeRecord> kruskalMST(Graph graph) {
        if (graph.isDirected())
            throw new IllegalArgumentException("Kruskal's MST works only on undirected graphs.");

        List<Graph.EdgeRecord> edges = new ArrayList<>(graph.getAllEdges());
        edges.sort(Comparator.comparingInt(e -> e.weight));

        Map<String, String> parent = new ConcurrentHashMap<>();
        for (String v : graph.getVertices())
            parent.put(v, v);

        List<Graph.EdgeRecord> mst = Collections.synchronizedList(new ArrayList<>());

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

    private static class Node implements Comparable<Node> {
        String name;
        int distance;

        Node(String name, int distance) {
            this.name = name;
            this.distance = distance;
        }

        @Override
        public int compareTo(Node other) {
            return Integer.compare(this.distance, other.distance);
        }
    }

    public static void shutdownExecutor() {
        executor.shutdown();
    }
}
