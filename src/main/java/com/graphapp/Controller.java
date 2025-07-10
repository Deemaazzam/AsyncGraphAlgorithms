package com.graphapp;

import javafx.fxml.FXML;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.*;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import java.util.*;
import java.lang.management.ManagementFactory;
import com.sun.management.OperatingSystemMXBean;

public class Controller {

    // FXML fields
    @FXML
    private MenuButton addVertexMenu;
    @FXML
    private Canvas canvas;
    @FXML
    private RadioMenuItem labelTypeLetters, labelTypeNumbers;
    @FXML
    private Button btnAddEdge;
    @FXML
    private Button btnLarge;
    @FXML
    private MenuButton AlgorithmsMenu;
    @FXML
    private RadioMenuItem Dijsktra, BFS, DFS, FloydWarshall, ConnectedComponents, CycleDetection, TopologicalSort,
            MinimumSpanningTree;
    @FXML
    private Label instructionLabel;
    @FXML
    private TextArea graphDataArea;

    // Graph state
    private final List<Vertex> vertices = new ArrayList<>();
    private final List<EdgeRecord> edges = new ArrayList<>();
    private final ToggleGroup labelTypeGroup = new ToggleGroup();
    private final Deque<Runnable> undoStack = new ArrayDeque<>();
    private final Graph graph = new Graph(false); // Start undirected; will update dynamically

    private GraphicsContext gc;
    private boolean addVertexMode = false, addEdgeMode = false;
    private Vertex firstVertex = null;
    private String selectedAlgorithm = null;
    private boolean waitingForStartVertex = false;
    private int vertexCount = 0;

    @FXML
    public void initialize() {
        gc = canvas.getGraphicsContext2D();
        labelTypeLetters.setToggleGroup(labelTypeGroup);
        labelTypeNumbers.setToggleGroup(labelTypeGroup);
        labelTypeLetters.setSelected(true);

        // Vertex and edge creation
        addVertexMenu.setOnShowing(e -> addVertexMode = true);
        btnAddEdge.setOnAction(e -> {
            addEdgeMode = true;
            firstVertex = null;
            instructionLabel.setText("Click first vertex to start edge.");
        });

        // Add algorithm menu actions
        Dijsktra.setOnAction(e -> prepareAlgorithm("Dijkstra"));
        BFS.setOnAction(e -> prepareAlgorithm("BFS"));
        DFS.setOnAction(e -> prepareAlgorithm("DFS"));
        FloydWarshall.setOnAction(e -> runAlgorithm("Floyd-Warshall", null));
        ConnectedComponents.setOnAction(e -> runAlgorithm("Connected Components", null));
        CycleDetection.setOnAction(e -> runAlgorithm("Cycle Detection", null));
        TopologicalSort.setOnAction(e -> runAlgorithm("Topological Sort", null));
        MinimumSpanningTree.setOnAction(e -> runAlgorithm("Kruskal", null));
        btnLarge.setOnAction(e -> handleLargeGraph());

        // Input handling
        canvas.setFocusTraversable(true);
        canvas.addEventFilter(KeyEvent.KEY_PRESSED, this::handleKeyPress);
        canvas.addEventHandler(MouseEvent.MOUSE_CLICKED, this::handleCanvasClick);

        redraw();
    }

    private void handleLargeGraph() {
        TextInputDialog dialog = new TextInputDialog();
        dialog.setTitle("Run Large Graph Algorithm");
        dialog.setHeaderText("Enter start and end node IDs separated by a space (e.g., 12 56789):");

        Optional<String> result = dialog.showAndWait();
        result.ifPresent(input -> {
            try {
                String[] parts = input.trim().split("\\s+");
                int start = Integer.parseInt(parts[0]);
                int end = Integer.parseInt(parts[1]);

                // Load graph if needed
                Graph largeGraph = GraphLoader.loadDirectedGraphFromFile("Email-EuAll.txt"); // You'll need to write
                                                                                             // this class

                // Run sequential
                long startTime = System.nanoTime();
                Map<String, Integer> seqResult = GraphAlgorithms.dijkstra(largeGraph, String.valueOf(start));
                long seqTime = System.nanoTime() - startTime;

                // Run parallel
                long parStartTime = System.nanoTime();
                Map<String, Integer> parResult = GraphAlgorithmsParallel.dijkstraParallelMultiQueue(largeGraph,
                        String.valueOf(start));
                long parTime = System.nanoTime() - parStartTime;

                int seqDist = seqResult.getOrDefault(String.valueOf(end), -1);
                int parDist = parResult.getOrDefault(String.valueOf(end), -1);

                showResults(start, end, seqDist, parDist, seqTime, parTime);

            } catch (Exception ex) {
                ex.printStackTrace();
                instructionLabel.setText("Invalid input or error running algorithm.");
            }
        });
    }

    private void showResults(int start, int end, int seqDist, int parDist, long seqTime, long parTime) {
        Alert alert = new Alert(Alert.AlertType.INFORMATION);
        alert.setTitle("Algorithm Results");
        alert.setHeaderText("Shortest Path from " + start + " to " + end);
        alert.setContentText(
                "Sequential Dijkstra:\n" +
                        "  Distance: " + seqDist + "\n" +
                        "  Time: " + seqTime + " ns\n\n" +
                        "Parallel Dijkstra:\n" +
                        "  Distance: " + parDist + "\n" +
                        "  Time: " + parTime + " ns\n\n" +
                        "Speed-up: ~" + String.format("%.2f", (double) seqTime / parTime) + "x");
        alert.showAndWait();
    }

    private String getCpuUsage() {
        OperatingSystemMXBean osBean = ManagementFactory.getPlatformMXBean(OperatingSystemMXBean.class);
        double cpuLoad = osBean.getSystemCpuLoad();
        return cpuLoad < 0 ? "Unavailable" : String.format("%.2f%%", cpuLoad * 100);
    }

    private boolean isGraphDirected() {
        return edges.stream().anyMatch(e -> e.directed);
    }

    private void prepareAlgorithm(String algorithm) {
        selectedAlgorithm = algorithm;
        waitingForStartVertex = true;
        instructionLabel.setText("Click on a start vertex for " + algorithm + ".");
    }

    private void handleCanvasClick(MouseEvent event) {
        double x = event.getX(), y = event.getY();

        // Algorithm start vertex click
        if (waitingForStartVertex && event.getButton() == MouseButton.PRIMARY) {
            Vertex clicked = getVertexAt(x, y);
            if (clicked != null) {
                runAlgorithm(selectedAlgorithm, clicked);
                waitingForStartVertex = false;
                selectedAlgorithm = null;
                return;
            } else {
                instructionLabel.setText("Please click on a valid vertex.");
                return;
            }
        }

        // Right-click delete vertex
        if (event.getButton() == MouseButton.SECONDARY) {
            Vertex clicked = getVertexAt(x, y);
            if (clicked != null) {
                removeVertex(clicked);
                redraw();
                return;
            }
        }
        // Middle-click delete edge
        else if (event.getButton() == MouseButton.MIDDLE) {
            removeEdgeAt(x, y);
            return;
        }

        // Add vertex
        if (addVertexMode) {
            String label = generateLabel();
            Vertex v = new Vertex(x, y, label);
            vertices.add(v);
            graph.addVertex(label);
            vertexCount++;
            undoStack.push(() -> {
                vertices.remove(v);
                graph.removeVertex(label);
            });
            addVertexMode = false;
            redraw();
            instructionLabel.setText("Vertex added.");
        }
        // Add edge
        else if (addEdgeMode) {
            Vertex clicked = getVertexAt(x, y);
            if (clicked == null) {
                instructionLabel.setText("Canceled edge creation.");
                addEdgeMode = false;
                firstVertex = null;
                return;
            }
            if (firstVertex == null) {
                firstVertex = clicked;
                instructionLabel.setText("Click second vertex.");
            } else {
                showEdgeDialog(firstVertex, clicked);
                addEdgeMode = false;
            }
        }
    }

    private String runAlgorithmInternal(String algorithm, Vertex startVertex, boolean isParallel) throws Exception {
        StringBuilder result = new StringBuilder();

        // Use correct class
        var alg = isParallel ? GraphAlgorithmsParallel.class : GraphAlgorithms.class;

        switch (algorithm) {
            case "BFS":
                if (startVertex == null)
                    throw new IllegalArgumentException("Start vertex required.");
                List<String> bfs = (List<String>) alg.getMethod("bfs", Graph.class, String.class)
                        .invoke(null, graph, startVertex.label);
                if (!isParallel)
                    highlightPath(bfs);
                result.append(bfs);
                break;

            case "DFS":
                if (startVertex == null)
                    throw new IllegalArgumentException("Start vertex required.");
                List<String> dfs = (List<String>) alg.getMethod("dfs", Graph.class, String.class)
                        .invoke(null, graph, startVertex.label);
                if (!isParallel)
                    highlightPath(dfs);
                result.append(dfs);
                break;

            case "Dijkstra":
                if (startVertex == null)
                    throw new IllegalArgumentException("Start vertex required.");
                Map<String, Integer> dij = (Map<String, Integer>) alg.getMethod("dijkstra", Graph.class, String.class)
                        .invoke(null, graph, startVertex.label);
                if (!isParallel)
                    highlightPath(new ArrayList<>(dij.keySet()));
                result.append(dij);
                break;

            case "Topological Sort":
                if (!graph.isDirected())
                    throw new IllegalArgumentException("Topological Sort requires a directed graph.");
                List<String> topo = (List<String>) alg.getMethod("topologicalSort", Graph.class)
                        .invoke(null, graph);
                if (topo != null) {
                    if (!isParallel)
                        highlightPath(topo);
                    result.append(topo);
                } else {
                    result.append("Cycle detected - no valid topological sort.");
                }
                break;

            case "Floyd-Warshall":
                Map<String, Map<String, Integer>> fw = (Map<String, Map<String, Integer>>) alg
                        .getMethod("floydWarshall", Graph.class).invoke(null, graph);
                for (String from : fw.keySet()) {
                    result.append(from).append(": ").append(fw.get(from)).append("\n");
                }
                break;

            case "Connected Components":
                if (graph.isDirected())
                    throw new IllegalArgumentException("Requires undirected graph.");
                List<List<String>> comps = (List<List<String>>) alg.getMethod("connectedComponents", Graph.class)
                        .invoke(null, graph);
                for (List<String> comp : comps)
                    result.append("  ").append(comp).append("\n");
                break;

            case "Cycle Detection":
                if (graph.isDirected())
                    throw new IllegalArgumentException("Only supports undirected graphs.");
                boolean cycle = (boolean) alg.getMethod("hasCycle", Graph.class).invoke(null, graph);
                result.append("Cycle Detected: ").append(cycle);
                break;

            case "Kruskal":
                List<Graph.EdgeRecord> mst = (List<Graph.EdgeRecord>) alg.getMethod("kruskalMST", Graph.class)
                        .invoke(null, graph);
                if (!isParallel) {
                    List<EdgeRecord> highlightEdges = new ArrayList<>();
                    for (Graph.EdgeRecord er : mst) {
                        Vertex from = findVertex(er.from), to = findVertex(er.to);
                        if (from != null && to != null)
                            highlightEdges.add(new EdgeRecord(from, to, er.weight, false));
                    }
                    highlightEdges(highlightEdges);
                }
                result.append(mst);
                break;

            default:
                result.append("Unknown algorithm.");
        }

        return result.toString();
    }

    private void highlightPath(List<String> labels) {
        gc.setStroke(Color.RED);
        gc.setLineWidth(3);
        for (int i = 0; i < labels.size() - 1; i++) {
            Vertex v1 = findVertex(labels.get(i));
            Vertex v2 = findVertex(labels.get(i + 1));
            if (v1 != null && v2 != null) {
                gc.strokeLine(v1.x, v1.y, v2.x, v2.y);
            }
        }
    }

    private void highlightEdges(List<EdgeRecord> edgeList) {
        gc.setStroke(Color.RED);
        gc.setLineWidth(3);
        for (EdgeRecord e : edgeList) {
            gc.strokeLine(e.v1.x, e.v1.y, e.v2.x, e.v2.y);
        }
    }

    private Vertex findVertex(String label) {
        for (Vertex v : vertices)
            if (v.label.equals(label))
                return v;
        return null;
    }

    private void removeVertex(Vertex vertex) {
        List<EdgeRecord> rem = new ArrayList<>();
        edges.removeIf(e -> {
            if (e.v1 == vertex || e.v2 == vertex) {
                rem.add(e);
                graph.removeEdge(e.v1.label, e.v2.label, e.directed);
                return true;
            }
            return false;
        });
        vertices.remove(vertex);
        graph.removeVertex(vertex.label);
        undoStack.push(() -> {
            vertices.add(vertex);
            graph.addVertex(vertex.label);
            rem.forEach(e -> {
                edges.add(e);
                graph.addEdge(e.v1.label, e.v2.label, e.weight, e.directed);
            });
        });
        instructionLabel.setText("Vertex removed.");
    }

    private void removeEdgeAt(double x, double y) {
        for (Iterator<EdgeRecord> it = edges.iterator(); it.hasNext();) {
            EdgeRecord e = it.next();
            double dx = x - (e.v1.x + e.v2.x) / 2;
            double dy = y - (e.v1.y + e.v2.y) / 2;
            if (Math.hypot(dx, dy) < 10) {
                it.remove();
                graph.removeEdge(e.v1.label, e.v2.label, e.directed);
                undoStack.push(() -> {
                    edges.add(e);
                    graph.addEdge(e.v1.label, e.v2.label, e.weight, e.directed);
                });
                instructionLabel.setText("Edge removed.");
                redraw();
                return;
            }
        }
    }

    private void handleKeyPress(KeyEvent ev) {
        if (ev.isControlDown() && ev.getCode() == KeyCode.Z && !undoStack.isEmpty()) {
            undoStack.pop().run();
            redraw();
        }
    }

    private void showEdgeDialog(Vertex v1, Vertex v2) {
        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Add Edge");
        TextField weightField = new TextField("1");
        RadioButton dir = new RadioButton("Directed"), undir = new RadioButton("Undirected");
        undir.setSelected(true);
        ToggleGroup g = new ToggleGroup();
        dir.setToggleGroup(g);
        undir.setToggleGroup(g);
        VBox content = new VBox(10, new Label("Weight:"), weightField, dir, undir);
        dialog.getDialogPane().setContent(content);
        dialog.getDialogPane().getButtonTypes().addAll(ButtonType.OK, ButtonType.CANCEL);

        dialog.showAndWait().ifPresent(bt -> {
            if (bt == ButtonType.OK) {
                try {
                    int w = Integer.parseInt(weightField.getText());
                    boolean isDir = dir.isSelected();
                    graph.addEdge(v1.label, v2.label, w, isDir);
                    EdgeRecord e = new EdgeRecord(v1, v2, w, isDir);
                    edges.add(e);
                    undoStack.push(() -> {
                        edges.remove(e);
                        graph.removeEdge(v1.label, v2.label, isDir);
                    });
                    redraw();
                    instructionLabel.setText("Edge added.");
                } catch (NumberFormatException ex) {
                    instructionLabel.setText("Invalid weight!");
                }
            }
        });
    }

    private void redraw() {
        gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        edges.forEach(this::drawEdge);
        vertices.forEach(this::drawVertex);
        updateGraphOutput(graphData());
    }

    private String graphData() {
        StringBuilder sb = new StringBuilder("Vertices:\n");
        vertices.forEach(v -> sb.append(v.label).append("\n"));
        sb.append("\nEdges:\n");
        edges.forEach(e -> sb.append(e.v1.label).append(e.directed ? "->" : "--")
                .append(e.v2.label).append(" [").append(e.weight).append("]\n"));
        sb.append("\nGraph Type: ").append(isGraphDirected() ? "Directed" : "Undirected").append("\n");
        return sb.toString();
    }

    private void drawVertex(Vertex v) {
        gc.setFill(Color.LIGHTBLUE);
        gc.fillOval(v.x - 15, v.y - 15, 30, 30);
        gc.setStroke(Color.BLACK);
        gc.strokeOval(v.x - 15, v.y - 15, 30, 30);
        gc.setFill(Color.BLACK);
        gc.fillText(v.label, v.x - 4, v.y + 4);
    }

    private void drawEdge(EdgeRecord e) {
        gc.setStroke(Color.DARKBLUE);
        gc.setLineWidth(2);
        gc.strokeLine(e.v1.x, e.v1.y, e.v2.x, e.v2.y);
        double mx = (e.v1.x + e.v2.x) / 2, my = (e.v1.y + e.v2.y) / 2;
        gc.fillText("" + e.weight, mx, my);
        if (e.directed)
            drawArrow(e);
    }

    private void drawArrow(EdgeRecord e) {
        double x1 = e.v1.x, y1 = e.v1.y, x2 = e.v2.x, y2 = e.v2.y;
        double ang = Math.atan2(y2 - y1, x2 - x1), len = 10, r = 15;
        double tx = x2 - r * Math.cos(ang), ty = y2 - r * Math.sin(ang);
        double a1 = ang - Math.PI / 8, a2 = ang + Math.PI / 8;
        gc.strokeLine(x1, y1, tx, ty);
        gc.strokeLine(tx, ty, tx - len * Math.cos(a1), ty - len * Math.sin(a1));
        gc.strokeLine(tx, ty, tx - len * Math.cos(a2), ty - len * Math.sin(a2));
    }

    private Vertex getVertexAt(double x, double y) {
        return vertices.stream()
                .filter(v -> Math.hypot(x - v.x, y - v.y) <= 15)
                .findFirst().orElse(null);
    }

    private String generateLabel() {
        return labelTypeLetters.isSelected()
                ? String.valueOf((char) ('A' + vertexCount))
                : String.valueOf(vertexCount + 1);
    }

    // Inner classes
    private static class Vertex {
        double x, y;
        String label;

        Vertex(double x, double y, String label) {
            this.x = x;
            this.y = y;
            this.label = label;
        }
    }

    private static class EdgeRecord {
        Vertex v1, v2;
        int weight;
        boolean directed;

        EdgeRecord(Vertex v1, Vertex v2, int weight, boolean directed) {
            this.v1 = v1;
            this.v2 = v2;
            this.weight = weight;
            this.directed = directed;
        }
    }

    // Add these at the **very end** of your class Controller, before the closing
    // `}`

    private void runAlgorithm(String algorithm, Vertex startVertex) {
        graph.setDirected(isGraphDirected()); // Refresh directed state
        redraw(); // Redraw graph before running

        StringBuilder output = new StringBuilder();
        output.append("CPU Usage Before: ").append(getCpuUsage()).append("\n\n");

        try {
            long startTime = System.nanoTime();
            String resultSeq = runAlgorithmInternal(algorithm, startVertex, false);
            long duration = System.nanoTime() - startTime;

            output.append("Sequential Result:\n").append(resultSeq).append("\n");
            output.append("Sequential Time: ").append(duration).append(" ns\n\n");
        } catch (Exception e) {
            output.append("Sequential Error: ").append(e.getMessage()).append("\n");
        }

        try {
            long startTime = System.nanoTime();
            String resultPar = runAlgorithmInternal(algorithm, startVertex, true);
            long duration = System.nanoTime() - startTime;

            output.append("Parallel Result:\n").append(resultPar).append("\n");
            output.append("Parallel Time: ").append(duration).append(" ns\n");
        } catch (Exception e) {
            output.append("Parallel Error: ").append(e.getMessage()).append("\n");
        }

        output.append("\nCPU Usage After: ").append(getCpuUsage());

        updateGraphOutput(output.toString());
    }

    private void updateGraphOutput(String text) {
        graphDataArea.setText(text);
    }
    // for the large network
}
