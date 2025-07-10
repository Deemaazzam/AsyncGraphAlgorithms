package com.graphapp;

import java.io.*;
import java.util.*;
import com.graphapp.Graph;

public class GraphLoader {
    public static Graph loadDirectedGraphFromFile(String filePath) throws IOException {
        Graph graph = new Graph(true); // directed = true
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    // Skip empty lines or comments
                    continue;
                }
                String[] parts = line.split("\\s+");
                if (parts.length < 2)
                    continue;

                String from = parts[0];
                String to = parts[1];

                // Add vertices (if your Graph class requires explicit vertex adding)
                if (!graph.containsVertex(from)) {
                    graph.addVertex(from);
                }
                if (!graph.containsVertex(to)) {
                    graph.addVertex(to);
                }

                // Add directed edge with default weight 1
                graph.addEdge(from, to, 1, true);
            }
        }
        return graph;
    }
}
