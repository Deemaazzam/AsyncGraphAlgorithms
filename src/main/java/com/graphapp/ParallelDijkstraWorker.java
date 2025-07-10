package com.graphapp;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

public class ParallelDijkstraWorker implements Runnable {
    private final MultiQueues queue;
    private final Graph graph;
    private final int[] distances;
    private final ReentrantLock[] distancesLocks;
    private final ReentrantLock[] offersLocks;
    private final Offer[] offers;
    private final AtomicBoolean[] done;
    private final int tid;
    private final int numThreads;
    private final Map<String, Integer> vertexIndices; // maps vertex name to index

    public ParallelDijkstraWorker(MultiQueues queue, Graph graph, int[] distances, ReentrantLock[] distancesLocks,
            ReentrantLock[] offersLocks, Offer[] offers, AtomicBoolean[] done, int tid, int numThreads,
            Map<String, Integer> vertexIndices) {
        this.queue = queue;
        this.graph = graph;
        this.distances = distances;
        this.distancesLocks = distancesLocks;
        this.offersLocks = offersLocks;
        this.offers = offers;
        this.done = done;
        this.tid = tid;
        this.numThreads = numThreads;
        this.vertexIndices = vertexIndices;
    }

    private boolean finishedWork() {
        for (int i = 0; i < numThreads; i++) {
            if (!done[i].get())
                return false;
        }
        return true;
    }

    private void relax(String vertex, int alt) {
        int index = vertexIndices.get(vertex);
        offersLocks[index].lock();
        try {
            distancesLocks[index].lock();
            try {
                int currDist = distances[index];
                if (alt < currDist) {
                    Offer currOffer = offers[index];
                    if (currOffer == null || alt < currOffer.dist) {
                        Offer newOffer = queue.insert(vertex, alt); // MultiQueues and Offer should use String vertex
                                                                    // now
                        offers[index] = newOffer;
                    }
                }
            } finally {
                distancesLocks[index].unlock();
            }
        } finally {
            offersLocks[index].unlock();
        }
    }

    @Override
    public void run() {
        while (true) {
            if (queue.isEmpty()) {
                done[tid].set(true);
            }

            if (!done[tid].get()) {
                Offer minOffer = queue.deleteMin();
                if (minOffer == null) {
                    done[tid].set(true);
                    if (finishedWork())
                        return;
                    else
                        continue;
                }

                done[tid].set(false);

                String currV = minOffer.vertex; // now vertex is a String
                int currDist = minOffer.dist;
                int currIndex = vertexIndices.get(currV);

                distancesLocks[currIndex].lock();
                boolean explore;
                try {
                    if (currDist < distances[currIndex]) {
                        distances[currIndex] = currDist;
                        explore = true;
                    } else {
                        explore = false;
                    }
                } finally {
                    distancesLocks[currIndex].unlock();
                }

                if (explore) {
                    List<Graph.Edge> neighbors = graph.getNeighbors(currV);
                    for (Graph.Edge edge : neighbors) {
                        String neighbor = edge.to; // use String directly
                        int weight = edge.weight;
                        int alt = currDist + weight;
                        relax(neighbor, alt);
                    }
                }
            } else {
                if (finishedWork())
                    return;
            }
        }
    }
}
