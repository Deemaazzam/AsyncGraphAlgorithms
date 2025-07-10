package com.graphapp;

import com.graphapp.Graph;

public class Offer implements Comparable<Offer> {
    public final String vertex;
    public final int dist;

    public Offer(String vertex, int dist) {
        this.vertex = vertex;
        this.dist = dist;
    }

    @Override
    public int compareTo(Offer other) {
        return Integer.compare(this.dist, other.dist);
    }
}
