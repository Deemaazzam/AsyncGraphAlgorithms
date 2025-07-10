package com.graphapp;

import java.util.Random;
import java.util.concurrent.PriorityBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class MultiQueues {
    private final int numQueues;
    private final PriorityBlockingQueue<Offer>[] queues;
    private final ReentrantLock[] locks;
    private final Random random = new Random();

    @SuppressWarnings("unchecked")
    public MultiQueues(int numQueues) {
        this.numQueues = numQueues;
        this.queues = new PriorityBlockingQueue[numQueues];
        this.locks = new ReentrantLock[numQueues];

        for (int i = 0; i < numQueues; i++) {
            queues[i] = new PriorityBlockingQueue<>(11, (o1, o2) -> Integer.compare(o1.dist, o2.dist));
            locks[i] = new ReentrantLock();
        }
    }

    // Insert offer into a random queue - vertex is now String
    public Offer insert(String vertex, int dist) {
        int idx = random.nextInt(numQueues);
        Offer offer = new Offer(vertex, dist);
        locks[idx].lock();
        try {
            queues[idx].offer(offer);
        } finally {
            locks[idx].unlock();
        }
        return offer;
    }

    // Delete min by checking two random queues, return the minimal offer found or
    // null
    public Offer deleteMin() {
        int q1 = random.nextInt(numQueues);
        int q2 = random.nextInt(numQueues);
        if (q1 == q2)
            q2 = (q2 + 1) % numQueues;

        locks[q1].lock();
        locks[q2].lock();
        try {
            Offer o1 = queues[q1].peek();
            Offer o2 = queues[q2].peek();

            if (o1 == null && o2 == null) {
                return null;
            } else if (o1 == null) {
                return queues[q2].poll();
            } else if (o2 == null) {
                return queues[q1].poll();
            } else {
                if (o1.dist <= o2.dist) {
                    return queues[q1].poll();
                } else {
                    return queues[q2].poll();
                }
            }
        } finally {
            locks[q2].unlock();
            locks[q1].unlock();
        }
    }

    public boolean isEmpty() {
        for (int i = 0; i < numQueues; i++) {
            if (!queues[i].isEmpty())
                return false;
        }
        return true;
    }
}
