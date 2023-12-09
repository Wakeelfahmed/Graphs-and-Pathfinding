package comp2023v2;

public class NetworkUtils implements INetworkUtils {
    // Other methods...

    @Override
    public ListInt depthFirstSearch(Network network, int index) {
        ListInt visitedNodes = new ListInt(network.getNumStations());
        boolean[] visited = new boolean[network.getNumStations()];
        depthFirstSearchHelper(network, index, visited, visitedNodes);
        return visitedNodes;
    }

    private void depthFirstSearchHelper(Network network, int index, boolean[] visited, ListInt visitedNodes) {
        visited[index] = true;
        visitedNodes.append(index);

        for (int i = 0; i < network.getNumStations(); i++) {
            if (network.getDistance(index, i) != network.NO_LINK && !visited[i]) {
                depthFirstSearchHelper(network, i, visited, visitedNodes);
            }
        }
    }

    @Override
    public ListInt breadthFirstSearch(Network network, int index) {
        ListInt visitedNodes = new ListInt(network.getNumStations());
        boolean[] visited = new boolean[network.getNumStations()];
        QueueInt queue = new QueueInt(network.getNumStations());

        visited[index] = true;
        visitedNodes.append(index);
        queue.addToBack(index);

        while (queue.getSize() != 0) {
            int current = queue.removefromFront();

            for (int i = 0; i < network.getNumStations(); i++) {
                if (network.getDistance(current, i) != network.NO_LINK && !visited[i]) {
                    visited[i] = true;
                    visitedNodes.append(i);
                    queue.addToBack(i);
                }
            }
        }

        return visitedNodes;
    }

    @Override
    public ListInt dijkstraPath(Network network, int startIndex, int endIndex) {
        ListInt result = new ListInt(network.getNumStations());

        SetInt closed = new SetInt(network.getNumStations());
        SetInt open = new SetInt(network.getNumStations());

        double[] gValues = new double[network.getNumStations()];
        int[] previous = new int[network.getNumStations()];

        for (int i = 0; i < network.getNumStations(); i++) {
            open.include(i);
            gValues[i] = Double.MAX_VALUE;
            previous[i] = -1;
        }

        gValues[startIndex] = 0.0;

        while (!closed.contains(endIndex)) {
            int x = findMinGValueNode(open, gValues);
            open.exclude(x);
            closed.include(x);

            if (x != endIndex) {
                for (int n = 0; n < network.getNumStations(); n++) {
                    if (network.getDistance(x, n) != network.NO_LINK && open.contains(n)) {
                        double gPrime = gValues[x] + network.getDistance(x, n);
                        if (gPrime < gValues[n]) {
                            gValues[n] = gPrime;
                            previous[n] = x;
                        }
                    }
                }
            }
        }

        int currentNode = endIndex;
        while (currentNode != -1) {
            result.append(currentNode);
            currentNode = previous[currentNode];
        }

        return result.reverse();
    }

    @Override
    public ListInt aStarPath(Network network, int startIndex, int endIndex) {
        ListInt result = new ListInt(network.getNumStations());

        SetInt closed = new SetInt(network.getNumStations());
        SetInt open = new SetInt(network.getNumStations());

        double[] gValues = new double[network.getNumStations()];
        double[] fValues = new double[network.getNumStations()];
        int[] previous = new int[network.getNumStations()];

        for (int i = 0; i < network.getNumStations(); i++) {
            open.include(i);
            gValues[i] = Double.MAX_VALUE;
            fValues[i] = Double.MAX_VALUE;
            previous[i] = -1;
        }

        gValues[startIndex] = 0.0;
        fValues[startIndex] = network.pythogoras(
                network.getStationInfo(startIndex).getxPos(),
                network.getStationInfo(startIndex).getyPos(),
                network.getStationInfo(endIndex).getxPos(),
                network.getStationInfo(endIndex).getyPos()
        );

        while (!closed.contains(endIndex)) {
            int x = findMinFValueNode(open, fValues);
            open.exclude(x);
            closed.include(x);

            if (x != endIndex) {
                for (int n = 0; n < network.getNumStations(); n++) {
                    if (network.getDistance(x, n) != network.NO_LINK && open.contains(n)) {
                        double gPrime = gValues[x] + network.getDistance(x, n);
                        if (gPrime < gValues[n]) {
                            gValues[n] = gPrime;
                            fValues[n] = gPrime + network.pythogoras(
                                    network.getStationInfo(n).getxPos(),
                                    network.getStationInfo(n).getyPos(),
                                    network.getStationInfo(endIndex).getxPos(),
                                    network.getStationInfo(endIndex).getyPos()
                            );
                            previous[n] = x;
                        }
                    }
                }
            }
        }

        int currentNode = endIndex;
        while (currentNode != -1) {
            result.append(currentNode);
            currentNode = previous[currentNode];
        }

        return result.reverse();
    }

    private int findMinGValueNode(SetInt open, double[] gValues) {
        int minNode = -1;
        double minValue = Double.MAX_VALUE;

        for (int node : open.set) {
            if (gValues[node] < minValue) {
                minValue = gValues[node];
                minNode = node;
            }
        }

        return minNode;
    }

    private int findMinFValueNode(SetInt open, double[] fValues) {
        int minNode = -1;
        double minValue = Double.MAX_VALUE;

        for (int node : open.set) {
            if (fValues[node] < minValue) {
                minValue = fValues[node];
                minNode = node;
            }
        }

        return minNode;
    }

    // Other methods...
}
