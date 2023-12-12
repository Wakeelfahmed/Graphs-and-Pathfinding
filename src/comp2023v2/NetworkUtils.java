package comp2023v2;

public class NetworkUtils implements INetworkUtils {
    // Helper method to get adjacent stations
    private ListInt getAdjacentStations(Network network, int stationIndex) {
        ListInt adjacentStations = new ListInt(network.getNumStations());
        for (int i = 0; i < network.getNumStations(); i++) {
            if (network.getDistance(stationIndex, i) != network.NO_LINK) {
                adjacentStations.append(i);
            }
        }
        return adjacentStations;
    }

    @Override
    public ListInt breadthFirstSearch(Network network, int index) {
        ListInt result = new ListInt(network.MAX_STATIONS);
        QueueInt queue = new QueueInt(network.MAX_STATIONS);
        SetInt visitedSet = new SetInt(network.MAX_STATIONS);

        queue.addToBack(index);

        while (queue.getSize() != 0) {
            int currentStation = queue.removefromFront();

            if (!visitedSet.contains(currentStation)) {
                result.append(currentStation);
                visitedSet.include(currentStation);

                ListInt adjacentStations = getAdjacentStations(network, currentStation);
                for (int i = 0; i < adjacentStations.getSize(); i++) {
                    int neighbor = adjacentStations.get(i);
                    if (!visitedSet.contains(neighbor)) {
                        queue.addToBack(neighbor);
                    }
                }
            }
        }

        return result;
    }

    @Override
    public ListInt depthFirstSearch(Network network, int index) {
        ListInt result = new ListInt(network.MAX_STATIONS);
        StackInt stack = new StackInt(network.MAX_STATIONS);
        SetInt visitedSet = new SetInt(network.MAX_STATIONS);

        stack.push(index);

        while (stack.getSize() != 0) {
            int currentStation = stack.pop();

            if (!visitedSet.contains(currentStation)) {
                result.append(currentStation);
                visitedSet.include(currentStation);

                ListInt adjacentStations = getAdjacentStations(network, currentStation);
                for (int i = 0; i < adjacentStations.getSize(); i++) {
                    int neighbor = adjacentStations.get(i);
                    if (!visitedSet.contains(neighbor)) {
                        stack.push(neighbor);
                    }
                }
            }
        }
        return result;
    }

    @Override
    public ListInt dijkstraPath(Network network, int startIndex, int endIndex) {
        ListInt result = new ListInt(network.getNumStations());

        SetInt closedSet = new SetInt(network.getNumStations());
        SetInt openSet = new SetInt(network.getNumStations());
        double[] gValues = new double[network.getNumStations()];
        int[] previous = new int[network.getNumStations()];

        // Initialize open set, g-values, and previous
        for (int i = 0; i < network.getNumStations(); i++) {
            openSet.include(i);
            gValues[i] = Double.MAX_VALUE;
            previous[i] = -1; // None
        }

        // Set the g-value of the start node to 0
        gValues[startIndex] = 0;

        while (!closedSet.contains(endIndex)) {
            int x = findLowestGValueNode(openSet, gValues);
            openSet.exclude(x);
            closedSet.include(x);

            if (x != endIndex) {
                ListInt adjacentNodes = getAdjacentStations(network, x);
                for (int i = 0; i < adjacentNodes.getSize(); i++) {
                    int n = adjacentNodes.get(i);
                    if (openSet.contains(n)) {
                        double gPrime = gValues[x] + network.getDistance(x, n);
                        if (gPrime < gValues[n]) {
                            gValues[n] = gPrime;
                            previous[n] = x;
                        }
                    }
                }
            }
        }

        // Reconstruct the path
        int current = endIndex;
        while (current != -1) {
            result.append(current);
            current = previous[current];
        }

        // Reverse the result list to get the correct order
        result = reverseList(result);

        return result;
    }

    // Helper method to find the node with the lowest g-value
    private int findLowestGValueNode(SetInt openSet, double[] gValues) {
        int lowestNode = -1;  // Initialize to an invalid node index
        double lowestValue = Double.POSITIVE_INFINITY;

        for (int i = 0; i < gValues.length; i++) {
            if (openSet.contains(i) && gValues[i] < lowestValue) {
                lowestNode = i;
                lowestValue = gValues[i];
            }
        }

        return lowestNode;
    }


    // Helper method to reverse a list
    private ListInt reverseList(ListInt list) {
        ListInt reversed = new ListInt(list.getCapacity());
        for (int i = list.getSize() - 1; i >= 0; i--) {
            reversed.append(list.get(i));
        }
        return reversed;
    }

    @Override
    public ListInt aStarPath(Network network, int startIndex, int endIndex) {
        ListInt result = new ListInt(network.MAX_STATIONS);
        SetInt closedSet = new SetInt(network.MAX_STATIONS);
        SetInt openSet = new SetInt(network.MAX_STATIONS);
        double[] gValues = new double[network.MAX_STATIONS];
        double[] fValues = new double[network.MAX_STATIONS];
        double[] straightLineDistances = new double[network.MAX_STATIONS];
        int[] previous = new int[network.MAX_STATIONS];

        // Initialize open set, g-values, f-values, straight-line distances, and previous
        for (int i = 0; i < network.getNumStations(); i++) {
            openSet.include(i);
            gValues[i] = Double.POSITIVE_INFINITY;
            fValues[i] = Double.POSITIVE_INFINITY;
            straightLineDistances[i] = calculateStraightLineDistance(
                    network.getStationInfo(i).getxPos(),
                    network.getStationInfo(i).getyPos(),
                    network.getStationInfo(endIndex).getxPos(),
                    network.getStationInfo(endIndex).getyPos()
            );
            previous[i] = -1; // None
        }

        // Set the g-value of the start node to 0
        gValues[startIndex] = 0;

        // Set the f-value of the start node to straight-line distance from start to end
        fValues[startIndex] = gValues[startIndex] + straightLineDistances[startIndex];

        while (!closedSet.contains(endIndex)) {
            int x = findLowestFValueNode(openSet, fValues);
            openSet.exclude(x);
            closedSet.include(x);

            if (x != endIndex) {
                ListInt adjacentNodes = getAdjacentStations(network, x);
                for (int i = 0; i < adjacentNodes.getSize(); i++) {
                    int n = adjacentNodes.get(i);
                    if (openSet.contains(n)) {
                        double gPrime = gValues[x] + network.getDistance(x, n);
                        if (gPrime < gValues[n]) {
                            gValues[n] = gPrime;
                            fValues[n] = gValues[n] + straightLineDistances[n];
                            previous[n] = x;
                        }
                    }
                }
            }
        }

        // Reconstruct the path
        int current = endIndex;
        while (current != -1) {
            result.append(current);
            current = previous[current];
        }

        // Reverse the result list to get the correct order
        result = reverseList(result);

        return result;
    }

    // Helper method to calculate straight-line distance using Pythagoras
    private double calculateStraightLineDistance(double x0, double y0, double x1, double y1) {
        return Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    }

    // Helper method to find the node with the lowest f-value
    private int findLowestFValueNode(SetInt openSet, double[] fValues) {
        int lowestNode = -1;
        double lowestValue = Double.POSITIVE_INFINITY;

        for (int i = 0; i < fValues.length; i++) {
            if (openSet.contains(i) && fValues[i] < lowestValue) {
                lowestNode = i;
                lowestValue = fValues[i];
            }
        }

        return lowestNode;
    }
}
