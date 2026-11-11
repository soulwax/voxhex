import 'hex_coord.dart';

typedef HexPassableFn = bool Function(HexCoord coord);
typedef HexCostFn = double Function(HexCoord from, HexCoord to);

/// Result of a Dijkstra tactical movement reachability expansion.
final class HexReachableResult {
  final HexCoord origin;
  final Map<HexCoord, double> costs;
  final Map<HexCoord, HexCoord?> parents;

  const HexReachableResult({
    required this.origin,
    required this.costs,
    required this.parents,
  });

  bool canReach(HexCoord coord) => costs.containsKey(coord);
  Iterable<HexCoord> get reachableCoords => costs.keys;

  /// Reconstructs the cheapest path from [origin] to [goal], inclusive.
  /// Returns `null` when [goal] is unreachable within the movement budget.
  List<HexCoord>? pathTo(HexCoord goal) {
    if (!costs.containsKey(goal)) return null;
    final rev = <HexCoord>[];
    HexCoord? cur = goal;
    while (cur != null) {
      rev.add(cur);
      if (cur == origin) break;
      cur = parents[cur];
    }
    return rev.reversed.toList(growable: false);
  }
}

/// Tactical pathfinding & movement-range solvers over axial hex grids.
abstract final class HexPathfinder {
  /// Computes all tiles reachable from [origin] within movement point [budget].
  ///
  /// When [stopsMovement] returns `true` for an entered tile (e.g., an enemy Zone
  /// of Control), the agent can step onto that tile if budget permits, but cannot
  /// expand beyond it on the same turn.
  static HexReachableResult reachable(
    HexCoord origin, {
    required double budget,
    required HexPassableFn isPassable,
    HexCostFn? moveCost,
    bool Function(HexCoord coord)? stopsMovement,
  }) {
    final costFn = moveCost ?? (_, __) => 1.0;
    final costs = <HexCoord, double>{origin: 0.0};
    final parents = <HexCoord, HexCoord?>{origin: null};
    final frontier = <(HexCoord, double)>[(origin, 0.0)];

    while (frontier.isNotEmpty) {
      // Pop lowest accumulated cost
      var bestIdx = 0;
      for (int i = 1; i < frontier.length; i++) {
        if (frontier[i].$2 < frontier[bestIdx].$2) {
          bestIdx = i;
        }
      }
      final current = frontier.removeAt(bestIdx);
      final u = current.$1;
      final currentCost = current.$2;

      if (currentCost > (costs[u] ?? double.infinity)) continue;
      if (u != origin && stopsMovement != null && stopsMovement(u)) continue;

      for (final next in u.neighbors) {
        if (!isPassable(next)) continue;
        final stepCost = costFn(u, next);
        if (stepCost < 0) continue;
        final nextCost = currentCost + stepCost;
        if (nextCost <= budget + 1e-9 && nextCost < (costs[next] ?? double.infinity)) {
          costs[next] = nextCost;
          parents[next] = u;
          frontier.add((next, nextCost));
        }
      }
    }

    return HexReachableResult(
      origin: origin,
      costs: costs,
      parents: parents,
    );
  }

  /// A* shortest path search between [start] and [goal].
  static List<HexCoord>? findPath(
    HexCoord start,
    HexCoord goal, {
    required HexPassableFn isPassable,
    HexCostFn? moveCost,
    double heuristicScale = 1.0,
    int maxIterations = 8192,
  }) {
    if (start == goal) return [start];
    if (!isPassable(goal)) return null;

    final costFn = moveCost ?? (_, __) => 1.0;
    final gScore = <HexCoord, double>{start: 0.0};
    final parents = <HexCoord, HexCoord?>{start: null};
    final open = <(HexCoord coord, double f)>[
      (start, start.distanceTo(goal) * heuristicScale),
    ];
    final closed = <HexCoord>{};

    var steps = 0;
    while (open.isNotEmpty && steps++ < maxIterations) {
      var bestIdx = 0;
      for (int i = 1; i < open.length; i++) {
        if (open[i].$2 < open[bestIdx].$2) {
          bestIdx = i;
        }
      }
      final current = open.removeAt(bestIdx).$1;
      if (current == goal) {
        final rev = <HexCoord>[];
        HexCoord? c = goal;
        while (c != null) {
          rev.add(c);
          c = parents[c];
        }
        return rev.reversed.toList(growable: false);
      }

      if (!closed.add(current)) continue;
      final baseG = gScore[current]!;

      for (final next in current.neighbors) {
        if (closed.contains(next) || !isPassable(next)) continue;
        final tentativeG = baseG + costFn(current, next);
        if (tentativeG < (gScore[next] ?? double.infinity)) {
          gScore[next] = tentativeG;
          parents[next] = current;
          final f = tentativeG + next.distanceTo(goal) * heuristicScale;
          open.add((next, f));
        }
      }
    }

    return null;
  }
}
