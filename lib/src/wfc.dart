import 'dart:math' as math;

import 'hex_coord.dart';
import 'hex_map.dart';

final class _WfcTileEntry<T> {
  final T value;
  final double weight;
  final List<Set<int>> allowedNeighbors;
  List<String>? sockets;

  _WfcTileEntry({
    required this.value,
    required this.weight,
    required int directionsCount,
    this.sockets,
  }) : allowedNeighbors = List.generate(directionsCount, (_) => <int>{});
}

/// Adjacency rule builder for 2D rectangular (`directionsCount = 4`) or hexagonal
/// (`directionsCount = 6`) Wave-Function Collapse.
final class WfcRuleSet<T> {
  final int directionsCount;
  final List<_WfcTileEntry<T>> _entries = [];
  final Map<T, int> _indexByValue = {};

  WfcRuleSet({required this.directionsCount}) {
    if (directionsCount < 2) {
      throw ArgumentError.value(directionsCount, 'directionsCount', 'Must be >= 2');
    }
  }

  /// Helper for 4-neighbor orthogonal grids (0=North, 1=East, 2=South, 3=West).
  factory WfcRuleSet.grid2d() => WfcRuleSet<T>(directionsCount: 4);

  /// Helper for 6-neighbor hex grids matching [HexCoord.directions] order `0..5`.
  factory WfcRuleSet.hex() => WfcRuleSet<T>(directionsCount: 6);

  int get tileCount => _entries.length;
  int _opposite(int dir) => (dir + (directionsCount ~/ 2)) % directionsCount;

  int addTile(T tile, {double weight = 1.0}) {
    final existing = _indexByValue[tile];
    if (existing != null) return existing;
    final idx = _entries.length;
    _entries.add(_WfcTileEntry<T>(
      value: tile,
      weight: math.max(1e-6, weight),
      directionsCount: directionsCount,
    ));
    _indexByValue[tile] = idx;
    return idx;
  }

  /// Declares tiles using directional edge socket names (`edgeSockets.length == directionsCount`).
  ///
  /// Any pair of tiles whose touching opposite edges share the exact same socket string
  /// are automatically allowed adjacent in that direction.
  void addSocketTile(T tile, List<String> edgeSockets, {double weight = 1.0}) {
    if (edgeSockets.length != directionsCount) {
      throw ArgumentError(
        'Expected $directionsCount sockets for tile "$tile", got ${edgeSockets.length}',
      );
    }
    final idx = addTile(tile, weight: weight);
    _entries[idx].sockets = List<String>.of(edgeSockets);

    // Connect against all registered socket tiles including self
    for (int j = 0; j < _entries.length; j++) {
      final otherSockets = _entries[j].sockets;
      if (otherSockets == null) continue;
      for (int dir = 0; dir < directionsCount; dir++) {
        final opp = _opposite(dir);
        if (edgeSockets[dir] == otherSockets[opp]) {
          _entries[idx].allowedNeighbors[dir].add(j);
          _entries[j].allowedNeighbors[opp].add(idx);
        }
      }
    }
  }

  /// Allows [toTile] adjacent to [fromTile] in [direction].
  void allow(
    T fromTile,
    int direction,
    T toTile, {
    bool symmetric = true,
  }) {
    final a = _indexByValue[fromTile] ?? addTile(fromTile);
    final b = _indexByValue[toTile] ?? addTile(toTile);
    final d = (direction % directionsCount + directionsCount) % directionsCount;
    _entries[a].allowedNeighbors[d].add(b);
    if (symmetric) {
      _entries[b].allowedNeighbors[_opposite(d)].add(a);
    }
  }

  int? indexOf(T tile) => _indexByValue[tile];
}

/// Directed edge between two cell indices in a general topology graph.
final class WfcCellEdge {
  final int neighborCellIndex;
  final int directionFromSelf;

  const WfcCellEdge(this.neighborCellIndex, this.directionFromSelf);
}

/// Topology-agnostic backtracking Wave-Function Collapse solver using Shannon entropy
/// heuristic and iterative AC-3 constraint propagation.
final class WfcSolver<T> {
  final int cellCount;
  final List<List<WfcCellEdge>> adjacency;
  final WfcRuleSet<T> rules;
  final math.Random _rng;

  WfcSolver({
    required this.cellCount,
    required this.adjacency,
    required this.rules,
    int? seed,
  }) : _rng = math.Random(seed);

  /// Solves the constraint graph and returns one collapsed [T] per cell, or `null`
  /// if no assignment satisfies [rules] and [preplaced] constraints.
  List<T>? solve({
    Map<int, T>? preplaced,
    int maxBacktracks = 4096,
  }) {
    if (rules.tileCount == 0 || cellCount == 0) return null;

    final domains = List<Set<int>>.generate(
      cellCount,
      (_) => {for (int i = 0; i < rules.tileCount; i++) i},
    );

    final worklist = <int>[];
    if (preplaced != null) {
      for (final e in preplaced.entries) {
        final tileIdx = rules.indexOf(e.value);
        if (tileIdx == null || e.key < 0 || e.key >= cellCount) return null;
        domains[e.key] = {tileIdx};
        worklist.add(e.key);
      }
    }

    if (!_propagate(domains, worklist)) {
      return null;
    }

    var backtracksRemaining = maxBacktracks;

    bool backtrack(List<Set<int>> currentDomains) {
      // Choose uncollapsed cell with minimum Shannon entropy (> 1 option)
      var chosenCell = -1;
      var minEntropy = double.infinity;

      for (int c = 0; c < cellCount; c++) {
        final dom = currentDomains[c];
        if (dom.isEmpty) return false;
        if (dom.length == 1) continue;

        final e = _entropy(dom) + _rng.nextDouble() * 1e-4;
        if (e < minEntropy) {
          minEntropy = e;
          chosenCell = c;
        }
      }

      if (chosenCell == -1) return true; // All cells collapsed!

      // Order candidate tile choices weighted randomly
      final candidates = _weightedShuffle(currentDomains[chosenCell]);
      for (final pick in candidates) {
        final nextDomains = [
          for (final d in currentDomains) Set<int>.of(d),
        ];
        nextDomains[chosenCell] = {pick};

        if (_propagate(nextDomains, [chosenCell])) {
          if (backtrack(nextDomains)) {
            for (int c = 0; c < cellCount; c++) {
              currentDomains[c] = nextDomains[c];
            }
            return true;
          }
        }
        if (backtracksRemaining-- <= 0) return false;
      }
      return false;
    }

    if (!backtrack(domains)) return null;

    return [
      for (int c = 0; c < cellCount; c++) rules._entries[domains[c].first].value,
    ];
  }

  double _entropy(Set<int> dom) {
    var sumW = 0.0;
    var sumWLogW = 0.0;
    for (final t in dom) {
      final w = rules._entries[t].weight;
      sumW += w;
      sumWLogW += w * math.log(w);
    }
    return math.log(sumW) - (sumWLogW / sumW);
  }

  List<int> _weightedShuffle(Set<int> dom) {
    final pool = dom.toList();
    final ordered = <int>[];
    while (pool.isNotEmpty) {
      var total = 0.0;
      for (final idx in pool) {
        total += rules._entries[idx].weight;
      }
      var roll = _rng.nextDouble() * total;
      var chosenPos = pool.length - 1;
      for (int i = 0; i < pool.length; i++) {
        roll -= rules._entries[pool[i]].weight;
        if (roll <= 0.0) {
          chosenPos = i;
          break;
        }
      }
      ordered.add(pool.removeAt(chosenPos));
    }
    return ordered;
  }

  bool _propagate(List<Set<int>> domains, List<int> seedQueue) {
    final queue = List<int>.of(seedQueue);
    final inQueue = queue.toSet();
    var head = 0;

    while (head < queue.length) {
      final cell = queue[head++];
      inQueue.remove(cell);
      final selfDom = domains[cell];
      if (selfDom.isEmpty) return false;

      for (final edge in adjacency[cell]) {
        final neighbor = edge.neighborCellIndex;
        final neighborDom = domains[neighbor];
        final dir = edge.directionFromSelf;

        // Collect all tile indices permitted by at least one option in selfDom along dir
        final allowedForNeighbor = <int>{};
        for (final t in selfDom) {
          allowedForNeighbor.addAll(rules._entries[t].allowedNeighbors[dir]);
        }

        final beforeSize = neighborDom.length;
        neighborDom.retainAll(allowedForNeighbor);
        if (neighborDom.isEmpty) return false;

        if (neighborDom.length < beforeSize && !inQueue.contains(neighbor)) {
          queue.add(neighbor);
          inQueue.add(neighbor);
        }
      }
    }
    return true;
  }
}

/// Convenience 2D rectangular WFC generator (`width` x `height`).
abstract final class WfcGrid2D {
  static List<List<T>>? solve<T>({
    required int width,
    required int height,
    required WfcRuleSet<T> rules,
    int? seed,
    Map<(int x, int y), T>? preplaced,
  }) {
    final count = width * height;
    final adj = List<List<WfcCellEdge>>.generate(count, (_) => []);

    // Direction indices: 0=North(y-1), 1=East(x+1), 2=South(y+1), 3=West(x-1)
    const dx = [0, 1, 0, -1];
    const dy = [-1, 0, 1, 0];

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        final idx = y * width + x;
        for (int dir = 0; dir < 4; dir++) {
          final nx = x + dx[dir];
          final ny = y + dy[dir];
          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            adj[idx].add(WfcCellEdge(ny * width + nx, dir));
          }
        }
      }
    }

    final preMap = <int, T>{};
    if (preplaced != null) {
      for (final e in preplaced.entries) {
        final (x, y) = e.key;
        if (x >= 0 && x < width && y >= 0 && y < height) {
          preMap[y * width + x] = e.value;
        }
      }
    }

    final flat = WfcSolver<T>(
      cellCount: count,
      adjacency: adj,
      rules: rules,
      seed: seed,
    ).solve(preplaced: preMap);

    if (flat == null) return null;
    return [
      for (int y = 0; y < height; y++)
        [for (int x = 0; x < width; x++) flat[y * width + x]],
    ];
  }
}

/// Convenience hexagonal grid WFC generator over arbitrary sets of [HexCoord]s.
abstract final class WfcHexGrid {
  static HexMap<T>? solve<T>({
    required Iterable<HexCoord> coords,
    required WfcRuleSet<T> rules,
    int? seed,
    Map<HexCoord, T>? preplaced,
  }) {
    final coordList = coords.toList(growable: false);
    final indexOf = <HexCoord, int>{
      for (int i = 0; i < coordList.length; i++) coordList[i]: i,
    };

    final adj = List<List<WfcCellEdge>>.generate(coordList.length, (_) => []);
    for (int i = 0; i < coordList.length; i++) {
      final c = coordList[i];
      for (int dir = 0; dir < 6; dir++) {
        final n = c.neighbor(dir);
        final nIdx = indexOf[n];
        if (nIdx != null) {
          adj[i].add(WfcCellEdge(nIdx, dir));
        }
      }
    }

    final preMap = <int, T>{};
    if (preplaced != null) {
      for (final e in preplaced.entries) {
        final idx = indexOf[e.key];
        if (idx != null) preMap[idx] = e.value;
      }
    }

    final flat = WfcSolver<T>(
      cellCount: coordList.length,
      adjacency: adj,
      rules: rules,
      seed: seed,
    ).solve(preplaced: preMap);

    if (flat == null) return null;
    final out = HexMap<T>();
    for (int i = 0; i < coordList.length; i++) {
      out[coordList[i]] = flat[i];
    }
    return out;
  }
}
