import 'hex_coord.dart';

/// Sparse or shaped axial map container storing [T] values keyed by [HexCoord].
final class HexMap<T> {
  final Map<HexCoord, T> _cells = {};

  HexMap([Map<HexCoord, T>? initial]) {
    if (initial != null) {
      _cells.addAll(initial);
    }
  }

  /// Creates a hexagonal shaped map of radius [radius] (containing `1 + 3*r*(r+1)` cells).
  factory HexMap.radial(int radius, T Function(HexCoord coord) builder) {
    final map = HexMap<T>();
    for (final c in HexCoord.zero.spiral(radius)) {
      map[c] = builder(c);
    }
    return map;
  }

  /// Creates a rectangular offset grid (`odd-r` layout) converted to axial coordinates.
  factory HexMap.rectOddR(
    int columns,
    int rows,
    T Function(int col, int row, HexCoord coord) builder,
  ) {
    final map = HexMap<T>();
    for (int row = 0; row < rows; row++) {
      final offset = row >> 1;
      for (int col = 0; col < columns; col++) {
        final q = col - offset;
        final r = row;
        final c = HexCoord(q, r);
        map[c] = builder(col, row, c);
      }
    }
    return map;
  }

  int get length => _cells.length;
  bool get isEmpty => _cells.isEmpty;
  bool get isNotEmpty => _cells.isNotEmpty;

  Iterable<HexCoord> get coords => _cells.keys;
  Iterable<T> get values => _cells.values;
  Iterable<MapEntry<HexCoord, T>> get entries => _cells.entries;

  T? operator [](HexCoord coord) => _cells[coord];
  void operator []=(HexCoord coord, T value) => _cells[coord] = value;

  T? get(HexCoord coord) => _cells[coord];
  void set(HexCoord coord, T value) => _cells[coord] = value;
  bool contains(HexCoord coord) => _cells.containsKey(coord);
  T? remove(HexCoord coord) => _cells.remove(coord);
  void clear() => _cells.clear();

  /// Returns all existing neighbor coordinates present in this map.
  List<HexCoord> neighborCoords(HexCoord coord) => [
        for (final n in coord.neighbors)
          if (_cells.containsKey(n)) n,
      ];

  List<MapEntry<HexCoord, T>> neighborEntries(HexCoord coord) => [
        for (final n in coord.neighbors)
          if (_cells.containsKey(n)) MapEntry(n, _cells[n] as T),
      ];

  List<HexCoord> coordsWhere(bool Function(HexCoord coord, T value) predicate) => [
        for (final e in _cells.entries)
          if (predicate(e.key, e.value)) e.key,
      ];
}
