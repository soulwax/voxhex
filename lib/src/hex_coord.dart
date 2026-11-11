import 'dart:math' as math;

/// Axial (`q`, `r`) hexagonal grid coordinate with implicit cube third axis `s = -q - r`.
final class HexCoord {
  final int q;
  final int r;

  const HexCoord(this.q, this.r);

  static const zero = HexCoord(0, 0);

  /// Cube constraint `q + r + s == 0`.
  int get s => -q - r;

  static const List<HexCoord> directions = [
    HexCoord(1, 0),
    HexCoord(1, -1),
    HexCoord(0, -1),
    HexCoord(-1, 0),
    HexCoord(-1, 1),
    HexCoord(0, 1),
  ];

  static const List<HexCoord> diagonalDirections = [
    HexCoord(2, -1),
    HexCoord(1, -2),
    HexCoord(-1, -1),
    HexCoord(-2, 1),
    HexCoord(-1, 2),
    HexCoord(1, 1),
  ];

  HexCoord operator +(HexCoord other) => HexCoord(q + other.q, r + other.r);
  HexCoord operator -(HexCoord other) => HexCoord(q - other.q, r - other.r);
  HexCoord operator *(int factor) => HexCoord(q * factor, r * factor);
  HexCoord operator -() => HexCoord(-q, -r);

  int get length => (q.abs() + r.abs() + s.abs()) ~/ 2;

  int distanceTo(HexCoord other) => (this - other).length;

  HexCoord neighbor(int direction) => this + directions[(direction % 6 + 6) % 6];

  List<HexCoord> get neighbors => [
        for (int d = 0; d < 6; d++) neighbor(d),
      ];

  HexCoord diagonalNeighbor(int direction) =>
      this + diagonalDirections[(direction % 6 + 6) % 6];

  /// Rotates 60 degrees counter-clockwise in cube space around [pivot].
  HexCoord rotateLeft([HexCoord pivot = zero]) {
    final v = this - pivot;
    // Cube left rotation: (-r, -s, -q)
    return pivot + HexCoord(-v.r, -v.s);
  }

  /// Rotates 60 degrees clockwise in cube space around [pivot].
  HexCoord rotateRight([HexCoord pivot = zero]) {
    final v = this - pivot;
    // Cube right rotation: (-s, -q, -r)
    return pivot + HexCoord(-v.s, -v.q);
  }

  /// Rounds continuous fractional axial coords (`qf`, `rf`) to nearest [HexCoord].
  static HexCoord round(double qf, double rf) {
    final sf = -qf - rf;
    var rq = qf.round();
    var rr = rf.round();
    final rs = sf.round();

    final dq = (rq - qf).abs();
    final dr = (rr - rf).abs();
    final ds = (rs - sf).abs();

    if (dq > dr && dq > ds) {
      rq = -rr - rs;
    } else if (dr > ds) {
      rr = -rq - rs;
    }
    return HexCoord(rq, rr);
  }

  /// All hex tiles lying on the exact perimeter ring of distance [radius].
  List<HexCoord> ring(int radius) {
    if (radius < 0) throw ArgumentError.value(radius, 'radius', 'Must be >= 0');
    if (radius == 0) return [this];
    final result = <HexCoord>[];
    var hex = this + (directions[4] * radius);
    for (int side = 0; side < 6; side++) {
      for (int step = 0; step < radius; step++) {
        result.add(hex);
        hex = hex.neighbor(side);
      }
    }
    return result;
  }

  /// Filled spiral of all hex coordinates within distance `<= maxRadius` centered at `this`.
  List<HexCoord> spiral(int maxRadius) {
    if (maxRadius < 0) throw ArgumentError.value(maxRadius, 'maxRadius', 'Must be >= 0');
    final result = <HexCoord>[this];
    for (int k = 1; k <= maxRadius; k++) {
      result.addAll(ring(k));
    }
    return result;
  }

  /// Rasterizes a straight hex line from `this` to [target] inclusive.
  List<HexCoord> lineTo(HexCoord target) {
    final n = distanceTo(target);
    if (n == 0) return [this];
    final results = <HexCoord>[];
    // Deterministic slight epsilon nudge breaks exact edge ties cleanly
    const epsQ = 1e-6;
    const epsR = 2e-6;
    final aq = q + epsQ;
    final ar = r + epsR;
    final bq = target.q + epsQ;
    final br = target.r + epsR;

    for (int i = 0; i <= n; i++) {
      final t = i / n;
      results.add(round(aq + (bq - aq) * t, ar + (br - ar) * t));
    }
    return results;
  }

  Map<String, int> toJson() => {'q': q, 'r': r};

  factory HexCoord.fromJson(Map<String, Object?> json) =>
      HexCoord((json['q'] as num).toInt(), (json['r'] as num).toInt());

  @override
  bool operator ==(Object other) =>
      identical(this, other) || other is HexCoord && q == other.q && r == other.r;

  @override
  int get hashCode => Object.hash(q, r);

  @override
  String toString() => 'HexCoord($q, $r)';
}

/// Visual layout geometry for hexagons.
enum HexOrientation {
  pointyTop,
  flatTop,
}

/// Converts between axial [HexCoord]s and 2D Euclidean pixel/world positions.
final class HexLayout {
  final HexOrientation orientation;
  final double sizeX;
  final double sizeY;
  final double originX;
  final double originY;

  const HexLayout({
    this.orientation = HexOrientation.pointyTop,
    double size = 1.0,
    double? sizeX,
    double? sizeY,
    this.originX = 0.0,
    this.originY = 0.0,
  })  : sizeX = sizeX ?? size,
        sizeY = sizeY ?? size;

  static final double _sqrt3 = math.sqrt(3.0);

  ({double x, double y}) toPixel(HexCoord h) {
    if (orientation == HexOrientation.pointyTop) {
      final x = (_sqrt3 * h.q + _sqrt3 * 0.5 * h.r) * sizeX + originX;
      final y = (1.5 * h.r) * sizeY + originY;
      return (x: x, y: y);
    } else {
      final x = (1.5 * h.q) * sizeX + originX;
      final y = (_sqrt3 * 0.5 * h.q + _sqrt3 * h.r) * sizeY + originY;
      return (x: x, y: y);
    }
  }

  HexCoord fromPixel(double px, double py) {
    final nx = (px - originX) / sizeX;
    final ny = (py - originY) / sizeY;
    if (orientation == HexOrientation.pointyTop) {
      final q = (_sqrt3 / 3.0 * nx - 1.0 / 3.0 * ny);
      final r = (2.0 / 3.0 * ny);
      return HexCoord.round(q, r);
    } else {
      final q = (2.0 / 3.0 * nx);
      final r = (-1.0 / 3.0 * nx + _sqrt3 / 3.0 * ny);
      return HexCoord.round(q, r);
    }
  }

  /// Returns the 6 vertex coordinates in screen/world space around [h].
  List<({double x, double y})> polygonCorners(HexCoord h) {
    final center = toPixel(h);
    final startAngle = orientation == HexOrientation.pointyTop ? 0.5 : 0.0;
    return [
      for (int i = 0; i < 6; i++)
        (
          x: center.x + sizeX * math.cos(2.0 * math.pi * (startAngle + i) / 6.0),
          y: center.y + sizeY * math.sin(2.0 * math.pi * (startAngle + i) / 6.0),
        ),
    ];
  }
}
