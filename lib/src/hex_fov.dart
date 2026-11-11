import 'hex_coord.dart';

/// Field-of-view & line-of-sight visibility calculations on axial hex grids.
abstract final class HexFov {
  /// Returns true if a line-of-sight ray from [origin] to [target] is unobstructed.
  ///
  /// Intermediate opaque tiles (excluding [origin], and optionally [target] unless
  /// [blockTargetTileIfOpaque] is true) terminate sight.
  static bool hasLineOfSight(
    HexCoord origin,
    HexCoord target, {
    required bool Function(HexCoord coord) blocksSight,
    bool blockTargetTileIfOpaque = false,
  }) {
    final line = origin.lineTo(target);
    for (int i = 1; i < line.length; i++) {
      final cell = line[i];
      if (cell == target && !blockTargetTileIfOpaque) {
        return true;
      }
      if (blocksSight(cell)) {
        return false;
      }
    }
    return true;
  }

  /// Computes all tiles visible from [origin] within [radius] using hex raycasting.
  ///
  /// Opaque blocker tiles themselves remain visible so walls/pillars render naturally,
  /// while tiles behind them remain hidden.
  static Set<HexCoord> computeVisible(
    HexCoord origin, {
    required int radius,
    required bool Function(HexCoord coord) blocksSight,
    bool Function(HexCoord coord)? inBounds,
  }) {
    if (radius < 0) return const {};
    final visible = <HexCoord>{origin};
    for (final candidate in origin.spiral(radius)) {
      if (candidate == origin) continue;
      if (inBounds != null && !inBounds(candidate)) continue;
      if (hasLineOfSight(
        origin,
        candidate,
        blocksSight: blocksSight,
        blockTargetTileIfOpaque: false,
      )) {
        visible.add(candidate);
      }
    }
    return visible;
  }
}
