import 'package:test/test.dart';
import 'package:voxhex/voxhex.dart';

void main() {
  group('HexCoord math, rings, lines & pixel projection', () {
    test('satisfies cube invariants, rotations, rings and spirals', () {
      const a = HexCoord(2, -1);
      expect(a.q + a.r + a.s, 0);
      expect(a.length, 2);
      expect(a.distanceTo(const HexCoord(-1, 2)), 3);
      expect(a.distanceTo(-a), 4);

      var rot = a;
      for (int i = 0; i < 6; i++) {
        rot = rot.rotateRight();
      }
      expect(rot, a);

      var rotL = a;
      for (int i = 0; i < 6; i++) {
        rotL = rotL.rotateLeft();
      }
      expect(rotL, a);

      expect(HexCoord.zero.ring(0), [HexCoord.zero]);
      expect(HexCoord.zero.ring(1).length, 6);
      expect(HexCoord.zero.ring(2).length, 12);
      expect(HexCoord.zero.spiral(2).length, 19);

      final line = HexCoord.zero.lineTo(const HexCoord(3, -1));
      expect(line.first, HexCoord.zero);
      expect(line.last, const HexCoord(3, -1));
      for (int i = 1; i < line.length; i++) {
        expect(line[i - 1].distanceTo(line[i]), 1);
      }
    });

    test('round-trips pointy-top and flat-top HexLayout conversions', () {
      for (final orient in HexOrientation.values) {
        final layout = HexLayout(orientation: orient, size: 24.0, originX: 100.0, originY: -50.0);
        for (final c in HexCoord.zero.spiral(3)) {
          final px = layout.toPixel(c);
          final back = layout.fromPixel(px.x, px.y);
          expect(back, c);
          expect(layout.polygonCorners(c).length, 6);
        }
      }
    });
  });

  group('HexMap, HexPathfinder & HexFov', () {
    test('A* avoids obstacles and Dijkstra respects terrain costs & Zone of Control', () {
      final map = HexMap<String>.radial(4, (_) => 'plain');
      // Impassable ridge forcing detour
      for (final wall in const [HexCoord(1, 0), HexCoord(1, -1), HexCoord(0, 1)]) {
        map[wall] = 'wall';
      }
      // Swamp cost 2
      map[const HexCoord(0, -1)] = 'swamp';
      // Enemy ZoC tile
      const zocTile = HexCoord(-1, 0);

      final path = HexPathfinder.findPath(
        HexCoord.zero,
        const HexCoord(2, 0),
        isPassable: (c) => map.contains(c) && map[c] != 'wall',
        moveCost: (_, to) => map[to] == 'swamp' ? 2.0 : 1.0,
      );
      expect(path, isNotNull);
      expect(path!.first, HexCoord.zero);
      expect(path.last, const HexCoord(2, 0));
      for (final step in path) {
        expect(map[step], isNot('wall'));
      }

      final reach = HexPathfinder.reachable(
        HexCoord.zero,
        budget: 2.0,
        isPassable: (c) => map.contains(c) && map[c] != 'wall',
        moveCost: (_, to) => map[to] == 'swamp' ? 2.0 : 1.0,
        stopsMovement: (c) => c == zocTile,
      );
      expect(reach.canReach(zocTile), isTrue);
      // Tile behind ZoC (-2, 0) requires expanding past zocTile, which stops movement!
      expect(reach.canReach(const HexCoord(-2, 0)), isFalse);
      // Swamp tile (-1+1? No, (0,-1)) costs 2 so it can be reached but not passed with budget 2
      expect(reach.canReach(const HexCoord(0, -1)), isTrue);
    });

    test('HexFov computes visibility and casts shadows behind opaque hexes', () {
      const pillar = HexCoord(1, 0);
      final visible = HexFov.computeVisible(
        HexCoord.zero,
        radius: 3,
        blocksSight: (c) => c == pillar,
      );

      expect(visible.contains(HexCoord.zero), isTrue);
      expect(visible.contains(pillar), isTrue); // blocker itself is visible
      expect(visible.contains(const HexCoord(2, 0)), isFalse); // directly shadowed!
      expect(visible.contains(const HexCoord(3, 0)), isFalse); // directly shadowed!
      expect(visible.contains(const HexCoord(0, 2)), isTrue); // unobstructed angle
    });
  });

  group('Wave-Function Collapse (WFC) 2D & Hex solvers', () {
    test('solves 2D socket-constrained terrain & road grid cleanly', () {
      final rules = WfcRuleSet<String>.grid2d()
        // Order of sockets: [North, East, South, West]
        ..addSocketTile('grass', ['g', 'g', 'g', 'g'], weight: 4.0)
        ..addSocketTile('road_ns', ['r', 'g', 'r', 'g'], weight: 1.5)
        ..addSocketTile('road_ew', ['g', 'r', 'g', 'r'], weight: 1.5)
        ..addSocketTile('corner_ne', ['r', 'r', 'g', 'g'], weight: 1.0)
        ..addSocketTile('corner_es', ['g', 'r', 'r', 'g'], weight: 1.0)
        ..addSocketTile('corner_sw', ['g', 'g', 'r', 'r'], weight: 1.0)
        ..addSocketTile('corner_wn', ['r', 'g', 'g', 'r'], weight: 1.0);

      final grid = WfcGrid2D.solve<String>(
        width: 8,
        height: 6,
        rules: rules,
        seed: 1337,
        preplaced: {(3, 3): 'road_ns'},
      );

      expect(grid, isNotNull);
      expect(grid!.length, 6);
      expect(grid[3][3], 'road_ns');

      // North neighbor of (3,3) must allow south='r' (so North socket at (3,3) matches South at (3,2))
      expect(['road_ns', 'corner_es', 'corner_sw'], contains(grid[2][3]));
    });

    test('solves hexagonal WFC map respecting biome transitions', () {
      // 3-tier concentric transition rules: sea <-> coast <-> forest (no direct sea <-> forest!)
      final rules = WfcRuleSet<String>.hex()
        ..addTile('sea', weight: 2.0)
        ..addTile('coast', weight: 3.0)
        ..addTile('forest', weight: 2.0);

      for (int d = 0; d < 6; d++) {
        rules.allow('sea', d, 'sea');
        rules.allow('sea', d, 'coast');
        rules.allow('coast', d, 'coast');
        rules.allow('coast', d, 'forest');
        rules.allow('forest', d, 'forest');
      }

      final ringCoords = HexCoord.zero.spiral(3);
      final solved = WfcHexGrid.solve<String>(
        coords: ringCoords,
        rules: rules,
        seed: 77,
        preplaced: {HexCoord.zero: 'forest'},
      );

      expect(solved, isNotNull);
      expect(solved![HexCoord.zero], 'forest');
      // Verify zero direct sea <-> forest edges anywhere on map!
      for (final c in ringCoords) {
        final selfTile = solved[c];
        for (final n in c.neighbors) {
          final nbTile = solved[n];
          if (nbTile != null) {
            expect(
              (selfTile == 'sea' && nbTile == 'forest') ||
                  (selfTile == 'forest' && nbTile == 'sea'),
              isFalse,
            );
          }
        }
      }
    });
  });
}
