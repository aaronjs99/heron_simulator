#!/usr/bin/env python3
import os
import unittest
import xml.etree.ElementTree as ET


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
WORLD_PATH = os.path.join(REPO_ROOT, "worlds", "fathomwerx_pool.world")


class FathomwerxPoolWorldTests(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(WORLD_PATH).getroot()
        self.world = self.root.find("world")
        self.assertIsNotNone(self.world)

    def _model_names(self):
        return [m.attrib.get("name", "") for m in self.world.findall("model")]

    def _pose_xyz(self, model_name):
        model = self.world.find(f"model[@name='{model_name}']")
        self.assertIsNotNone(model)
        pose_vals = [float(v) for v in (model.findtext("pose") or "").split()]
        self.assertGreaterEqual(len(pose_vals), 3)
        return pose_vals[0], pose_vals[1], pose_vals[2]

    def _collision_size_xyz(self, model_name):
        model = self.world.find(f"model[@name='{model_name}']")
        self.assertIsNotNone(model)
        size_text = model.findtext("link/collision/geometry/box/size")
        self.assertIsNotNone(size_text)
        size_vals = [float(v) for v in size_text.split()]
        self.assertGreaterEqual(len(size_vals), 3)
        return size_vals[0], size_vals[1], size_vals[2]

    def test_pool_physics_matches_inspection_dock_profile(self):
        physics = self.world.find("physics")
        self.assertIsNotNone(physics)
        self.assertEqual((physics.findtext("max_step_size") or "").strip(), "0.001")
        self.assertEqual((physics.findtext("real_time_factor") or "").strip(), "1")
        self.assertEqual(
            (physics.findtext("real_time_update_rate") or "").strip(), "1000"
        )

    def test_pool_world_contains_only_pool_assets_by_default(self):
        names = self._model_names()
        required = {
            "pool_floor",
            "pool_water_surface",
            "pool_wall_north",
            "pool_wall_south",
            "pool_wall_east",
            "pool_wall_west",
        }
        self.assertTrue(required.issubset(set(names)))

    def test_pool_world_excludes_harbor_and_internal_targets(self):
        names = self._model_names()
        forbidden_prefixes = (
            "harbor_",
            "dock_",
            "service_boat",
            "inspection_target_",
            "porthole_",
            "wall_south_boundary",
        )
        for name in names:
            for prefix in forbidden_prefixes:
                self.assertFalse(
                    name.startswith(prefix) or name == prefix,
                    msg=f"unexpected model in pool world: {name}",
                )

    def test_pool_has_no_internal_inspection_targets(self):
        names = self._model_names()
        self.assertNotIn("inspection_target_north", names)
        self.assertNotIn("inspection_target_south", names)

    def test_pool_floor_top_is_below_water_surface(self):
        _, _, floor_z = self._pose_xyz("pool_floor")
        _, _, floor_h = self._collision_size_xyz("pool_floor")
        floor_top_z = floor_z + (floor_h * 0.5)
        self.assertLess(floor_top_z, 0.0)
        self.assertAlmostEqual(floor_top_z, -3.66, places=2)

    def test_pool_walls_reach_floor_and_top_slightly_above_surface(self):
        for name in (
            "pool_wall_north",
            "pool_wall_south",
            "pool_wall_east",
            "pool_wall_west",
        ):
            _, _, wall_z = self._pose_xyz(name)
            _, _, wall_h = self._collision_size_xyz(name)
            wall_bottom = wall_z - (wall_h * 0.5)
            wall_top = wall_z + (wall_h * 0.5)
            self.assertAlmostEqual(wall_bottom, -3.66, places=2)
            self.assertGreaterEqual(wall_top, 0.10)
            self.assertLessEqual(wall_top, 0.20)

    def test_pool_usable_tank_dimensions_match_fathomwerx_specs(self):
        # Inner water-space dimensions should be exactly 7.62m x 15.24m.
        _, _, _ = self._pose_xyz("pool_wall_east")
        _, _, _ = self._pose_xyz("pool_wall_west")
        _, _, _ = self._pose_xyz("pool_wall_north")
        _, _, _ = self._pose_xyz("pool_wall_south")

        ex, _, _ = self._pose_xyz("pool_wall_east")
        wx, _, _ = self._pose_xyz("pool_wall_west")
        _, _, _ = self._collision_size_xyz("pool_wall_east")
        _, _, _ = self._collision_size_xyz("pool_wall_west")
        east_thickness, _, _ = self._collision_size_xyz("pool_wall_east")
        west_thickness, _, _ = self._collision_size_xyz("pool_wall_west")
        inner_x_max = ex - (east_thickness * 0.5)
        inner_x_min = wx + (west_thickness * 0.5)
        width = inner_x_max - inner_x_min

        _, ny, _ = self._pose_xyz("pool_wall_north")
        _, sy, _ = self._pose_xyz("pool_wall_south")
        _, north_thickness, _ = self._collision_size_xyz("pool_wall_north")
        _, south_thickness, _ = self._collision_size_xyz("pool_wall_south")
        inner_y_max = ny - (north_thickness * 0.5)
        inner_y_min = sy + (south_thickness * 0.5)
        length = inner_y_max - inner_y_min

        self.assertAlmostEqual(width, 7.62, places=2)
        self.assertAlmostEqual(length, 15.24, places=2)

    def test_world_origin_is_inside_pool_and_near_short_edge(self):
        ex, _, _ = self._pose_xyz("pool_wall_east")
        wx, _, _ = self._pose_xyz("pool_wall_west")
        east_wall_thickness, _, _ = self._collision_size_xyz("pool_wall_east")
        west_wall_thickness, _, _ = self._collision_size_xyz("pool_wall_west")
        _, ny, _ = self._pose_xyz("pool_wall_north")
        _, sy, _ = self._pose_xyz("pool_wall_south")
        _, north_thickness, _ = self._collision_size_xyz("pool_wall_north")
        _, south_thickness, _ = self._collision_size_xyz("pool_wall_south")

        inner_x_min = wx + (west_wall_thickness * 0.5)
        inner_x_max = ex - (east_wall_thickness * 0.5)
        inner_y_min = sy + (south_thickness * 0.5)
        inner_y_max = ny - (north_thickness * 0.5)

        self.assertGreater(0.0, inner_x_min)
        self.assertLess(0.0, inner_x_max)
        self.assertGreater(0.0, inner_y_min)
        self.assertLess(0.0, inner_y_max)

        dist_west = abs(0.0 - inner_x_min)
        dist_east = abs(inner_x_max - 0.0)
        dist_south = abs(0.0 - inner_y_min)
        dist_north = abs(inner_y_max - 0.0)
        nearest = min(dist_west, dist_east, dist_south, dist_north)
        self.assertAlmostEqual(nearest, dist_south, places=2)
        self.assertGreaterEqual(nearest, 1.0)
        self.assertLessEqual(nearest, 1.5)

    def test_water_surface_visual_and_collision_match_expected_profile(self):
        water = self.world.find("model[@name='pool_water_surface']")
        self.assertIsNotNone(water)
        pose_vals = [float(v) for v in (water.findtext("pose") or "").split()]
        self.assertGreaterEqual(len(pose_vals), 3)
        self.assertAlmostEqual(pose_vals[2], 0.0, places=6)

        visual_plane = water.find("link/visual/geometry/plane")
        self.assertIsNotNone(visual_plane)
        collision_plane = water.find("link/collision/geometry/plane")
        self.assertIsNotNone(collision_plane)
        visual_size_text = visual_plane.findtext("size")
        collision_size_text = collision_plane.findtext("size")
        self.assertTrue(visual_size_text)
        self.assertTrue(collision_size_text)
        visual_size = [float(v) for v in visual_size_text.split()[:2]]
        collision_size = [float(v) for v in collision_size_text.split()[:2]]
        self.assertEqual(visual_size, [100.0, 100.0])
        self.assertEqual(collision_size, [100.0, 100.0])

        material = water.find("link/visual/material")
        self.assertIsNotNone(material)
        self.assertEqual(
            (material.findtext("ambient") or "").strip(), "0.1 0.3 0.5 0.8"
        )
        self.assertEqual(
            (material.findtext("diffuse") or "").strip(), "0.1 0.4 0.6 0.8"
        )
        self.assertEqual(
            (material.findtext("specular") or "").strip(), "0.8 0.8 0.8 1"
        )

        mu = water.findtext("link/collision/surface/friction/ode/mu")
        mu2 = water.findtext("link/collision/surface/friction/ode/mu2")
        self.assertEqual((mu or "").strip(), "0.01")
        self.assertEqual((mu2 or "").strip(), "0.01")

    def test_pool_scene_matches_ocean_background_profile(self):
        scene = self.world.find("scene")
        self.assertIsNotNone(scene)
        ambient_text = (scene.findtext("ambient") or "").strip()
        background_text = (scene.findtext("background") or "").strip()
        self.assertEqual(ambient_text, "0.01 0.01 0.01 1.0")
        self.assertEqual(background_text, "0.70 0.84 0.95 1.0")

    def test_pool_world_contains_expected_lighting_stack(self):
        lights = {
            light.attrib.get("name", ""): light
            for light in self.world.findall("light")
        }
        self.assertIn("sun1", lights)
        self.assertIn("sun_diffuse", lights)
        self.assertIn("pool_overhead_fill", lights)
        self.assertEqual(
            (lights["sun1"].findtext("direction") or "").strip(),
            "0.3 0.3 -1",
        )
        self.assertEqual(
            (lights["sun_diffuse"].findtext("direction") or "").strip(),
            "-0.3 -0.3 -1",
        )


if __name__ == "__main__":
    unittest.main()
