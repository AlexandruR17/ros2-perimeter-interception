"""SDF for animated human actors driven by the ActorDriver Gazebo plugin."""

from __future__ import annotations

MESHES = 'model://fob3_human/meshes'


def actor_sdf(name: str, x: float, y: float, yaw: float = 0.0, run_speed: float = 1.9) -> str:
    """Actor with walk, run and stand animations, spawned at (x, y)."""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <actor name="{name}">
    <pose>{x:.3f} {y:.3f} 1.0 0 0 {yaw:.4f}</pose>
    <skin><filename>{MESHES}/walk.dae</filename></skin>
    <animation name="walk"><filename>{MESHES}/walk.dae</filename><interpolate_x>true</interpolate_x></animation>
    <animation name="run"><filename>{MESHES}/run.dae</filename><interpolate_x>true</interpolate_x></animation>
    <animation name="stand"><filename>{MESHES}/stand.dae</filename></animation>
    <plugin filename="ActorDriver" name="perimeter_defense::ActorDriver">
      <run_speed>{run_speed}</run_speed>
    </plugin>
  </actor>
</sdf>
"""
