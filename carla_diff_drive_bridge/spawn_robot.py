#!/usr/bin/env python3
"""
Spawn a simple actor in CARLA to act as the differential drive robot.

Uses a small vehicle blueprint with physics disabled (for kinematic control).
You can swap the blueprint for any actor type — the bridge doesn't care.
"""

import argparse
import random
import carla


def main():
    parser = argparse.ArgumentParser(description='Spawn a diff-drive robot actor in CARLA')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--blueprint', default='vehicle.tesla.cybertruck',
                        help='CARLA blueprint ID. Use any vehicle or walker.')
    parser.add_argument('--role-name', default='ego_robot',
                        help='role_name attribute (bridge uses this to find the actor)')
    parser.add_argument('--spawn-point', type=int, default=None,
                        help='Spawn point index (None = random)')
    parser.add_argument('--disable-physics', action='store_true', default=False,
                        help='Disable physics simulation (pure kinematic, no collisions)')
    parser.add_argument('--enable-physics', action='store_true', default=True,
                        help='Keep physics enabled (default — enables collision detection)')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find(args.blueprint)
    if bp is None:
        print(f'Blueprint "{args.blueprint}" not found. Available vehicles:')
        for b in bp_lib.filter('vehicle.*'):
            print(f'  {b.id}')
        return

    # Set role_name so the bridge can find this actor
    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', args.role_name)

    # Pick spawn point
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print('No spawn points available on this map!')
        return

    if args.spawn_point is not None:
        if args.spawn_point >= len(spawn_points):
            print(f'Spawn point {args.spawn_point} out of range (0-{len(spawn_points)-1})')
            return
        spawn_tf = spawn_points[args.spawn_point]
    else:
        spawn_tf = random.choice(spawn_points)

    actor = world.spawn_actor(bp, spawn_tf)
    print(f'Spawned: id={actor.id}, type={actor.type_id}, role_name={args.role_name}')
    print(f'Location: ({spawn_tf.location.x:.1f}, {spawn_tf.location.y:.1f}, {spawn_tf.location.z:.1f})')

    # Disable physics for pure kinematic control
    if args.disable_physics and not args.enable_physics:
        actor.set_simulate_physics(False)
        print('Physics DISABLED (kinematic mode). Bridge controls position directly.')
    else:
        print('Physics ENABLED. Bridge sets target velocity; CARLA simulates dynamics.')

    print(f'\nActor is live. Bridge will auto-detect it via role_name="{args.role_name}".')
    print('Press Ctrl+C to keep it alive, or run your bridge node now.')

    try:
        while True:
            world.wait_for_tick()
    except KeyboardInterrupt:
        actor.destroy()
        print(f'\nActor {actor.id} destroyed.')


if __name__ == '__main__':
    main()
