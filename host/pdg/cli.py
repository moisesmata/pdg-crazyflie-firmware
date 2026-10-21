import argparse
import json
from pathlib import Path
from .io import load_problem, save_plan, plot_plan
from .planners import SuccessiveConvexification


def main():
    parser = argparse.ArgumentParser(description='Crazyflie offboard PDG; simulation is the default')
    parser.add_argument('command', choices=['plan', 'simulate', 'mission'])
    parser.add_argument('--config', default='config/nominal.yaml')
    parser.add_argument('--height', type=float, help='Takeoff rise above configured ground-body height [m]')
    parser.add_argument('--output', default='output/nominal')
    parser.add_argument('--execute', action='store_true', help='Publish actual ROS flight commands (mission only)')
    args = parser.parse_args()
    if args.execute and args.command != 'mission':
        parser.error('--execute is only valid with mission')
    vehicle, problem, config = load_problem(args.config, args.height)
    plan = SuccessiveConvexification(vehicle).solve(problem)
    directory = Path(args.output); directory.mkdir(parents=True, exist_ok=True)
    save_plan(directory/'plan.json', plan, vehicle, problem)
    plot_plan(directory/'trajectory.png', plan, vehicle)
    print(json.dumps(dict(converged=plan.converged, duration=plan.times[-1],
                         iterations=len(plan.diagnostics['history']),
                         validation=plan.diagnostics['validation']), indent=2))
    if not plan.converged:
        raise SystemExit('Plan rejected. No flight commands sent.')
    if args.command == 'mission':
        if not args.execute:
            print('DRY RUN: plan valid; would take off, track descent, and request nominal landing. No ROS connection.')
        else:
            from .ros_mission import run_mission
            run_mission(plan, vehicle, config['execution'], directory)
