import argparse
import json
from pathlib import Path
from .io import load_problem, save_plan, plot_plan, load_validated_plan
from .planners import SuccessiveConvexification


def main():
    parser = argparse.ArgumentParser(description='Virtual rocket PDG with scaled Crazyflie translation; flight requires mission --execute')
    parser.add_argument('command', choices=['plan', 'simulate', 'mission'])
    parser.add_argument('--config', default='config/rocket-emulation.yaml')
    parser.add_argument('--height', type=float, help='Takeoff rise above configured ground-body height [m]')
    parser.add_argument('--output', default='output/nominal')
    parser.add_argument('--execute', action='store_true', help='Publish actual ROS flight commands (mission only)')
    parser.add_argument('--plan-file',help='Reuse an offline plan for this exact mission/model after independent validation; avoid solving on the flight battery')
    parser.add_argument('--control-smoothing',type=float,default=0.,help='Nonnegative normalized control-slew penalty for a fresh solve (default 0)')
    parser.add_argument('--controller-trial',action='store_true',help='Explicit calibrated Vicon-only PDG trial: omit host voltage and descent tracking-error aborts; retain startup, telemetry and firmware checks')
    args = parser.parse_args()
    if args.execute and args.command != 'mission':
        parser.error('--execute is only valid with mission')
    if args.plan_file and args.command!='mission':parser.error('--plan-file is only valid with mission')
    if args.plan_file and args.control_smoothing:parser.error('--control-smoothing applies to fresh solves, not --plan-file')
    if args.controller_trial and args.command!='mission':parser.error('--controller-trial is only valid with mission')
    import yaml
    selected = yaml.safe_load(Path(args.config).read_text())
    if selected.get('planner', {}).get('model') == 'rocket_6dof':
        from .rocket.cli import run
        return run(args)
    if selected.get('planner', {}).get('model') not in (None, 'quadrotor_6dof'):
        parser.error('Unknown planner.model')
    vehicle, problem, config = load_problem(args.config, args.height)
    config['execution']['operator_controller_trial']=args.controller_trial
    from .ground_health import validate_controller_trial_mode
    validate_controller_trial_mode(config['execution'],hover=False)
    plan = load_validated_plan(args.plan_file,vehicle,problem) if args.plan_file else SuccessiveConvexification(vehicle,control_smoothing_weight=args.control_smoothing).solve(problem)
    directory = Path(args.output); directory.mkdir(parents=True, exist_ok=True)
    save_plan(directory/'plan.json', plan, vehicle, problem)
    if not (args.execute and args.plan_file):plot_plan(directory/'trajectory.png', plan, vehicle)
    print(json.dumps(dict(converged=plan.converged, duration=plan.times[-1],
                         iterations=len(plan.diagnostics['history']),
                         validation=plan.diagnostics['validation'],operator_controller_trial=args.controller_trial), indent=2))
    if not plan.converged:
        raise SystemExit('Plan rejected. No flight commands sent.')
    if args.command == 'mission':
        if not args.execute:
            print('DRY RUN: plan valid; would take off, track descent, and request nominal landing. No ROS connection.')
        else:
            (directory/'execution.json').write_text(json.dumps(config['execution'],indent=2)+'\n')
            from .ros_mission import run_mission
            run_mission(plan, vehicle, config['execution'], directory)
