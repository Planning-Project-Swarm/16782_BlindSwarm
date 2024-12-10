import argparse
import os
from swarm_io import SwarmIO
from visualizer_cbs import Visualizer
from cbs_rrt import CBS

def main():
	parser = argparse.ArgumentParser(description="Process map and leader plan files.")
	parser.add_argument('map_file', type=str, help='Path to the map file')
	parser.add_argument('leader_plan_file', type=str, help='Path to the leader plan file')
	parser.add_argument('formations_folder', type=str, help='Path to the formations folder')
	args = parser.parse_args()
	
	map_file = os.path.join("maps/", args.map_file)
	plan_file = os.path.join("plans/", args.leader_plan_file)

	output_folder = "output/"
	base_name = os.path.splitext(os.path.basename(args.map_file))[0]
	output_file = os.path.join(output_folder, f"{base_name}_formations_output.txt")
	output_viz_file = os.path.join(output_folder, "cbs_formation_viz.mp4")
	
	swarmio = SwarmIO()
    
	robots, _, obstacles = swarmio.read_map_file(map_file)
	# formation_infos: formation name and time
	formation_infos, leader = swarmio.readFormation(map_file) # to allow backwards compat
	
	# leader's plan
	with open(plan_file, 'r') as file:
		plan_lines = [line.strip().split(',') for line in file.readlines() if line.strip()]
	
	# A dict of dict to store formation timestep, formation name and poses
	formations = dict()

	for formation_info in formation_infos:
		file_name = formation_info['filename']
		formation_t = formation_info['time']
		# print(file_name, formation_t)
		formation_name = file_name.strip()
		formation_file = os.path.join(args.formations_folder, formation_name + ".txt")
		formation_pos = swarmio.read_formation_file(formation_file)
		formations[formation_t] = {"name": formation_name, "pos": formation_pos}

	output = {robot['id']: [] for robot in robots}
	output_goals = {robot['id']: [] for robot in robots}

	robot_r = 0.5
	rand_area = [0, 40]
	leader_poses = []

	for leader_plan_line in plan_lines:

		x, y, orientation, t = map(float, leader_plan_line)
		leader_poses.append([x, y, orientation, t])

		if formations.get(t):
			# update formation
			print("Formation changed")
			cur_formation = formations[t]

		goals = []
		# print(cur_formation['name'])
		for robot_id, pos in enumerate(cur_formation['pos'], start=1):

			goal_x = x + pos['x']
			goal_y = y + pos['y']
			
			# Check if the goal position falls within any obstacle
			for obs in obstacles:
				# print(obs)
				x1, y1, x2, y2 = obs['x1'], obs['y1'], obs['x2'], obs['y2']
				if x1 - 1 <= goal_x <= x2 + 1 and y1 - 1 <= goal_y <= y2 + 1:
					# Adjust the position to minimize least-squares distance by projecting to the nearest edge
					dx1, dx2 = abs(goal_x - x1), abs(goal_x - x2)
					dy1, dy2 = abs(goal_y - y1), abs(goal_y - y2)
					min_dist = min(dx1, dx2, dy1, dy2)
					
					if min_dist == dx1:
						goal_x = x1 - 1
					elif min_dist == dx2:
						goal_x = x2 + 1
					elif min_dist == dy1:
						goal_y = y1 - 1
					else:
						goal_y = y2 + 1
			
			goals.append({
				'id': robot_id,
				'x': goal_x,
				'y': goal_y,
				'orientation': orientation
			})
			output_goals[robot_id].append([goal_x, goal_y, orientation, t])

		# print(goals)
		solver = CBS(robots, goals, obstacles, rand_area, robot_r)
		solver.planning() # Generate set of solutions for the current state
		print(x, y, orientation, t)
		
		for robot in solver.paths:

			# update robot pose
			robots[robot-1]['x'] = solver.paths[robot][1][0]
			robots[robot-1]['y'] = solver.paths[robot][1][1]
			robots[robot-1]['orientation'] = solver.paths[robot][1][2]
			output[robot].append([solver.paths[robot][1][0], solver.paths[robot][1][1], solver.paths[robot][1][2], t])

	
	# print(output)

	# save the output to file
	swarmio.write_cbs_output_file(output, output_file)

	#hack the visualizer to visualize each frame
	vis = Visualizer()
	vis.viz_formations(output, output_goals, leader_poses, rand_area * 2, obstacles, robot_r, save_animation=False)
	vis.viz_formations(output, output_goals, leader_poses, rand_area * 2, obstacles, robot_r, save_animation=True, output_file=output_viz_file)


if __name__ == "__main__":
	main()