import argparse
from swarm_io import SwarmIO
from visualizer_cbs import Visualizer
from cbs_rrt import CBS

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description="Process map and plan files.")
	parser.add_argument('--map', type=str, required=True, help='Path to the map file')
	parser.add_argument('--plan', type=str, required=True, help='Path to the plan file')
	args = parser.parse_args()
	
	map_file = "maps/" + args.map
	plan_file = "plans/" + args.plan
	
	print(f"Map file: {map_file}")
	print(f"Plan file: {plan_file}")
	
	swarmio = SwarmIO()
    
	robots, _, obstacles = swarmio.read_map_file(map_file)
	formations, leader = swarmio.readFormation(map_file) #to allow backwards compat
	
	with open(plan_file, 'r') as file:
		plan_lines = [line.strip().split(',') for line in file.readlines() if line.strip()]
	
	goals = []
	cur_formation = 0
	formation_timer = formations[0]["time"]
	formation_file = "formations/" + formations[0]["filename"].strip() + ".txt"
	formation_pos = swarmio.read_formation_file(formation_file)
	
	output = []
	cur_pos = 0;
	for position in plan_lines:
		x, y, orientation = map(float, position)
		if(formation_timer == 0):
			cur_formation += 1
			formation_file = "formations/" + formations[cur_formation]["filename"].strip() + ".txt"
			formation_timer = formations[cur_formation]["time"]
			formation_pos = swarmio.read_formation_file(formation_file)
		
		goals = []
		temp_id = 1
		for formation in formation_pos:
			print(formation)
			goal_x = x + formation['x']
			goal_y = y + formation['y']
			
			# Check if the goal position falls within any obstacle
			for obs in obstacles:
				#print(obs)
				x1, y1, x2, y2 = obs['x1'], obs['y1'], obs['x2'], obs['y2']
				if x1 <= goal_x <= x2 and y1 <= goal_y <= y2:
					# Adjust the position to minimize least-squares distance
					if goal_x > x1:
						goal_x = x1 - 0.5
					elif goal_x < x2:
						goal_x = x2 + 0.5
					if goal_y > y1:
						goal_y = y1 - 0.5
					elif goal_y < y2:
						goal_y = y2 + 0.5
			
			goals.append({
				'id': temp_id,
				'x': goal_x,
				'y': goal_y,
				'orientation': orientation
			})
			temp_id += 1
		print(goals)
		solver = CBS(robots, goals, obstacles, [0, 40])
		solver.planning() #generate set of solutions for the current state
		
		formation_timer -= 1

		# Visualize the current frame
		frame = {
			'leader': position,
			'robots': []
		}
		print(solver.paths)
		for robot in solver.paths:
			if(solver.paths[robot][1]):
				robots[robot-1]['x'] = solver.paths[robot][1][0]
				robots[robot-1]['y'] = solver.paths[robot][1][1]
				robots[robot-1]['orientation'] = solver.paths[robot][1][2]
				frame['robots'].append({
					'id': robot,
					'x': robots[robot-1]['x'],
					'y': robots[robot-1]['y'],
					'orientation': robots[robot-1]['orientation']
				})
	
	output.append(frame)
	
	#hack the visualizer to visualize each frame
	vis = Visualizer([0, 40], obstacles)
	for frame in output:
		# Update positions for visualization
		robot_positions = [(robot['x'], robot['y'], robot['orientation']) for robot in frame['robots']]
		leader_pos = (frame['leader'][0], frame['leader'][1], 0)  # Assuming leader orientation is 0
		vis.update(robot_positions, leader_pos)