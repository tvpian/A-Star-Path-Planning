import time
import numpy as np
from map import Map
from node import Node, ActionSet
from solver import AStarSolver
from a_star_ros import ROSPublisher
import readline
import pandas as pd

def main():
    # map = Map(width=600, height=200)

    # while True:
    #     try:
    #         s_x, s_y, s_theta = input("\nEnter start state (x y theta) : ").split()
    #         if int(s_x) < 0 or int(s_y) < 0 or int(s_theta) < 0:
    #             print("\nStart state must be non-negative. Try again.")
    #             continue
    #         x, y = input("Enter end state (x y) : ").split()
    #         if int(x) < 0 or int(y) < 0:
    #             print("\nEnd state must be non-negative. Try again.")
    #             continue

    #         clearance = input("Enter clearance (clearance) : ")
    #         if int(clearance) < 0:
    #             print("\nClearance and radius must be non-negative. Try again.")
    #             continue
    #         map.set_clearance_radius(int(clearance))

    #         RPM1, RPM2 = input("Enter RPM1 and RPM2 (RPM1, RPM2) : ").split()
    #         if int(RPM1) < 0 or int(RPM2) < 0:
    #             print("\nRPM1 and RPM2 must be non-negative. Try again.")
    #             continue
    #         RPM1 = 2 * np.pi * int(RPM1) / 60
    #         RPM2 = 2 * np.pi * int(RPM2) / 60
    #         action_set = ActionSet(RPM1=RPM1, RPM2=RPM2, map=map)
    #         Node.set_resolution((0.5, 0.5, 30))
    #         Node.set_actionset(action_set)

    #         start = Node((int(s_x), int(s_y), int(s_theta)), 0, None)
    #         end = Node((int(x), int(y), 0), np.inf, None)

    #         if map.is_valid(start.state[0], start.state[1]) and map.is_valid(end.state[0], end.state[1]):
    #             print("\nStarting search...")
    #             break
            
    #         print("\nStart or end state is in obstacle. Try again.")
    #     except Exception:
    #         print(Exception.with_traceback())
    #         print("\nIncorrect input format. Try again.")
    #         continue

    # solver = AStarSolver(start, end, map)
    # start_time = time.time()
    # path = solver.solve()
    # end_time = time.time()

    # if path is None:
    #     print("\nNo path found.")
    #     return

    # print(f"\nFound path in {end_time - start_time} seconds.")
    # print("Distance from state to goal : ", path[-1].cost_to_come)

    # nodes = solver.get_explored_nodes()
    # print(f"Final node in the path : {path[-1].state}")
    # print(f"Number of nodes explored : {len(nodes)}")

    # publisher = ROSPublisher(action_set.r, action_set.l)
    data = pd.read_csv('file1.csv')
    data = data.iloc[:, 1:]
    theta = data['theta']
    rpm1 = data['rpm1']
    rpm2 = data['rpm2']
    publisher = ROSPublisher(3.3, 16)

    # for node in path:
    #     if node.parent_action is None:
    #         continue
    #     vel_mag, theta_dot = publisher.vel_calc(node.state[2], node.parent_action[0], node.parent_action[1])
    #     publisher.publish_velocity(vel_mag, theta_dot)

    for i in range(len(theta)):
        print(i)
        vel_mag, theta_dot = publisher.vel_calc(theta[i], rpm1[i], rpm2[i])
        publisher.publish_velocity(vel_mag, theta_dot)

    publisher.publish_velocity(0, 0)

if __name__ == "__main__":

    """
        Github link: https://github.com/abhijaysingh/ENPM661-Project-3
    
    """
    main()