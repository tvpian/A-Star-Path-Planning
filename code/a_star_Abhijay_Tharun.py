import matplotlib.pyplot as plt
import time
import numpy as np
from map import Map
from node import Node, ActionSet
from solver import AStarSolver
from visualizer import Visualizer
import readline

def main():
    map = Map(width=600, height=250, clearance=5)
    action_set = ActionSet()
    Node.set_resolution((1, 1, 30))
    Node.set_actionset(action_set)
    
    while True:
        try:
            x, y, theta = input("\nEnter start state (x y theta) : ").split()
            start = Node((int(x), int(y), int(theta)), 0, None)

            x, y, theta = input("Enter end state (x y theta) : ").split()
            end = Node((int(x), int(y), int(theta)), np.inf, None)

            if map.is_valid(start) and map.is_valid(end):
                print("\nStarting search...")
                break
            
            print("\nStart or end state is in obstacle. Try again.")
        except Exception:
            print("\nIncorrect input format. Try again.")
            continue

    solver = AStarSolver(start, end, map)
    start_time = time.time()
    path = solver.solve()
    end_time = time.time()

    print(f"\nFound path in {end_time - start_time} seconds.")
    print("Distance from state to goal : ", path[-1].cost_to_come)

    nodes = solver.get_explored_nodes()
    print(f"Final node in the path : {path[-1].state}")

    # fig, ax = plt.subplots(1,1)
    # for i in range(len(nodes)):
    #     if nodes[i].parent is None:
    #         continue
    #     x1 = np.array(nodes[i].parent.state[0])
    #     y1 = np.array(nodes[i].parent.state[1])
    #     x2 = np.array(nodes[i].state[0]) - x1 
    #     y2 = np.array(nodes[i].state[1]) - y1 
    #     ax.quiver(x1, y1, x2, y2,units='xy' ,scale=1, color='red')

    # plt.show()

    visualizer = Visualizer(map, path, nodes)
    visualizer.plot(step_size=1)

if __name__ == "__main__":
    main()