import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from map import Map
import numpy as np
import cv2

class Visualizer:
    """
    The visualizer class.
    """

    def __init__(self, map : Map, path : list, nodes: list) -> None:
        """
        Initialize the visualizer with the map, path, and nodes.
        Parameters
        ----------
        map : Map
            The map.
        path : list 
            The path from the start node to the goal node.
        nodes : list
            The list of nodes.
        """
        self.map = map
        self.path = path
        self.nodes = nodes
        
        self.fig,self.ax = self.map.plot_map()
        self.ln, = plt.plot([],[],' .b')
        self.x_vals, self.y_vals = [],[]
        self.stop_running=False

    def plot(self, step_size : int = 500) -> None:
        """
        Plot the path and nodes.
        Parameters
        ----------
        step_size : int
            The number of nodes to plot per frame.
        """
        self.step_size = step_size
        # self.anim = FuncAnimation(
        #         self.fig, 
        #         self._update,
        #         interval=5,
        #         init_func=self.init,
        #         frames=range(len(self.nodes)//self.step_size),
        #         blit=True)

        self.init()
        for i in range(len(self.nodes)):
            if self.nodes[i].parent is None:
                continue
            x1 = np.array((self.nodes[i].parent.state[0]))
            y1 = np.array((self.nodes[i].parent.state[1]))
            x2 = np.array((self.nodes[i].state[0])) - x1 
            y2 = np.array((self.nodes[i].state[1])) - y1 
            self.ax.quiver(x1, y1, x2, y2, units='xy' ,scale=np.sqrt(x2**2 + y2**2), color='b')
            plt.pause(0.0001)

        X = [node.state[0] for node in self.path]
        Y = [node.state[1] for node in self.path]
        plt.plot(X, Y, '-r')

        plt.show()
        # self.save_animation("../results/matplotlib.mp4")

    def save_animation(self, filename) -> None:
        """
        Save the animation.
        Parameters
        ----------
        filename : str
            The name of the file to save the animation to.
        """
        writervideo = FFMpegWriter(fps=60) 
        self.anim.save(filename, writer=writervideo)

    def init(self) -> None:
        """
        Initialize the animation - plot the start and goal nodes.
        """
        plt.plot(*self.path[0].state[:2],'*r')
        plt.text(*self.path[0].state[:2],"START")
        plt.plot(*self.path[-1].state[:2],'*g')
        plt.text(*self.path[-1].state[:2],"GOAL")

        return self.ln,

    def _update(self, frame : int) -> None:
        """
        Update the animation - runs every frame and plots the nodes.
        Parameters
        ----------
        frame : int
            The current frame number.
        """
        if self.stop_running:
            return self.ln,

        if frame==0:
            self.x_vals = []
            self.y_vals = []
        
        for i in range(self.step_size):
            if self.nodes[(frame * self.step_size) + i].parent is None:
                continue
            # x1 = np.array((self.nodes[(frame * self.step_size) + i].parent.state[0]))
            # y1 = np.array((self.nodes[(frame * self.step_size) + i].parent.state[1]))
            # x2 = np.array((self.nodes[(frame * self.step_size) + i].state[0])) - x1 
            # y2 = np.array((self.nodes[(frame * self.step_size) + i].state[1])) - y1 
            # self.ax.quiver(x1, y1, x2, y2,units='xy' ,scale=1)
            self.x_vals.append(self.nodes[(frame*self.step_size) + i].state[0])
            self.y_vals.append(self.nodes[(frame*self.step_size) + i].state[1])

        self.ln.set_data(self.x_vals, self.y_vals)

        if frame == (len(self.nodes))//self.step_size-1:
            self.ln.set_data([],[])

            X = [n.state[0] for n in self.path]
            Y = [n.state[1] for n in self.path]
            self.ln, = plt.plot(X, Y, color='green', linewidth=2)

            self.stop_running=True
        
        return self.ln,

    def record_opencv(self, map, step_size : int = 500, record : bool = False) -> None:
        """
        Record the animation using OpenCV.
        Parameters
        ----------
        step_size : int
            The number of nodes to plot per frame.        
        """

        if record:
            print("Recording video....Please wait until said done")
            result = cv2.VideoWriter('../results/animation.avi', cv2.VideoWriter_fourcc(*'MJPG'), 120, (map.width, map.height)) 

        frame = np.zeros((map.height, map.width, 3), dtype = np.uint8)

        y, x = np.where(map.map == 1)                                                                                                  
        for i in range(len(x)):
            frame[x[i], y[i]] = [0,0,255]
                
        for i in range(0,len(self.nodes), step_size):
            if self.nodes[i].parent is None:
                continue
            x1 = self.nodes[i].parent.rounded_state[0]
            y1 = map.height - self.nodes[i].parent.rounded_state[1] 
            x2 = self.nodes[i].rounded_state[0]  
            y2 = map.height - self.nodes[i].rounded_state[1] 
            cv2.arrowedLine(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255,0,0), 1, 1)
        #     if record:
        #         result.write(frame) 
        
        cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Frame", 1280, 720)
        cv2.imshow("Frame", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        for i in range(0, len(self.path)-1,step_size):
            if self.nodes[i].parent is None:
                    continue
            x1 = self.path[i].state[0]
            y1 = map.height - self.path[i].state[1]
            x2 = self.path[i+1].state[0]
            y2 = map.height - self.path[i+1].state[1]
            cv2.arrowedLine(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255,255,255), 1, 1)
            if record:
                result.write(frame) 
        
        if record:
            result.release()
            print("Path planning and video recording complete")

        cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Frame", 1280, 720)
        cv2.imshow("Frame", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()