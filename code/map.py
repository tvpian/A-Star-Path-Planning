import numpy as np
from node import Node
from matplotlib import patches
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

class Map:
    """
    The map class.
    """

    def __init__(self, width : int = 600, height : int = 250, radius : int = 5, clearance : int = 5):
        """
        Initialize the map with the given width, height, and clearance.

        Parameters
        ----------
        width : int, optional
            The width of the map, by default 600
        height : int, optional
            The height of the map, by default 250
        clearance : int, optional
            The clearance of the map, by default 5
        """
        self.width = width
        self.height = height
        self.robot_radius = radius
        self.clearance = clearance + self.robot_radius

        self.map = np.zeros((width, height))
        self.workspace = mplPath.Path(np.array([(0, 0), [width-1, 0], [width-1, height-1], [0, height-1], [0, 0]]))
        
    def _is_obstacle(self, i, j):
        """
        Check if the given state is an obstacle.

        Parameters
        ----------
        i : int
            The x coordinate of the state.
        j : int
            The y coordinate of the state.

        Returns
        -------
        bool
            True if the state is an obstacle, False otherwise.
        """
        if (i >= (100 - self.clearance)) and (i <= (150 + self.clearance)) and (j >= 0) and (j <= (100 + self.clearance)):
            return True
        if (i >= (100 - self.clearance)) and (i <= (150 + self.clearance)) and j >= (150 - self.clearance) and (j <= self.height):
            return True             
        
        # Triangle
        if (i >= (460 - self.clearance)) and (j >= (25 - self.clearance)) and (j <= (225 + self.clearance)) and ((-2*i + j) >= -895 + self.clearance) and ((2*i + j) <= 1145 - self.clearance):
            return True             

        # Hexagon
        if (i >= (235 - self.clearance)) and (i <= (365 + self.clearance)) and ((i + 2*j) >= 400 - self.clearance) and ((i - 2*j) <= 210 - self.clearance) and ((i - 2*j) >= -110 + self.clearance) and ((i + 2*j) <= 700 + self.clearance):
            return True             

        # Walls
        if i < self.clearance or i >= (self.width - self.clearance) or j < self.clearance or j >= (self.height - self.clearance):
            return True

    def _is_in_bounds(self, x, y):
        """
        Check if the given state is in bounds.

        Parameters
        ----------
        x : int
            The x coordinate of the state.
        y : int
            The y coordinate of the state.

        Returns
        -------
        bool
            True if the state is in bounds, False otherwise.
        """
        return x >= 0 and x < self.width and y >= 0 and y < self.height
    
    def is_valid(self, node : Node):
        """
        Check if the given node is valid.

        Parameters
        ----------
        node : Node
            The node to check.

        Returns
        -------
        bool
            True if the node is valid, False otherwise.
        """
        x, y, theta = node.state
        return self._is_in_bounds(x, y) and not self._is_obstacle(x, y)

    def plot_map(self):
        """
        Plot the map.

        Returns
        -------
        fig, ax
            The figure and axis of the plot.
        """
        fig, ax = plt.subplots(subplot_kw={'aspect': 'auto'}, dpi=250)

        ws_verts = self.workspace.vertices
        plt.plot(ws_verts[:,0],ws_verts[:,1], color='y', linewidth=5)
        self.plot_obstacles(ax)

        return fig, ax
    
    def plot_obstacles(self, ax):
        """
        Plot the obstacles on the map.

        Parameters
        ----------
        ax : axis
            The axis to plot the obstacles on.
        """
        triangle = [[455, 230], [455, 20], [507.5, 125]]
        e = patches.Polygon(xy=triangle)
        ax.add_artist(e)
        e.set_facecolor('y')

        rect1 = [[95, 105], [155, 105], [155, 0], [95, 0]]
        e = patches.Polygon(xy=rect1)
        ax.add_artist(e)
        e.set_facecolor('y')

        rect2 = [[95, 145], [155, 145], [155, 250], [95, 250]]
        e = patches.Polygon(xy=rect2)
        ax.add_artist(e)
        e.set_facecolor('y')

        hex = [[300, 202.5], [370, 167.5], [370, 82.5], [300, 47.5], [230,82.5], [230, 167.5]]
        e = patches.Polygon(xy=hex)
        ax.add_artist(e)
        e.set_facecolor('y')