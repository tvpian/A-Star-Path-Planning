import numpy as np

class ActionSet:
    """
    Class containing all the available node actions.
    """

    def __init__(self, step_size=5, angles=[-60, -30, 0, 30, 60]):
        """
        """
        self.angles = angles
        self.step_size = step_size

    def get_actions(self, state):
        """
        Get the actions available for the given state.

        Parameters
        ----------
        state : tuple
            The state for which to get the actions.

        Returns
        -------
        dict
            A dictionary of actions, where the key is the action and the value is the cost of the action.
        """
        actions = {}
        for angle in self.angles:
            new_angle = (state[2] + angle) % 360
            x_diff = self.step_size * np.cos(np.deg2rad(new_angle))
            y_diff = self.step_size * np.sin(np.deg2rad(new_angle))
            actions[(x_diff, y_diff, angle)] = self.step_size
        return actions
    
class Node:
    """
    A node in the search tree.
    """
    def __init__(self, state : tuple, cost_to_come : float, parent):
        """
        Parameters
        ----------
        state : tuple
            The state of the node.
        cost_to_come : float
            The cost to come to this node.
        parent : Node
            The parent node.
        """        
        self.state = np.array(state)    
        self.cost_to_come = cost_to_come
        self.rounded_state = self._round(self.state)

        self.parent = parent

        self.action_set = ActionSet()

    def __hash__(self) -> int:
        return hash(tuple(self.rounded_state))
       
    def __eq__(self, other) -> bool:
        return self.rounded_state == other.rounded_state
    
    def __lt__(self, other) -> bool:
        return self.cost_to_come < other.cost_to_come
    
    def __str__(self) -> str:
        return str(self.state)
    
    @classmethod
    def set_actionset(cls, actionset):
        cls.action_set = actionset

    @classmethod
    def set_resolution(cls, resolution):
        cls.resolution = resolution

    def _round(self, state : np.array) -> np.array:
        """
        Round the state to the nearest integer.

        Parameters
        ----------
        state : np.array
            The state to round.

        Returns
        -------
        np.array
            The rounded state.
        """
        result = [] 
        for val, res in zip(state, self.resolution):
            result.append(round(val/res)*res)
        return result
    
    def cost_to_go(self, goal_node):
        """
        Compute the Euclidean distance between the current node and goal node.

        Parameters
        ----------
        goal_node : np.array
            The goal node to compare the distance to

        Returns
        -------
        float
            The Euclidean distance between the current state and the goal state
        """
        return np.linalg.norm(self.state[:2] - goal_node.state[:2])*0.8 + np.abs(self.state[2] - goal_node.state[2])*0.2
    
    def get_children(self) -> list:
        """
        Get the list of children nodes.
        
        Returns
        -------
        list
            The list of children nodes.
        """
        children = []
        actions = self.action_set.get_actions(self.state)
        for action, cost in actions.items():
            new_state = self.state + np.array(action)
            child = Node(new_state, cost, self)
            children.append(child)

        return children