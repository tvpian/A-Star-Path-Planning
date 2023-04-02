import numpy as np

class ActionSet:
    """
    Class containing all the available node actions.
    """

    def __init__(self, RPM1 : float, RPM2 : float):
        self.r = 0.038
        self.l = 0.354
        self.action_set = np.array([[0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2],
         [RPM2, 0], [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]])

    def get_actions(self, state):

        actions = {}
        for action in self.action_set:
            actions[str(action)] = self._get_action_cost(state, action)
        return actions
    
    def _get_action_cost(self, state : np.array, action : np.array) -> float:
        t = 0
        dt = 0.1
        D = 0
        x_init, y_init, theta_init = state[0], state[1], state[2]
        positions = []

        while t < 1:
            t = t + dt
            
            x_next = x_init + 0.5 * self.r * (action[0] + action[1]) * np.cos(np.deg2rad(theta_init)) * dt
            y_next = y_init + 0.5 * self.r * (action[0] + action[1]) * np.sin(np.deg2rad(theta_init)) * dt
            theta_next = theta_init + (self.r / self.l) * (action[1] - action[0]) * dt
            theta_next %= 360
            
            D += np.sqrt((0.5 * self.r * (action[0] + action[1]) * np.sin(np.deg2rad(theta_init)) * dt)**2 + (0.5 * self.r * (action[0] + action[1]) * np.cos(np.deg2rad(theta_init)) * dt)**2)

            positions.append([x_next, y_next])
            x_init, y_init, theta_init = x_next, y_next, theta_next
        
        new_state = np.array([x_next, y_next, theta_next])
        return new_state, D

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
        self.cost = 0
        self.rounded_state = self._round(self.state)

        self.parent = parent

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
        return np.linalg.norm(self.rounded_state[:2] - goal_node.state[:2])
    
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
        for new_state, cost in actions.values():
            children.append(Node(new_state, self.cost_to_come + cost, self))

        return children