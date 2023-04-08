import numpy as np
from map import Map

class ActionSet:
    """
    Class containing all the available node actions.
    """

    def __init__(self, RPM1 : float, RPM2 : float, map : Map):
        """
        Parameters
        ----------
        RPM1 : float
            The RPM of the first motor.
        RPM2 : float
            The RPM of the second motor.
        """
        self.r = 3.3
        self.l = 20
        self.map = map
        self.action_set = np.array([[0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2],
                                    [RPM2, 0], [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]])

    def get_actions(self, state):
        """
        Returns a dictionary of all the actions and their costs.

        Parameters
        ----------
        state : tuple
            The state of the node.
            
        Returns
        -------
        dict
            A dictionary of all the actions and their costs.
        """

        actions = {}
        for action in self.action_set:
            new_state, cost = self._get_action_cost(state, action)
            if new_state is not None:
                actions[tuple(action)] = (new_state, cost)
        return actions
    
    def _get_action_cost(self, state : np.array, action : np.array) -> float:
        """
        Returns the cost of the action.

        Parameters
        ----------
        state : np.array
            The state of the node.
        action : np.array
            The action to be taken.

        Returns
        -------
        tuple
            The new state and the cost of the action.
        """
        t = 0
        dt = 0.1
        D = 0
        x_init, y_init, theta_init = state[0], state[1], state[2]
        positions = []

        while t < 1:
            t = t + dt
            
            x_next = x_init + (0.5 * self.r * (action[0] + action[1]) * np.cos(np.deg2rad(theta_init))) * dt
            y_next = y_init + (0.5 * self.r * (action[0] + action[1]) * np.sin(np.deg2rad(theta_init))) * dt
            theta_next = theta_init + np.rad2deg((self.r / self.l) * (action[1] - action[0]) * dt)
            theta_next %= 360
            
            D += np.sqrt((0.5 * self.r * (action[0] + action[1]) * np.sin(np.deg2rad(theta_next)) * dt)**2 + (0.5 * self.r * (action[0] + action[1]) * np.cos(np.deg2rad(theta_next)) * dt)**2)

            if not self.map.is_valid(x_next, y_next):
                return None, None
                
            positions.append([x_next, y_next])
            x_init, y_init, theta_init = x_next, y_next, theta_next

        
        new_state = np.array([x_next, y_next, theta_next])
        return new_state, D

class Node:
    """
    A node in the search tree.
    """
    def __init__(self, state : tuple, cost_to_come : float, parent, parent_action : tuple = None):
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
        self.parent_action = parent_action

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
        return 2 * np.linalg.norm(self.state[:2] - goal_node.state[:2])
    
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
        for action, (new_state, cost) in actions.items():
             children.append(Node(new_state, cost, self, action))

        return children