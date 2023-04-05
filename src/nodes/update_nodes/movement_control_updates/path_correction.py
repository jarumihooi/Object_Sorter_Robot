from ...nodes.update import Update
import rospy
import math




class PathCorrection(Update):

    def __init__(
            self,
            correction_var_name: str,
            goal_value_var_name: str,
            current_value_var_name: str,
            max_vel: float,
            steepness: float = 5,
            offset: float = 0
        ):
        super().__init__()


        self.correction = correction_var_name
        self.goal = goal_value_var_name
        self.curr = current_value_var_name

        self.correct = lambda x: max_vel*((2/(1+math.e**(-1*steepness*(x-offset)))) - 1)

    def update_blackboard(self, blackboard: dict):
        try:
            cur_val = blackboard[self.curr]
            goal_val = blackboard[self.goal]
            correct_val = self.correct(goal_val-cur_val)
            blackboard[self.correction] = correct_val

            return "success"
        except:
            return "failure"