from src.nodes.nodes.conditional import Conditional


class SkeletonConditional(Conditional):

    def __init__(self):
        super().__init__()
        ...

    def condition(self, blackboard: dict) ->bool:

        if (...) :
            return True
        return False
