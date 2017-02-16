from transitions import Machine
from cozmo.util import degrees, distance_mm, speed_mmps


class CozmoSearcher(object):
    states = ['init', 'search', 'pursue', 'touch']

    def __init__(self, robot):
        self.robot = robot
        self.lastSeen = None

        self.machine = Machine(model=self, states=CozmoSearcher.states, initial='init')

        self.machine.add_transition(trigger='initialize', source='init', dest='search')

        self.machine.add_transition(trigger='ball_found', source='search', dest='pursue')
        self.machine.add_transition(trigger='at_ball', source='pursue', dest='dest')

        self.machine.add_transition(trigger='ball_lost', source='pursue', dest='search')
        self.machine.add_transition(trigger='ball_lost', source='touch', dest='search')

        self.machine.on_enter_search('print_state')
        self.machine.on_enter_pursue('print_state')
        self.machine.on_enter_touch('print_state')

        self.machine.on_enter_search('enter_search')

        self.initialize()

    def print_state(self):
        print("-> Entering state " + str(self.state))

    def enter_search(self):
        print("Initializing at " + str(self.state))
        self.robot.set_head_angle(degrees(-10)).wait_for_completed()
        self.robot.set_lift_height(0).wait_for_completed()

    def save_ball_location(self, ball_found):
        self.lastSeen = ball_found
