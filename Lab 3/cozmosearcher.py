from transitions import Machine
from cozmo.util import degrees, distance_mm, speed_mmps


class CozmoSearcher(object):
    states = ['init', 'search', 'pursue', 'touch', 'done']

    def __init__(self, robot):
        self.robot = robot
        self.lastSeen = None

        self.machine = Machine(model=self, states=CozmoSearcher.states, initial='init')

        self.machine.add_transition(trigger='initialize', source='init', dest='search')

        self.machine.add_transition(trigger='ball_found', source='search', dest='pursue')
        self.machine.add_transition(trigger='at_ball', source='pursue', dest='touch')

        self.machine.add_transition(trigger='ball_lost', source='pursue', dest='search')
        self.machine.add_transition(trigger='ball_lost', source='touch', dest='search')

        self.machine.add_transition(trigger='finish', source='touch', dest='done')

        self.machine.on_exit_search('print_state_out')
        self.machine.on_exit_pursue('print_state_out')
        self.machine.on_exit_touch('print_state_out')
        self.machine.on_exit_init('print_state_out')
        self.machine.on_exit_done('print_state_out')

        self.machine.on_enter_init('print_state')
        self.machine.on_enter_done('print_state')
        self.machine.on_enter_search('print_state')
        self.machine.on_enter_pursue('print_state')
        self.machine.on_enter_touch('print_state')

        self.machine.on_exit_init('exit_init')

        self.initialize()

    def print_state_out(self):
        print("-> Leaving state " + str(self.state))

    def print_state(self):
        print("-> Entering state " + str(self.state))
        self.robot.say_text(self.state, use_cozmo_voice=False, duration_scalar=0.25, voice_pitch=1).wait_for_completed()

    def exit_init(self):
        print("Initializing at " + str(self.state))
        self.robot.set_head_angle(degrees(-10)).wait_for_completed()
        self.robot.set_lift_height(0).wait_for_completed()

    def save_ball_location(self, ball_found):
        self.lastSeen = ball_found
