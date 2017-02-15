from transitions import Machine

class CozmoSearcher(object):
	states = ['search', 'pursue', 'touch', 'search.left', 'search.right']

	def __init__(self):
		self.lastSeen = None

		self.machine = Machine(model=self, states=CozmoSearcher.states, initial='search')
		self.machine.add_transition(trigger='searchLeft', source='search', dest='search.left')
		self.machine.add_transition(trigger='searchRight', source='search', dest='search.right')
		self.machine.add_transition(trigger='ballFound', source='search.left', dest='pursue')
		self.machine.add_transition(trigger='ballFound', source='search.right', dest='pursue')
		self.machine.add_transition(trigger='atBall', source='pursue', dest='dest')
		self.machine.add_transition(trigger='ballLost', source='pursue', dest='search')
		self.machine.add_transition(trigger='ballLost', source='touch', dest='search')