# Manages the current task the robot is performing

class StateMachine:
	stack = []

	def update(self):
		state = self.current()
		if (state is not None):
			if (state.condition()):
				state.do()
			else:
				state.next()
				self.pop()

	def pop(self):
		self.stack.pop()

	def push(self, state):
		if (self.current() != state):
			self.stack.insert(0, state)

	def clear(self):
		self.stack = []

	def current(self):
		if (len(self.stack) > 0):
			return self.stack[len(self.stack) - 1]
		else:
			return None