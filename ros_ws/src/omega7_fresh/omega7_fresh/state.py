"""Small explicit arm enable state machine."""


class MotionGate:
    def __init__(self):
        self.enabled = False
        self.generation = 0

    def enable(self, ready):
        if self.enabled:
            return False, "already enabled"
        if not ready:
            return False, "inputs not ready; arm remains disabled"
        self.enabled = True
        self.generation += 1
        return True, "arm enabled"

    def disable(self):
        was_enabled = self.enabled
        self.enabled = False
        self.generation += 1
        return was_enabled
