from abc import ABC, abstractmethod

class FunctionalTest(ABC):
    def __init__(self):
        """Initialize the final mask and waypoint."""
        self.final_mask = None
        self.waypoint = None

    @abstractmethod
    def run_frame(self, hsv_identifier="1", frame=None):
        """Run the functional test on a single frame. Should return final mask and waypoint."""
        pass


"""
Sample usage:

class PedestrianLaneChange(FunctionalTest):
    def __init__(self):
        super().__init__()
        # Initialize any additional attributes specific to this test
    
    def state_machine(self):
        # Implement the state machine logic for pedestrian lane change

    
    def update_mask(self):
        # Update the final mask based on the current state and frame


    def run_frame(self, hsv_identifier="1", frame=None):
        self.image = frame
        self.state_machine()
        self.update_mask()

        return self.final_mask, self.waypoint
"""