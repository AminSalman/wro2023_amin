from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Button


class Configurator:
    """
    This class is responsible for capturing the random blocks' configuration before running the main loop.
    """

    def __init__(self, EV3: EV3Brick()):
        self.hub = EV3

    def wait_until_no_buttons_pressed(self):
        """
        Waits until all the buttons on the hub are released before continuing with the rest of the program.
        """
        while True:
            if not self.hub.buttons.pressed():
                return

    def capture_depot(self):
        """
        Starts capturing the depot block colors from LTR. UP for Green, DOWN for Blue, CENTER for reset
        :returns: The order of the colors as a string, such as "BGGB"
        """
        random_config = ""
        x = []
        while not ((x.count("B") == 2) and (x.count("G") == 2)):
            random_config = ""
            x = []
            self.hub.screen.clear()
            self.hub.screen.draw_text(65, 60, "Depot Colors", Color.BLACK, None)

            while len(x) < 4:
                if self.hub.buttons.pressed() == [Button.UP]:
                    x.append("G")
                    self.wait_until_no_buttons_pressed()
                elif self.hub.buttons.pressed() == [Button.DOWN]:
                    x.append("B")
                    self.wait_until_no_buttons_pressed()
                elif self.hub.buttons.pressed() == [Button.CENTER]:
                    x = []
                self.hub.screen.clear()
                self.hub.screen.draw_text(65, 60, str(random_config), Color.BLACK, None)
        random_config = random_config.join(x)
        return random_config

    def capture_markers(self):
        """
        Captures the marking blocks state. Left for both green, Right for both blue, CENTER for different colors.
        :return: 0 for both green, 1 for different colors, 2 for both blue
        """
        self.hub.screen.clear()
        self.hub.screen.draw_text(65, 60, "Markers", Color.BLACK, None)
        while True:
            if self.hub.buttons.pressed() == [Button.LEFT]:
                self.hub.screen.clear()
                self.hub.screen.draw_text(65, 60, "Both Green", Color.BLACK, None)
                return 0
            elif self.hub.buttons.pressed() == [Button.CENTER]:
                self.hub.screen.clear()
                self.hub.screen.draw_text(65, 60, "Different", Color.BLACK, None)
                return 1
            elif self.hub.buttons.pressed() == [Button.RIGHT]:
                self.hub.screen.clear()
                self.hub.screen.draw_text(65, 60, "Both Blue", Color.BLACK, None)
                return 2

    def print_configuration(self, m, d):
        """
        Prints the results of the capture on the EV3 screen for verification.
        """
        self.hub.screen.clear()
        self.hub.screen.draw_text(65, 40, d, Color.BLACK, None)
        self.hub.screen.draw_text(65, 60, m, Color.BLACK, None)
        return


# Capture and display configuration:
# marker_colors = Configurator(EV3Brick()).capture_markers()
# depot_colors = Configurator(EV3Brick()).capture_depot()
# Configurator(EV3Brick()).print_configuration(marker_colors, depot_colors)