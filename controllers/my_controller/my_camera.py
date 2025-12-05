from controller import Camera
import numpy as np
import csv

class MyCamera:
    def __init__(self, robot, keyboard):
        self.robot = robot
        self.keyboard = keyboard
        self.camera = self.robot.getDevice("camera")
        self.timestep = int(self.robot.getBasicTimeStep())

    def Init_All_Module(self):
        self.camera.enable(self.timestep)

    def detect_traffic_light(self, image, width, height, debug = False):
        if image is None:
            return "UNKNOWN", (None, None)

        red_score = 0.0
        yellow_score = 0.0
        green_score = 0.0

        red_pixels = []
        yellow_pixels = []
        green_pixels = []

        for y in range(0, height, 2):
            for x in range(0, width, 2):
                r = Camera.imageGetRed(image, width, x, y)
                g = Camera.imageGetGreen(image, width, x, y)
                b = Camera.imageGetBlue(image, width, x, y)

                if r < 30 and g < 30 and b < 30:
                    continue

                red_strength = max(0, r - max(g, b))
                green_strength = max(0, g - max(r, b))
                yellow_strength = 0
                if r > 60 and g > 60 and b < 80:
                    yellow_strength = min(r, g) - b - abs(r - g) * 0.5

                if red_strength > 20:
                    red_score += red_strength
                    red_pixels.append((x, y))
                if green_strength > 20:
                    green_score += green_strength
                    green_pixels.append((x, y))
                if yellow_strength > 20:
                    yellow_score += yellow_strength
                    yellow_pixels.append((x, y))

        # if debug:
        #     print(f"[TL Debug] red_score={red_score}, yellow_score={yellow_score}, green_score={green_score}")

        max_score = max(red_score, yellow_score, green_score)
        if max_score < 200:
            return "UNKNOWN", (None, None)

        if red_score > 1.5 * yellow_score and red_score > 1.5 * green_score:
            if red_score > 300 and red_score < 1200:
                color = "RED"
                pts = red_pixels
            else:
                return "UNKNOWN", (None, None)
        elif yellow_score >= red_score and yellow_score >= green_score:
            color = "YELLOW"
            pts = yellow_pixels
        else:
            color = "GREEN"
            pts = green_pixels

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        cx = int(sum(xs) / len(xs))
        cy = int(sum(ys) / len(ys))

        return color, (cx, cy)

    def Run_Traffic_Light_Camera(self):
        if self.camera is not None:
            width = self.camera.getWidth()
            height = self.camera.getHeight()
            image = self.camera.getImage()

            light_color, (cx, cy) = self.detect_traffic_light(image, width, height, debug=True)
            # print(f"Detected: {light_color}, center: {cx, cy}")
            return light_color


