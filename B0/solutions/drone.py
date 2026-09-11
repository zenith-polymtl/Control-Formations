#!/usr/bin/env python3
"""
Solution de l'exercice de B0 section 4 : une classe Drone avec une position.

À rouler depuis le dossier b0 : python src/drone.py
"""


class Drone:
    def __init__(self, name):
        self.name = name
        self.north = 0.0      # m, positif vers le nord
        self.east = 0.0       # m, positif vers l'est
        self.altitude = 0.0   # m au-dessus du point de départ

    def takeoff(self, altitude):
        self.altitude = altitude
        print(f"{self.name} décolle à {self.altitude} m")

    def move(self, north, east):
        self.north += north
        self.east += east

    def status(self):
        print(f"{self.name} : nord {self.north} m, est {self.east} m, altitude {self.altitude} m")


if __name__ == "__main__":
    drone = Drone("Hexa")
    drone.takeoff(10)
    drone.move(40, 0)
    drone.status()        # nord 40.0 m, est 0.0 m, altitude 10 m
    drone.move(0, 40)
    drone.status()        # nord 40.0 m, est 40.0 m, altitude 10 m
