#!/usr/bin/env python3

import ntcore
import time
from datetime import datetime

class NetworkTable():
    def __init__(self) -> None:
        # Get the default instance of NetworkTables that was created automatically
        # when the robot program starts
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4("wpilibpi")
        inst.setServer('10.54.6.2', 5810) #server_name (str), port
        #inst.startDSClient() # Starts requesting server address from driver station This connects to the Driver Station running on localhost to obtain the server IP address.

        # Get the table within that instance that contains the data. There can
        # be as many tables as you like and exist to make it easier to organize
        # your data. In this case, it's a table called datatable.
        table = inst.getTable("datatable")

        # Start publishing topics within that table that correspond to the X and Y values
        # for some operation in your program.
        # The topic names are actually "/datatable/x" and "/datatable/y".
        self.xPub = table.getStringTopic("time").publish()
        #self.yPub = table.getDoubleTopic("y").publish()

        #self.x = str(datetime.now())
        #self.x = jsonData
        #self.y = 0

        while True:
            self.x = str(datetime.now())
            self.xPub.set(self.x)
        #self.yPub.set(self.y)


    def teleopPeriodic(self) -> None:
        # Publish values that are constantly increasing.
        while True:
            self.xPub.set(self.x)
            time.sleep(1)
        #self.yPub.set(self.y)
        #self.x += 0.05
        #self.y += 1.0


if __name__ == "__main__":
    NetworkTable()