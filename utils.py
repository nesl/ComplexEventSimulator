import csv
import matplotlib.pyplot as plt
import numpy as np
import os
import sys


# Read in all our lines and get the data structure
def read_data(points_filepath="locations.txt"):

    points_data = []

    # Open the points file where we store all our waypoints/camera positions/etc
    with open(points_filepath, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in spamreader:
            row = [x.strip() for x in row]
            if not row:
                continue
            # We ignore rows with '#' comments
            comments = [1 if "#" in x else 0 for x in row]
            if comments[0] == 1:  # The whole row is a comment, and we ignore
                continue
            elif 1 in comments: # There is a comment at a later point
                comment_index = comments.index(1)
                last_obj = row[comment_index].split("#")[0]
                row = row[:comment_index]
                row.append(last_obj.strip())

            # Do some basic error checking
            assert len(row) == 7

            #  Appent our data points
            desc = row[-1]
            row = [float(x) for x in row[:-1]]
            points_data.append(row + [desc])

    return points_data



