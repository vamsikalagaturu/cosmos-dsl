import os
import sys

import random
from matplotlib.patches import Ellipse
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from units import unit


class Plotter:

    def __init__(self) -> None:
        # set the output path to be the root of the project
        self.root_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))), 'outputs')

        self.out_path = os.path.join(self.root_path, 'plots')

        # create the output directory if it does not exist
        os.makedirs(self.out_path, exist_ok=True)

        self.meter = unit('m')
        self.second = unit('s')

        # all plots are in meters

    def plot_point(self, label, point:np.ndarray, d2: bool = False):
        # initialize the plot 2D or 3D with size 10x10
        if d2:
            plt.figure(figsize=(10, 10))
            ax = plt.axes()
        else:
            plt.figure(figsize=(10, 10))
            # set the 3D projection
            ax = plt.axes(projection='3d')
        
        if d2:
            x = random.uniform(0, 5)
            y = random.uniform(0, 5)

            point = np.array([x, y])

        # plot the point 2D or 3D
        if d2:
            ax.scatter(point[0], point[1], label=label)
        else:
            ax.scatter(point[0], point[1], point[2], label=label, marker='o', s=100)
            ax.text(point[0]+0.002, point[1]+0.002, point[2]+0.002, label, size=15, zorder=2, color='k')

        # set the title of the plot
        plt.title('Point Plot')
        # set the x-axis label
        ax.set_xlabel('X')
        # set the y-axis label
        ax.set_ylabel('Y')
        # add grid
        plt.grid()
        # fit the plot to the window
        plt.tight_layout()

        # if 3D plot the z-axis label
        if not d2:
            ax.set_zlabel('Z')

        # set the legend
        plt.legend()

        plt.show()

    def plot_frame(self):

        # Define the origin and orientation of the frame
        origin = np.array([0, 0])  # Change this to your desired origin

        theta = 0

        # calculate the x-axis and y-axis
        x_axis = np.array([np.cos(theta), np.sin(theta)])
        y_axis = np.array([-np.sin(theta), np.cos(theta)])

        # Define arrow lengths
        arrow_length = 10

        # Calculate arrow endpoints
        x_arrow_end = origin + arrow_length * x_axis
        y_arrow_end = origin + arrow_length * y_axis

        # Create the plot
        plt.figure(figsize=(8, 6))

        # Plot the arrows
        plt.arrow(origin[0], origin[1], x_arrow_end[0] - origin[0], x_arrow_end[1] - origin[1],
                head_width=0.5, head_length=0.7, fc='r', ec='r', label='X-Axis')
        plt.arrow(origin[0], origin[1], y_arrow_end[0] - origin[0], y_arrow_end[1] - origin[1],
                head_width=0.5, head_length=0.7, fc='g', ec='g', label='Y-Axis')

        # Plot the origin point
        plt.plot(origin[0], origin[1], 'go', label='Origin')

        # Set plot limits
        plt.xlim(-1, origin[0] + arrow_length + 2)
        plt.ylim(-1, origin[1] + arrow_length + 2)

        # Add labels and legend
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('XY Frame')
        plt.legend()

        # Show the plot
        plt.grid()
        plt.show()

    def plot_frame_point(self):

        # Define the origin and orientation of the frame
        origin = np.array([0, 0])  # Change this to your desired origin

        theta = 0

        # calculate the x-axis and y-axis
        x_axis = np.array([np.cos(theta), np.sin(theta)])
        y_axis = np.array([-np.sin(theta), np.cos(theta)])

        # Define arrow lengths
        arrow_length = 3

        # Calculate arrow endpoints
        x_arrow_end = origin + arrow_length * x_axis
        y_arrow_end = origin + arrow_length * y_axis

        # calculate the head width and length based on the arrow length
        head_width = arrow_length / 15
        head_length = arrow_length / 10

        # Create the plot
        plt.figure(figsize=(8, 6))

        # Plot the arrows
        plt.arrow(origin[0], origin[1], x_arrow_end[0] - origin[0], x_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='r', ec='r', label='X-Axis')
        plt.arrow(origin[0], origin[1], y_arrow_end[0] - origin[0], y_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='g', ec='g', label='Y-Axis')

        # Plot the origin point
        plt.plot(origin[0], origin[1], 'go', label='Origin')
        # annotate the origin
        plt.annotate('O', (0, 0), (0 - 0.2, 0 - 0.2), size=15, zorder=2, color='k')

        # plot the point
        plt.plot(1.5, 1.5, 'bo', label='Point')
        # annotate the point
        plt.annotate('P', (1.5, 1.5), (1.5 + 0.1, 1.5 + 0.1), size=15, zorder=2, color='k')

        # draw dashed lines from the point to the x-axis and y-axis
        plt.plot([1.5, 1.5], [0, 1.5], 'g--')
        plt.plot([0, 1.5], [1.5, 1.5], 'r--')

        # Set plot limits
        plt.xlim(-1, origin[0] + arrow_length + 1)
        plt.ylim(-1, origin[1] + arrow_length + 1)

        # Add labels and legend
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('XY Frame')
        plt.legend()

        # Show the plot
        plt.grid()
        plt.show()

    def plot_vector(self):
        # Define the origin and orientation of the frame
        origin = np.array([0, 0])  # Change this to your desired origin
        
        theta = 0

        # calculate the x-axis and y-axis
        x_axis = np.array([np.cos(theta), np.sin(theta)])
        y_axis = np.array([-np.sin(theta), np.cos(theta)])

        # Define arrow lengths
        arrow_length = 3

        # Calculate arrow endpoints
        x_arrow_end = origin + arrow_length * x_axis
        y_arrow_end = origin + arrow_length * y_axis

        # calculate the head width and length based on the arrow length
        head_width = arrow_length / 15
        head_length = arrow_length / 10

        # Create the plot
        plt.figure(figsize=(8, 6))

        # Plot the arrows
        plt.arrow(origin[0], origin[1], x_arrow_end[0] - origin[0], x_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='r', ec='r', label='X-Axis')
        plt.arrow(origin[0], origin[1], y_arrow_end[0] - origin[0], y_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='g', ec='g', label='Y-Axis')

        # Plot the origin point
        plt.plot(origin[0], origin[1], 'go', label='Origin')
        # annotate the origin
        plt.annotate('O', (0, 0), (0 - 0.2, 0 - 0.2), size=15, zorder=2, color='k')

        # plot the point
        plt.plot(1.5, 1.5, 'bo', label='Point')
        # annotate the point
        plt.annotate('P', (1.5, 1.5), (1.5 + 0.1, 1.5 + 0.1), size=15, zorder=2, color='k')

        # Define the vector
        vector_point = np.array([1.5, 1.5])

        # Plot the vector using annotate
        vector_props = dict(facecolor='black', shrink=0.0, width=1, headwidth=10)
        plt.annotate('', xy=vector_point, xytext=origin, arrowprops=vector_props)

        # Set plot limits
        plt.xlim(-1, origin[0] + arrow_length + 1)
        plt.ylim(-1, origin[1] + arrow_length + 1)

        # Add labels and legend
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('XY Frame with Vector')
        plt.legend()

        # Show the plot
        plt.grid()
        plt.show()

    def plot_rigid_body(self):
        # Define the origin and orientation of the frame
        origin = np.array([0, 0])  # Change this to your desired origin
        
        theta = 0

        # calculate the x-axis and y-axis
        x_axis = np.array([np.cos(theta), np.sin(theta)])
        y_axis = np.array([-np.sin(theta), np.cos(theta)])

        # Define arrow lengths
        arrow_length = 3

        # Calculate arrow endpoints
        x_arrow_end = origin + arrow_length * x_axis
        y_arrow_end = origin + arrow_length * y_axis

        # calculate the head width and length based on the arrow length
        head_width = arrow_length / 15
        head_length = arrow_length / 10

        # Create the plot
        plt.figure(figsize=(8, 6))

        # Plot the arrows
        plt.arrow(origin[0], origin[1], x_arrow_end[0] - origin[0], x_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='r', ec='r', label='X-Axis')
        plt.arrow(origin[0], origin[1], y_arrow_end[0] - origin[0], y_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='g', ec='g', label='Y-Axis')

        # Plot the origin point
        plt.plot(origin[0], origin[1], 'go', label='Origin')
        # annotate the origin
        plt.annotate('O', (0, 0), (0 - 0.2, 0 - 0.2), size=15, zorder=2, color='k')

        # Define link lengths and joint angles
        link_lengths = [1, 1, 1]  # Lengths of the three links
        joint_angles = [np.pi / 4, np.pi / 3, np.pi / 6]  # Joint angles for each link

        # Initialize starting point
        current_x, current_y = 0.5, 0.5

        # Loop through each link
        for length, angle in zip(link_lengths, joint_angles):
            # Calculate the new end point of the link
            new_x = current_x + length * np.cos(angle)
            new_y = current_y + length * np.sin(angle)
            
            # Calculate the mid-point of the link for ellipse center
            mid_x = (current_x + new_x) / 2
            mid_y = (current_y + new_y) / 2
            
            # Calculate ellipse width and height based on link length
            ellipse_width = length * 1
            ellipse_height = 0.3
            
            # Calculate the angle of rotation for the ellipse
            ellipse_angle = np.degrees(angle)
            
            # Create and add the ellipse patch to the plot
            ellipse = Ellipse((mid_x, mid_y), width=ellipse_width, height=ellipse_height, angle=ellipse_angle,
                            edgecolor='black', facecolor='none')
            
            plt.gca().add_patch(ellipse)

            # plot the center of the ellipse
            plt.plot(mid_x, mid_y, 'ro')
            
            # Update the current position
            current_x, current_y = new_x, new_y


        # Set plot limits
        plt.xlim(-1, 4)
        plt.ylim(-1, 4)

        # Add labels and legend
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('XY Frame with Vector')
        plt.legend()

        # Show the plot
        plt.grid()
        plt.show()

    def plot_point_to_point_distance(self):
        # Define the origin and orientation of the frame
        origin = np.array([0, 0])  # Change this to your desired origin
        
        theta = 0

        # calculate the x-axis and y-axis
        x_axis = np.array([np.cos(theta), np.sin(theta)])
        y_axis = np.array([-np.sin(theta), np.cos(theta)])

        # Define arrow lengths
        arrow_length = 3

        # Calculate arrow endpoints
        x_arrow_end = origin + arrow_length * x_axis
        y_arrow_end = origin + arrow_length * y_axis

        # calculate the head width and length based on the arrow length
        head_width = arrow_length / 15
        head_length = arrow_length / 10

        # Create the plot
        plt.figure(figsize=(8, 6))

        # Plot the arrows
        plt.arrow(origin[0], origin[1], x_arrow_end[0] - origin[0], x_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='r', ec='r', label='X-Axis')
        plt.arrow(origin[0], origin[1], y_arrow_end[0] - origin[0], y_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='g', ec='g', label='Y-Axis')

        # Plot the origin point
        plt.plot(origin[0], origin[1], 'go', label='Origin')
        # annotate the origin
        plt.annotate('O', (0, 0), (0 - 0.2, 0 - 0.2), size=15, zorder=2, color='k')

        # plot the point
        plt.plot(1, 1, 'bo', label='Point')
        # annotate the point
        plt.annotate('A', (1, 1), (1 - 0.2, 1 - 0.2), size=15, zorder=2, color='k')

        # plot the point
        plt.plot(2, 2, 'bo', label='Point')
        # annotate the point
        plt.annotate('B', (2, 2), (2 + 0.1, 2 + 0.1), size=15, zorder=2, color='k')

        # plot line between the points
        plt.plot([1, 2], [1, 2], 'r--')

        # Set plot limits
        plt.xlim(-1, origin[0] + arrow_length + 1)
        plt.ylim(-1, origin[1] + arrow_length + 1)

        # Add labels and legend
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('XY Frame with Vector')
        plt.legend()

        # Show the plot
        plt.grid()
        plt.show()

    def plot_point_to_line_distance(self):
        # Define the origin and orientation of the frame
        origin = np.array([0, 0])  # Change this to your desired origin
        
        theta = 0

        # calculate the x-axis and y-axis
        x_axis = np.array([np.cos(theta), np.sin(theta)])
        y_axis = np.array([-np.sin(theta), np.cos(theta)])

        # Define arrow lengths
        arrow_length = 3

        # Calculate arrow endpoints
        x_arrow_end = origin + arrow_length * x_axis
        y_arrow_end = origin + arrow_length * y_axis

        # calculate the head width and length based on the arrow length
        head_width = arrow_length / 15
        head_length = arrow_length / 10

        # Create the plot
        plt.figure(figsize=(8, 6))

        # Plot the arrows
        plt.arrow(origin[0], origin[1], x_arrow_end[0] - origin[0], x_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='r', ec='r', label='X-Axis')
        plt.arrow(origin[0], origin[1], y_arrow_end[0] - origin[0], y_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='g', ec='g', label='Y-Axis')

        # Plot the origin point
        plt.plot(origin[0], origin[1], 'go', label='Origin')
        # annotate the origin
        plt.annotate('O', (0, 0), (0 - 0.2, 0 - 0.2), size=15, zorder=2, color='k')

        # plot the line
        plt.plot([1, 2], [1, 2], 'r')

        # plot line start point and end point
        plt.plot(1, 1, 'ro', label='Point')
        plt.plot(2, 2, 'ro', label='Point')

        # plot the point
        plt.plot(2, 1, 'bo', label='Point')
        # annotate the point
        plt.annotate('P', (2, 1), (2 + 0.1, 1 + 0.1), size=15, zorder=2, color='k')

        # plot the perpendicular line from the point to the center of the line
        plt.plot([2, 1.5], [1, 1.5], 'g--')

        # Set plot limits
        plt.xlim(-1, origin[0] + arrow_length + 1)
        plt.ylim(-1, origin[1] + arrow_length + 1)

        # Add labels and legend
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('XY Frame with Vector')
        plt.legend()

        # Show the plot
        plt.grid()
        plt.show()

    def plot_line_to_line_distance(self):
        # Define the origin and orientation of the frame
        origin = np.array([0, 0])  # Change this to your desired origin
        
        theta = 0

        # calculate the x-axis and y-axis
        x_axis = np.array([np.cos(theta), np.sin(theta)])
        y_axis = np.array([-np.sin(theta), np.cos(theta)])

        # Define arrow lengths
        arrow_length = 3

        # Calculate arrow endpoints
        x_arrow_end = origin + arrow_length * x_axis
        y_arrow_end = origin + arrow_length * y_axis

        # calculate the head width and length based on the arrow length
        head_width = arrow_length / 15
        head_length = arrow_length / 10

        # Create the plot
        plt.figure(figsize=(8, 6))

        # Plot the arrows
        plt.arrow(origin[0], origin[1], x_arrow_end[0] - origin[0], x_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='r', ec='r', label='X-Axis')
        plt.arrow(origin[0], origin[1], y_arrow_end[0] - origin[0], y_arrow_end[1] - origin[1],
                head_width=head_width, head_length=head_length, fc='g', ec='g', label='Y-Axis')

        # Plot the origin point
        plt.plot(origin[0], origin[1], 'go', label='Origin')
        # annotate the origin
        plt.annotate('O', (0, 0), (0 - 0.2, 0 - 0.2), size=15, zorder=2, color='k')

        # plot two parallel lines
        plt.plot([0.5, 1.0], [0.5, 2.5], 'r')
        plt.plot([2.0, 2.0], [0.5, 2.0], 'r')

        # plot a shortest perpendicular line between the two lines
        plt.plot([0.87, 2.0], [2.0, 2.0], 'c--')

        # Set plot limits
        plt.xlim(-1, origin[0] + arrow_length + 1)
        plt.ylim(-1, origin[1] + arrow_length + 1)

        # Add labels and legend
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('XY Frame with Vector')
        plt.legend()

        # Show the plot
        plt.grid()
        plt.show()


    def test(self):
        pass
                

if __name__ == '__main__':
    plotter = Plotter()
    # plotter.plot_point('P', np.array([0., 0., 1.3]), d2=True)
    # plotter.plot_frame_point()
    # plotter.plot_vector()
    # plotter.plot_rigid_body()
    # plotter.plot_point_to_point_distance()
    # plotter.plot_point_to_line_distance()
    plotter.plot_line_to_line_distance()