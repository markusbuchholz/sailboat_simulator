import tkinter as tk
from tkinter import messagebox
import numpy as np
from matplotlib.path import Path
from shapely.geometry import Polygon, Point, LineString, LinearRing
from shapely.ops import unary_union

class CoordinateApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Obstacle Avoidance Planner")
        self.canvas_size = 800
        self.node_radius = 5
        self.obstacle_radius = 1.0
        self.grid_size = 50
        self.polygon_points = []
        self.dynamic_obstacles = []
        self.polygon_mode = False
        self.polygon = None

        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=15, rowspan=15, sticky='nsew')

        self.polygon_point_button = tk.Button(root, text="Add Polygon Point", command=self.add_polygon_point)
        self.polygon_point_button.grid(row=16, column=0, sticky='nsew')

        self.finalize_polygon_button = tk.Button(root, text="Finalize Polygon", command=self.finalize_polygon)
        self.finalize_polygon_button.grid(row=16, column=1, sticky='nsew')

        self.clear_button = tk.Button(root, text="Clear All", command=self.clear_canvas)
        self.clear_button.grid(row=16, column=2, sticky='nsew')

        self.add_dynamic_obstacle_button = tk.Button(root, text="Add Dynamic Obstacle", command=self.add_dynamic_obstacle)
        self.add_dynamic_obstacle_button.grid(row=16, column=3, sticky='nsew')

        self.replan_polygon_button = tk.Button(root, text="Replan Polygon", command=self.replan_polygon)
        self.replan_polygon_button.grid(row=16, column=4, sticky='nsew')

        self.canvas.bind("<Button-1>", self.get_coordinates)
        self.mode = None

    def add_polygon_point(self):
        self.polygon_mode = True

    def finalize_polygon(self):
        if len(self.polygon_points) < 3:
            messagebox.showerror("Error", "A polygon must have at least 3 points.")
            return
        self.polygon_mode = False
        self.polygon_points.append(self.polygon_points[0])  # Close the polygon
        self.polygon = Polygon(self.polygon_points)
        for i in range(len(self.polygon_points) - 1):
            x1, y1 = self.polygon_points[i]
            x2, y2 = self.polygon_points[i + 1]
            self.canvas.create_line(x1, y1, x2, y2, fill="blue")

    def clear_canvas(self):
        self.canvas.delete("all")
        self.polygon_points = []
        self.dynamic_obstacles = []
        self.polygon = None

    def add_dynamic_obstacle(self):
        self.mode = "dynamic_obstacle"

    def get_coordinates(self, event):
        x, y = event.x, event.y
        coord = (x, y)

        if self.polygon_mode:
            self.polygon_points.append(coord)
            self.canvas.create_oval(coord[0] - self.node_radius, coord[1] - self.node_radius,
                                    coord[0] + self.node_radius, coord[1] + self.node_radius, fill="purple")
        elif self.mode == "dynamic_obstacle":
            self.dynamic_obstacles.append(coord)
            self.canvas.create_oval(coord[0] - self.obstacle_radius * self.grid_size, coord[1] - self.obstacle_radius * self.grid_size,
                                    coord[0] + self.obstacle_radius * self.grid_size, coord[1] + self.obstacle_radius * self.grid_size, fill="red")

    def replan_polygon(self):
        if not self.polygon:
            messagebox.showerror("Error", "Please define a polygon first.")
            return

        new_polygon_points = self.polygon_points[:-1]
        for obs in self.dynamic_obstacles:
            new_polygon_points = self.avoid_obstacle(new_polygon_points, obs)

        new_polygon_points.append(new_polygon_points[0])  # Close the new polygon

        self.canvas.delete("all")
        for i in range(len(new_polygon_points) - 1):
            x1, y1 = new_polygon_points[i]
            x2, y2 = new_polygon_points[i + 1]
            self.canvas.create_line(x1, y1, x2, y2, fill="green")

        self.polygon_points = new_polygon_points
        self.polygon = Polygon(self.polygon_points)

        for obs in self.dynamic_obstacles:
            self.canvas.create_oval(obs[0] - self.obstacle_radius * self.grid_size, obs[1] - self.obstacle_radius * self.grid_size,
                                    obs[0] + self.obstacle_radius * self.grid_size, obs[1] + self.obstacle_radius * self.grid_size, fill="red")

    def avoid_obstacle(self, points, obstacle):
        obs_point = Point(obstacle)
        obstacle_buffer = obs_point.buffer(self.obstacle_radius * self.grid_size * 1.5)
        polygon = Polygon(points)

        # Check if the obstacle is inside the polygon
        if polygon.contains(obs_point):
            new_polygon = polygon.difference(obstacle_buffer)
            exterior_coords = list(new_polygon.exterior.coords)
            return exterior_coords[:-1]

        new_points = []
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]
            line = LineString([p1, p2])
            if line.intersects(obstacle_buffer):
                intersection_points = line.intersection(obstacle_buffer)
                if intersection_points.geom_type == 'MultiPoint':
                    intersection_points = list(intersection_points)
                    for pt in intersection_points:
                        new_points.append((pt.x, pt.y))
                elif intersection_points.geom_type == 'Point':
                    new_points.append((intersection_points.x, intersection_points.y))
                new_points.append(p2)
            else:
                new_points.append(p1)

        return new_points

def main():
    root = tk.Tk()
    app = CoordinateApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
