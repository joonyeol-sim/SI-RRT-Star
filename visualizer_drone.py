import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from scipy.spatial.transform import Rotation as RR
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d.axes3d import Axes3D


class Arrow3D(FancyArrowPatch):
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)


def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)
    return arrow


setattr(Axes3D, "arrow3D", _arrow3D)


class Sphere2:
    def __init__(self, ax, radius, color):
        self.ax = ax
        self.radius = radius
        self.color = color
        self.sphere = None
    
    def draw_at(self, position, **kwargs):
        color = kwargs.get('color', self.color)
        self.sphere = self.ax.scatter(
            [position[0]],
            [position[1]], 
            [position[2]],
            s=self.radius*200,
            marker="o",
            **kwargs
        )
    
    def delete(self):
        if self.sphere is not None:
            self.sphere.remove()


class Line:
    def __init__(self, ax, color):
        self.ax = ax
        self.color = color
        self.line = None
    
    def draw_from_to(self, start, end, **kwargs):
        color = kwargs.get('color', self.color)
        self.line, = self.ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], 
                                 color=color, linewidth=2)
    
    def delete(self):
        if self.line is not None:
            self.line.remove()


class Uav:
    def __init__(self, ax, arm_length):
        self.ax = ax
        self.arm_length = arm_length

        self.b1 = np.array([1.0, 0.0, 0.0]).T
        self.b2 = np.array([0.0, 1.0, 0.0]).T
        self.b3 = np.array([0.0, 0.0, 1.0]).T

        self.body = Sphere2(self.ax, 0.08, "y")
        self.motor1 = Sphere2(self.ax, 0.05, "r")
        self.motor2 = Sphere2(self.ax, 0.05, "g")
        self.motor3 = Sphere2(self.ax, 0.05, "b")
        self.motor4 = Sphere2(self.ax, 0.05, "b")

        self.arm_b1 = Line(ax, "k")
        self.arm_b2 = Line(ax, "k")
        self.arm_b3 = Line(ax, "k")
        self.arm_b4 = Line(ax, "k")

    def delete(self):
        self.body.delete()
        self.motor1.delete()
        self.motor2.delete()
        self.motor3.delete()
        self.motor4.delete()
        self.arm_b1.delete()
        self.arm_b2.delete()
        self.arm_b3.delete()
        self.arm_b4.delete()

    def draw_at(self, x=np.array([0.0, 0.0, 0.0]).T, R=np.eye(3), **kwargs):
        self.body.draw_at(x, **kwargs)
        self.motor1.draw_at(x + R.dot(self.b1) * self.arm_length, **kwargs)
        self.motor2.draw_at(x + R.dot(self.b2) * self.arm_length, **kwargs)
        self.motor3.draw_at(x + R.dot(-self.b1) * self.arm_length, **kwargs)
        self.motor4.draw_at(x + R.dot(-self.b2) * self.arm_length, **kwargs)

        self.arm_b1.draw_from_to(x, x + R.dot(-self.b1) * self.arm_length, **kwargs)
        self.arm_b2.draw_from_to(x, x + R.dot(-self.b2) * self.arm_length, **kwargs)
        self.arm_b3.draw_from_to(x, x + R.dot(self.b1) * self.arm_length, **kwargs)
        self.arm_b4.draw_from_to(x, x + R.dot(self.b2) * self.arm_length, **kwargs)


class Robot:
    def __init__(self):
        self.h = None
        self.uav = None
        self.arrow = None

    def draw(self, ax, x, **kwargs):
        self.h, = ax.plot([x[0]], [x[1]], [x[2]], color=".5", linestyle="", marker=".", zorder=100)

        arm_length = 0.24
        self.uav = Uav(ax, arm_length)
        q = x[3:7]
        p = np.array(x[:3])
        R_mat = RR.from_quat(q).as_matrix()
        self.uav.draw_at(p, R_mat, **kwargs, zorder=200)

        scale = 0.4
        self.arrow = ax.arrow3D(
            p[0], p[1], p[2],
            scale * R_mat[0, 2],
            scale * R_mat[1, 2],
            scale * R_mat[2, 2],
            mutation_scale=10,
            arrowstyle="-|>",
            **kwargs,
        )

    def draw_traj_minimal(self, ax, Xs):
        xs = [p[0] for p in Xs]
        ys = [p[1] for p in Xs]
        zs = [p[2] for p in Xs]
        ax.plot3D(xs, ys, zs, "blue", alpha=0.5, zorder=100)


class DroneVisualizer:
    def __init__(self):
        self.fig = None
        self.ax = None
        
    def setup_plot(self, figsize=(10, 8)):
        self.fig = plt.figure(figsize=figsize)
        self.ax = plt.axes(projection="3d")
        
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        
        return self.fig, self.ax
    
    def set_limits(self, xlim=(-2, 2), ylim=(-2, 2), zlim=(0, 3)):
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_zlim(zlim)
    
    def draw_obstacles(self, obstacles):
        for obs in obstacles:
            if obs["type"] == "sphere":
                self.draw_sphere(obs["center"], obs["radius"])
            elif obs["type"] == "box":
                self.draw_box(obs["center"], obs["size"])
    
    def draw_sphere(self, center, radius):
        u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
        x = radius * np.cos(u) * np.sin(v) + center[0]
        y = radius * np.sin(u) * np.sin(v) + center[1]
        z = radius * np.cos(v) + center[2]
        self.ax.plot_surface(x, y, z, color="red", alpha=0.3)
    
    def draw_box(self, center, size):
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        o = [a - b / 2 for a, b in zip(center, size)]
        l, w, h = size
        
        vertices = [
            [o[0], o[1], o[2]],
            [o[0] + l, o[1], o[2]],
            [o[0] + l, o[1] + w, o[2]],
            [o[0], o[1] + w, o[2]],
            [o[0], o[1], o[2] + h],
            [o[0] + l, o[1], o[2] + h],
            [o[0] + l, o[1] + w, o[2] + h],
            [o[0], o[1] + w, o[2] + h]
        ]
        
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[4], vertices[5], vertices[6], vertices[7]],
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[2], vertices[3], vertices[7], vertices[6]],
            [vertices[1], vertices[2], vertices[6], vertices[5]],
            [vertices[4], vertices[7], vertices[3], vertices[0]]
        ]
        
        self.ax.add_collection3d(Poly3DCollection(faces, facecolors='gray', alpha=0.3, edgecolors='black'))
    
    def visualize_drone(self, state, **kwargs):
        robot = Robot()
        robot.draw(self.ax, state, **kwargs)
        return robot
    
    def visualize_trajectory(self, states, show_intermediate=True, intermediate_step=10):
        robot = Robot()
        robot.draw_traj_minimal(self.ax, states)
        
        if show_intermediate and len(states) > intermediate_step:
            for i in range(0, len(states), intermediate_step):
                r = Robot()
                r.draw(self.ax, states[i], alpha=0.3, color="blue")
    
    def animate_trajectory(self, states, interval=10, save_path=None):
        robot = Robot()
        robot.draw(self.ax, states[0], color="blue")
        
        def animate_func(i):
            if i < len(states):
                robot.uav.delete()
                robot.h.remove()
                robot.arrow.remove()
                robot.draw(self.ax, states[i], color="blue")
        
        anim = animation.FuncAnimation(
            self.fig, animate_func, frames=len(states), interval=interval, blit=False
        )
        
        if save_path:
            print(f"Saving animation to {save_path}")
            anim.save(save_path, "ffmpeg", fps=10, dpi=200)
        
        return anim
    
    def show(self):
        plt.show()


def compute_velocities_from_trajectory(positions, orientations, max_linear_speed=2.0, max_angular_speed=1.0, dt=0.1):
    """
    Compute velocities from position and orientation trajectory
    
    Args:
        positions: list of [x, y, z] positions
        orientations: list of [qx, qy, qz, qw] quaternions
        max_linear_speed: maximum linear velocity
        max_angular_speed: maximum angular velocity
        dt: time step between trajectory points
    
    Returns:
        velocities: list of [vx, vy, vz, wx, wy, wz] velocities
    """
    velocities = []
    
    for i in range(len(positions)):
        if i == 0:
            # First point - zero velocity
            vx = vy = vz = 0
            wx = wy = wz = 0
        else:
            # Compute linear velocity
            dp = np.array(positions[i]) - np.array(positions[i-1])
            v = dp / dt
            
            # Clamp to max speed
            v_norm = np.linalg.norm(v)
            if v_norm > max_linear_speed:
                v = v * (max_linear_speed / v_norm)
            
            vx, vy, vz = v
            
            # Compute angular velocity (simplified)
            q_prev = np.array(orientations[i-1])
            q_curr = np.array(orientations[i])
            
            # Simple finite difference for angular velocity
            dq = q_curr - q_prev
            w = 2 * dq[:3] / dt  # Simplified angular velocity computation
            
            # Clamp to max angular speed
            w_norm = np.linalg.norm(w)
            if w_norm > max_angular_speed:
                w = w * (max_angular_speed / w_norm)
                
            wx, wy, wz = w
        
        velocities.append([vx, vy, vz, wx, wy, wz])
    
    return velocities

def states_from_positions_orientations(positions, orientations, max_linear_speed=2.0, max_angular_speed=1.0):
    """
    Create full 13D states from positions and orientations with computed velocities
    
    Args:
        positions: list of [x, y, z] positions
        orientations: list of [qx, qy, qz, qw] quaternions
        max_linear_speed: maximum linear velocity
        max_angular_speed: maximum angular velocity
    
    Returns:
        states: list of 13D states [x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz]
    """
    velocities = compute_velocities_from_trajectory(positions, orientations, max_linear_speed, max_angular_speed)
    
    states = []
    for i in range(len(positions)):
        pos = positions[i]
        ori = orientations[i]
        vel = velocities[i]
        
        state = pos + ori + vel  # [x,y,z] + [qx,qy,qz,qw] + [vx,vy,vz,wx,wy,wz]
        states.append(state)
    
    return states

def generate_sample_trajectory(num_points=200):
    t = np.linspace(0, 4*np.pi, num_points)
    
    positions = []
    orientations = []
    
    for i in range(num_points):
        x = 2 * np.cos(t[i] * 0.3)
        y = 2 * np.sin(t[i] * 0.3)
        z = 1 + 0.5 * np.sin(t[i])
        
        qx = 0.1 * np.sin(t[i] * 0.5)
        qy = 0.1 * np.cos(t[i] * 0.5)
        qz = 0.1 * np.sin(t[i] * 0.2)
        qw = np.sqrt(1 - qx**2 - qy**2 - qz**2)
        
        positions.append([x, y, z])
        orientations.append([qx, qy, qz, qw])
    
    # Generate states with computed velocities
    states = states_from_positions_orientations(positions, orientations, 
                                              max_linear_speed=1.5, 
                                              max_angular_speed=0.8)
    
    return states


def main():
    visualizer = DroneVisualizer()
    fig, ax = visualizer.setup_plot()
    visualizer.set_limits(xlim=(-3, 3), ylim=(-3, 3), zlim=(0, 3))
    
    obstacles = [
        {"type": "sphere", "center": [0, 0, 1.5], "radius": 0.3},
        {"type": "box", "center": [1.5, 1.5, 0.5], "size": [0.5, 0.5, 1.0]}
    ]
    visualizer.draw_obstacles(obstacles)
    
    states = generate_sample_trajectory(100)
    
    start_state = states[0]
    goal_state = states[-1]
    visualizer.visualize_drone(start_state, color="green")
    visualizer.visualize_drone(goal_state, color="red")

    # Show trajectory with intermediate points
    visualizer.visualize_trajectory(states, show_intermediate=True, intermediate_step=100)

    # Create animation exactly like original viewer
    anim = visualizer.animate_trajectory(states, interval=20)

    # Optionally save animation
    # anim = visualizer.animate_trajectory(states, interval=50, save_path="drone_trajectory.mp4")

    visualizer.show()


if __name__ == "__main__":
    main()