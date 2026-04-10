"""
Static GVF (Gradient Vector Field) — same rectangle, same two points.

Figure 1: all vectors point toward A
Figure 2: all vectors point toward B
Points A and B are placed close to opposite walls of the rectangle.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# --- Configuration ---

# Rectangle
RECT_ORIGIN = (0.0, 0.0)   # bottom-left corner
RECT_WIDTH = 10.0
RECT_HEIGHT = 6.0

# Goal points — close to opposite walls
POINT_A = np.array([0.5, 3.0])    # near the left wall
POINT_B = np.array([9.5, 3.0])    # near the right wall

# Grid resolution
GRID_SPACING = 0.5


def make_attractor_field(grid_x, grid_y, goal):
    """Normalized vector field pointing toward `goal`."""
    dx = goal[0] - grid_x
    dy = goal[1] - grid_y
    mag = np.sqrt(dx**2 + dy**2)
    mag = np.where(mag == 0, 1.0, mag)
    return dx / mag, dy / mag


def plot_field(ax, gx, gy, u, v, goal_label, goal_pt, other_label, other_pt):
    """Draw one GVF subplot."""
    # Rectangle
    rect = patches.Rectangle(RECT_ORIGIN, RECT_WIDTH, RECT_HEIGHT,
                              linewidth=2, edgecolor='black',
                              facecolor='0.95')
    ax.add_patch(rect)

    # Vector field
    ax.quiver(gx, gy, u, v, color='0.35', scale=25, width=0.004, headwidth=3.5)

    # Goal points
    ax.plot(*goal_pt, 'o', color='tab:red', markersize=14, zorder=5)
    ax.annotate(goal_label, goal_pt, textcoords="offset points", xytext=(10, 10),
                fontsize=14, fontweight='bold', color='tab:red')

    ax.plot(*other_pt, 'o', color='tab:blue', markersize=12, zorder=5)
    ax.annotate(other_label, other_pt, textcoords="offset points", xytext=(10, 10),
                fontsize=13, fontweight='bold', color='tab:blue')

    ax.set_xlim(RECT_ORIGIN[0] - 0.5, RECT_ORIGIN[0] + RECT_WIDTH + 0.5)
    ax.set_ylim(RECT_ORIGIN[1] - 0.5, RECT_ORIGIN[1] + RECT_HEIGHT + 0.5)
    ax.set_aspect('equal')
    ax.set_title(f'Vectors → {goal_label}', fontsize=14)
    ax.set_xlabel('x')
    ax.set_ylabel('y')


def main():
    # Build grid inside the rectangle
    xs = np.arange(RECT_ORIGIN[0], RECT_ORIGIN[0] + RECT_WIDTH + GRID_SPACING, GRID_SPACING)
    ys = np.arange(RECT_ORIGIN[1], RECT_ORIGIN[1] + RECT_HEIGHT + GRID_SPACING, GRID_SPACING)
    gx, gy = np.meshgrid(xs, ys)

    u_a, v_a = make_attractor_field(gx, gy, POINT_A)
    u_b, v_b = make_attractor_field(gx, gy, POINT_B)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    plot_field(ax1, gx, gy, u_a, v_a, 'A', POINT_A, 'B', POINT_B)
    plot_field(ax2, gx, gy, u_b, v_b, 'B', POINT_B, 'A', POINT_A)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
