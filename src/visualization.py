"""Utility to visualize photo plans.
"""

import typing as T

import plotly.graph_objects as go

from src.data_model import Waypoint


def plot_photo_plan(photo_plans: T.List[Waypoint]) -> go.Figure:
    """Plot the photo plan on a 2D grid.

    Args:
        photo_plans: List of waypoints for the photo plan.

    Returns:
        Plotly figure object.
    """
    if not photo_plans or len(photo_plans) == 0:
        raise ValueError("photo_plans must be a non-empty list of Waypoint objects")

    # Extract x, y, z, and speed from waypoints
    xs = [wp.x for wp in photo_plans]
    ys = [wp.y for wp in photo_plans]
    zs = [wp.z for wp in photo_plans]
    speeds = [wp.speed for wp in photo_plans]

    # Create scatter plot for waypoints
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=xs,
        y=ys,
        mode='markers+lines',
        marker=dict(
            size=8,
            color=zs,
            colorscale='Viridis',
            line=dict(width=1, color='DarkSlateGrey')
        ),
        line=dict(color='royalblue', width=2),
        text=[f"Height: {z:.2f} m<br>Speed: {s:.2f} m/s" for z, s in zip(zs, speeds)],
        hoverinfo='text',
        name='Waypoints'
    ))
    fig.update_layout(
        title="Flight Plan Waypoints",
        xaxis_title="X Position (m)",
        yaxis_title="Y Position (m)",
        legend=dict(x=0.01, y=0.99),
        margin=dict(l=40, r=40, t=60, b=40),
        height=600,
        width=800,
    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    return fig
