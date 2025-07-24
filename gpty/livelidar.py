import airsim
import numpy as np
import dash
from dash import dcc, html
from dash.dependencies import Output, Input
import plotly.graph_objs as go

# AirSim client setup
client = airsim.MultirotorClient()
client.confirmConnection()

app = dash.Dash(__name__)

app.layout = html.Div([
    html.H2("Live AirSim Lidar Stream"),
    dcc.Graph(id='lidar-plot'),
    dcc.Interval(id='interval-component', interval=1000, n_intervals=0)  # Update every second
])

@app.callback(
    Output('lidar-plot', 'figure'),
    Input('interval-component', 'n_intervals')
)


def update_lidar(n):
    lidar_data = client.getLidarData(lidar_name="Lidar1")
    if lidar_data.point_cloud:
        points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        # Flip Z axis so up in AirSim is up in the plot
        points[:,2] = -points[:,2]

        scatter = go.Scatter3d(
            x=points[:,0], 
            y=points[:,1], 
            z=points[:,2],  # Now Z is up
            mode='markers',
            marker=dict(size=2, color=points[:,2], colorscale='Viridis', opacity=0.7)
        )
        layout = go.Layout(
            scene=dict(
                xaxis_title='X (Forward, m)',
                yaxis_title='Y (Right, m)',
                zaxis_title='Z (Up, m)'   # Change label to "Up"
            ),
            margin=dict(l=0, r=0, b=0, t=40)
        )
        fig = go.Figure(data=[scatter], layout=layout)
        return fig
    else:
        return go.Figure()


if __name__ == '__main__':
    app.run(debug=True)
