#!/usr/bin/env python3
"""
Web-based Drone Visualizer - Opens in browser, no focus stealing
Access at http://localhost:8050
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from collections import deque
import json
import threading
import plotly.graph_objects as go
from flask import Flask, render_template_string
import plotly.offline as pyo

class WebVisualizer(Node):
    def __init__(self):
        super().__init__('web_visualizer')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to drone position
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_position,
            qos_profile=self.mavros_qos
        )
        
        # Subscribe to pattern waypoints
        self.waypoints_sub = self.create_subscription(
            String, '/pattern_waypoints',
            self.update_waypoints, 10
        )
        
        # Data storage
        self.path_history = deque(maxlen=500)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.waypoints = []
        self.first_position_received = False
        self.start_position = None
        
        # Flask app for serving the plot
        self.app = Flask(__name__)
        self.setup_flask_routes()
        
        # Start Flask in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask)
        self.flask_thread.daemon = True
        self.flask_thread.start()
        
        self.get_logger().info('Web Visualizer started at http://localhost:8050')
    
    def update_position(self, msg):
        """Update drone position from ROS"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        if not self.first_position_received:
            self.start_position = (self.current_x, self.current_y, self.current_z)
            self.first_position_received = True
            self.get_logger().info(f'Tracking from: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}')
        
        self.path_history.append((self.current_x, self.current_y))
    
    def update_waypoints(self, msg):
        """Update waypoint list"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data.get('waypoints', [])
        except:
            pass
    
    def create_plot(self):
        """Create plotly figure"""
        fig = go.Figure()
        
        # Add path trace
        if len(self.path_history) > 1:
            path_x = [p[0] for p in self.path_history]
            path_y = [p[1] for p in self.path_history]
            fig.add_trace(go.Scatter(
                x=path_x, y=path_y,
                mode='lines',
                name='Drone Path',
                line=dict(color='blue', width=2)
            ))
        
        # Add current position
        fig.add_trace(go.Scatter(
            x=[self.current_x], y=[self.current_y],
            mode='markers',
            name='Current Position',
            marker=dict(color='red', size=12)
        ))
        
        # Add start position
        if self.start_position:
            fig.add_trace(go.Scatter(
                x=[self.start_position[0]], y=[self.start_position[1]],
                mode='markers',
                name=f'Start (Alt: {self.start_position[2]:.1f}m)',
                marker=dict(color='green', size=12, symbol='triangle-up')
            ))
        
        # Add waypoints
        if self.waypoints:
            wp_x = [w['x'] for w in self.waypoints]
            wp_y = [w['y'] for w in self.waypoints]
            fig.add_trace(go.Scatter(
                x=wp_x, y=wp_y,
                mode='lines+markers',
                name='Waypoints',
                line=dict(color='green', dash='dash'),
                marker=dict(color='green', size=8)
            ))
        
        # Update layout
        fig.update_layout(
            title=f'Drone Position Tracker - Altitude: {self.current_z:.1f}m',
            xaxis_title='X Position (meters)',
            yaxis_title='Y Position (meters)',
            hovermode='closest',
            showlegend=True,
            autosize=True,
            template='plotly_white',
            uirevision='constant',  # Preserve zoom/pan
            xaxis=dict(scaleanchor='y', scaleratio=1),  # Equal aspect ratio
        )
        
        # Add grid
        fig.update_xaxes(showgrid=True, gridwidth=1, gridcolor='LightGray')
        fig.update_yaxes(showgrid=True, gridwidth=1, gridcolor='LightGray')
        
        return fig
    
    def setup_flask_routes(self):
        """Setup Flask routes"""
        @self.app.route('/')
        def index():
            if not self.first_position_received:
                html = '''
                <html>
                <head>
                    <title>Drone Visualizer</title>
                    <meta http-equiv="refresh" content="2">
                </head>
                <body>
                    <h1>Waiting for drone position...</h1>
                    <p>Make sure the system is running.</p>
                </body>
                </html>
                '''
                return html
            
            fig = self.create_plot()
            graph_html = pyo.plot(fig, output_type='div', include_plotlyjs='cdn')
            
            html = f'''
            <html>
            <head>
                <title>Drone Visualizer</title>
                <meta http-equiv="refresh" content="2">
                <style>
                    body {{ font-family: Arial, sans-serif; margin: 20px; }}
                    .info {{ background: #f0f0f0; padding: 10px; border-radius: 5px; margin: 10px 0; }}
                </style>
            </head>
            <body>
                <h1>Drone Position Visualizer</h1>
                <div class="info">
                    <strong>Current Position:</strong> X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}<br>
                    <strong>Path Points:</strong> {len(self.path_history)}<br>
                    <strong>Auto-refresh:</strong> Every 2 seconds
                </div>
                {graph_html}
            </body>
            </html>
            '''
            return html
    
    def run_flask(self):
        """Run Flask server"""
        self.app.run(host='0.0.0.0', port=8050, debug=False)

def main(args=None):
    rclpy.init(args=args)
    node = WebVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()