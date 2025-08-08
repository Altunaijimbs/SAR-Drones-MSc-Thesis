#!/usr/bin/env python3
"""
Safe Pattern Runner - Ensures proper landing after any pattern
Wraps pattern execution with safety features
"""

import cosysairsim as airsim
import time
import sys
import signal

class SafePatternRunner:
    def __init__(self):
        self.client = None
        self.connected = False
        self.in_flight = False
        
        # Register signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self.emergency_stop)
        signal.signal(signal.SIGTERM, self.emergency_stop)
        
    def connect(self):
        """Connect to AirSim with safety checks"""
        print("Connecting to AirSim...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.connected = True
        print("✅ Connected!")
        
    def takeoff(self, altitude=15):
        """Safe takeoff procedure"""
        if not self.connected:
            self.connect()
            
        print("Arming...")
        self.client.armDisarm(True)
        
        state = self.client.getMultirotorState()
        if state.landed_state == airsim.LandedState.Landed:
            print(f"Taking off to {altitude}m...")
            self.client.takeoffAsync().join()
            self.in_flight = True
            
            # Move to altitude
            z = -altitude
            self.client.moveToZAsync(z, velocity=3).join()
            print(f"✅ At altitude: {altitude}m")
        else:
            print("Already in flight")
            self.in_flight = True
            
    def land(self):
        """Safe landing procedure"""
        if not self.in_flight:
            print("Already on ground")
            return
            
        try:
            print("\n" + "="*40)
            print("LANDING SEQUENCE")
            print("="*40)
            
            # First hover to stabilize
            print("Stabilizing...")
            self.client.hoverAsync().join()
            time.sleep(1)
            
            # Descend to low altitude first
            print("Descending to 5m...")
            self.client.moveToZAsync(-5, velocity=2).join()
            
            # Final landing
            print("Landing...")
            self.client.landAsync().join()
            
            # Wait for landing to complete
            time.sleep(2)
            
            # Disarm
            print("Disarming...")
            self.client.armDisarm(False)
            
            # Release control
            print("Releasing API control...")
            self.client.enableApiControl(False)
            
            self.in_flight = False
            print("✅ Landed safely!")
            print("="*40)
            
        except Exception as e:
            print(f"Error during landing: {e}")
            self.emergency_stop()
            
    def emergency_stop(self, signum=None, frame=None):
        """Emergency stop - called on errors or interrupts"""
        print("\n⚠️  EMERGENCY STOP INITIATED")
        
        if self.client and self.in_flight:
            try:
                # Try immediate landing
                print("Executing emergency landing...")
                self.client.landAsync().join()
                self.client.armDisarm(False)
                self.client.enableApiControl(False)
                self.in_flight = False
                print("✅ Emergency landing complete")
            except Exception as e:
                print(f"Emergency landing failed: {e}")
                # Last resort - just disarm
                try:
                    self.client.armDisarm(False)
                    self.client.enableApiControl(False)
                except:
                    pass
                    
        if signum:
            sys.exit(1)
            
    def fly_square(self, size=20, speed=5):
        """Fly a square pattern with guaranteed landing"""
        # Get current position
        pose = self.client.simGetVehiclePose()
        start_x = pose.position.x_val
        start_y = pose.position.y_val
        z = pose.position.z_val
        
        # Create square waypoints
        half = size / 2
        waypoints = [
            (start_x + half, start_y - half),  # Front-right
            (start_x + half, start_y + half),  # Right-back
            (start_x - half, start_y + half),  # Back-left
            (start_x - half, start_y - half),  # Left-front
            (start_x, start_y),                # Return to start
        ]
        
        print(f"\nFlying {size}m square at {speed} m/s...")
        
        for i, (x, y) in enumerate(waypoints):
            corner_names = ["Front-right", "Back-right", "Back-left", "Front-left", "Center"]
            print(f"  → {corner_names[i]}: ({x:.1f}, {y:.1f})")
            self.client.moveToPositionAsync(x, y, z, velocity=speed).join()
            time.sleep(0.5)  # Brief pause at corners
            
        print("✅ Pattern complete!")
        
    def fly_circle(self, radius=15, speed=3, segments=16):
        """Fly a circle pattern with guaranteed landing"""
        import math
        
        # Get current position as center
        pose = self.client.simGetVehiclePose()
        center_x = pose.position.x_val
        center_y = pose.position.y_val
        z = pose.position.z_val
        
        print(f"\nFlying {radius}m radius circle at {speed} m/s...")
        
        waypoints = []
        for i in range(segments + 1):  # +1 to complete the circle
            angle = 2 * math.pi * i / segments
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            waypoints.append((x, y))
            
        # Execute circle
        for i, (x, y) in enumerate(waypoints):
            progress = int((i / segments) * 100)
            print(f"  Circle progress: {progress}%", end='\r')
            self.client.moveToPositionAsync(x, y, z, velocity=speed).join()
            
        print("\n✅ Circle complete!")
        
        # Return to center
        print("Returning to center...")
        self.client.moveToPositionAsync(center_x, center_y, z, velocity=speed).join()
        
    def run_pattern(self, pattern_name="square", **kwargs):
        """Run a pattern with full safety wrapper"""
        try:
            # Connect and takeoff
            self.connect()
            self.takeoff(altitude=kwargs.get('altitude', 15))
            
            # Execute chosen pattern
            if pattern_name == "square":
                self.fly_square(
                    size=kwargs.get('size', 20),
                    speed=kwargs.get('speed', 5)
                )
            elif pattern_name == "circle":
                self.fly_circle(
                    radius=kwargs.get('radius', 15),
                    speed=kwargs.get('speed', 3)
                )
            else:
                print(f"Unknown pattern: {pattern_name}")
                
            # Hover to show completion
            print("\nHovering for 3 seconds...")
            self.client.hoverAsync().join()
            time.sleep(3)
            
        except KeyboardInterrupt:
            print("\n⚠️  Pattern interrupted by user")
            
        except Exception as e:
            print(f"\n❌ Error during pattern: {e}")
            
        finally:
            # ALWAYS land, no matter what
            self.land()
            
def main():
    """Example usage"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Safe Pattern Runner')
    parser.add_argument('--pattern', default='square', 
                       choices=['square', 'circle'],
                       help='Pattern to fly')
    parser.add_argument('--size', type=float, default=20,
                       help='Size/radius of pattern')
    parser.add_argument('--speed', type=float, default=5,
                       help='Flight speed in m/s')
    parser.add_argument('--altitude', type=float, default=15,
                       help='Flight altitude in meters')
    args = parser.parse_args()
    
    print("="*60)
    print("SAFE PATTERN RUNNER")
    print("="*60)
    print("This script guarantees proper landing after patterns")
    print("Press Ctrl+C at any time for emergency landing")
    print("="*60)
    
    runner = SafePatternRunner()
    runner.run_pattern(
        pattern_name=args.pattern,
        size=args.size,
        speed=args.speed,
        altitude=args.altitude
    )
    
    print("\nFlight session complete!")

if __name__ == "__main__":
    main()