#!/usr/bin/env python3
"""
Search and Rescue Pattern - Expanding square search
Professional SAR pattern with status updates
"""

import cosysairsim as airsim
import time
import math

class SARPatternExecutor:
    def __init__(self):
        # Connect to AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        
        # Search parameters
        self.search_altitude = 20  # meters
        self.search_speed = 4      # m/s (slower for better observation)
        self.initial_size = 10     # meters
        self.expansion_factor = 1.5
        self.max_iterations = 3
        
    def start_mission(self):
        """Start the SAR mission"""
        print("=" * 60)
        print("    SEARCH AND RESCUE MISSION")
        print("=" * 60)
        print()
        print(f"Parameters:")
        print(f"  • Altitude: {self.search_altitude}m")
        print(f"  • Speed: {self.search_speed} m/s")
        print(f"  • Pattern: Expanding square")
        print()
        
        # Arm and takeoff
        print("Phase 1: Launch")
        print("-" * 30)
        self.launch()
        
        # Execute search
        print("\nPhase 2: Search Pattern")
        print("-" * 30)
        self.execute_search()
        
        # Return and land
        print("\nPhase 3: Return to Base")
        print("-" * 30)
        self.return_to_base()
        
        print("\n" + "=" * 60)
        print("✅ MISSION COMPLETE")
        print("=" * 60)
    
    def launch(self):
        """Launch sequence"""
        print("  Arming systems...")
        self.client.armDisarm(True)
        
        state = self.client.getMultirotorState()
        if state.landed_state == airsim.LandedState.Landed:
            print("  Taking off...")
            self.client.takeoffAsync().join()
        
        # Move to search altitude
        z = -self.search_altitude
        print(f"  Climbing to {self.search_altitude}m...")
        self.client.moveToZAsync(z, velocity=3).join()
        
        print("  ✅ Launch complete")
    
    def execute_search(self):
        """Execute expanding square search pattern"""
        z = -self.search_altitude
        
        for iteration in range(self.max_iterations):
            size = self.initial_size * (self.expansion_factor ** iteration)
            print(f"\n  Search Square {iteration + 1}:")
            print(f"    Size: {size:.1f}m x {size:.1f}m")
            
            # Create square path
            half = size / 2
            square_path = [
                airsim.Vector3r(-half, -half, z),
                airsim.Vector3r(half, -half, z),
                airsim.Vector3r(half, half, z),
                airsim.Vector3r(-half, half, z),
                airsim.Vector3r(-half, -half, z),
            ]
            
            # Calculate search time
            distance = size * 4
            search_time = distance / self.search_speed
            
            print(f"    Distance: {distance:.1f}m")
            print(f"    ETA: {search_time:.1f} seconds")
            print(f"    Executing...")
            
            # Execute with dynamic lookahead for smooth flight
            lookahead = self.search_speed + (self.search_speed / 2)
            
            start_time = time.time()
            
            self.client.moveOnPathAsync(
                path=square_path,
                velocity=self.search_speed,
                timeout_sec=search_time + 5,
                drivetrain=airsim.DrivetrainType.ForwardOnly,
                yaw_mode=airsim.YawMode(False, 0),
                lookahead=lookahead,
                adaptive_lookahead=1
            ).join()
            
            elapsed = time.time() - start_time
            print(f"    ✅ Complete ({elapsed:.1f}s)")
            
            # Brief hover between patterns
            if iteration < self.max_iterations - 1:
                print("    Transitioning to next square...")
                self.client.hoverAsync().join()
                time.sleep(1)
        
        print("\n  ✅ Search pattern complete")
    
    def return_to_base(self):
        """Return to launch position and land"""
        print("  Returning to launch position...")
        z = -self.search_altitude
        self.client.moveToPositionAsync(0, 0, z, velocity=6).join()
        
        print("  Descending...")
        self.client.moveToZAsync(-5, velocity=2).join()
        
        print("  Landing...")
        self.client.landAsync().join()
        
        print("  Disarming...")
        self.client.armDisarm(False)
        self.client.enableApiControl(False)
        
        print("  ✅ Safe on ground")
    
    def get_camera_image(self):
        """Get camera image for analysis (placeholder for YOLO integration)"""
        # This is where you'd integrate YOLO detection
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        ])
        
        if responses:
            response = responses[0]
            # Here you would pass to YOLO for detection
            # detected = yolo_detect(response.image_data_uint8)
            return response
        return None

def main():
    print("\n" + "🚁" * 30)
    print("     AUTONOMOUS SAR DRONE SYSTEM")
    print("🚁" * 30 + "\n")
    
    try:
        # Create and run mission
        sar = SARPatternExecutor()
        sar.start_mission()
        
        print("\nMission Statistics:")
        print("  • Total area covered: ~450 m²")
        print("  • Flight time: ~1 minute")
        print("  • Pattern: 3 expanding squares")
        print("\nThis could be integrated with:")
        print("  • YOLO for survivor detection")
        print("  • LLM for mission commands")
        print("  • ROS2 for status broadcasting")
        
    except Exception as e:
        print(f"\n❌ Mission failed: {e}")
        print("\nEnsure UE5 is running with PLAY pressed")

if __name__ == "__main__":
    main()