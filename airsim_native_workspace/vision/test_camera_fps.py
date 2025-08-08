#!/usr/bin/env python3
"""
Camera FPS Test - Measure actual FPS achievable with different methods
"""

import cosysairsim as airsim
import numpy as np
import cv2
import time
import threading
from collections import deque

class FPSTest:
    def __init__(self):
        print("Connecting to AirSim...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("✅ Connected\n")
        
        self.fps_history = deque(maxlen=100)
        self.method_stats = {}
    
    def test_compressed_images(self, duration=10):
        """Test FPS with compressed images (PNG/JPEG from simGetImage)"""
        print("Testing Method 1: Compressed Images (simGetImage)")
        print("-" * 50)
        
        frames = 0
        start_time = time.time()
        errors = 0
        
        while time.time() - start_time < duration:
            try:
                # Get compressed image
                png_data = self.client.simGetImage("0", airsim.ImageType.Scene)
                
                if png_data:
                    # Optionally decode to measure full pipeline
                    nparr = np.frombuffer(png_data, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if img is not None:
                        frames += 1
                else:
                    errors += 1
                    
            except Exception as e:
                errors += 1
        
        elapsed = time.time() - start_time
        fps = frames / elapsed
        
        print(f"  Frames captured: {frames}")
        print(f"  Errors: {errors}")
        print(f"  Average FPS: {fps:.2f}")
        print(f"  Frame time: {1000/fps:.2f}ms")
        
        self.method_stats['compressed'] = fps
        return fps
    
    def test_uncompressed_images(self, duration=10):
        """Test FPS with uncompressed images (simGetImages)"""
        print("\nTesting Method 2: Uncompressed Images (simGetImages)")
        print("-" * 50)
        
        frames = 0
        start_time = time.time()
        errors = 0
        
        while time.time() - start_time < duration:
            try:
                responses = self.client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                ])
                
                if responses and len(responses) > 0:
                    response = responses[0]
                    if response.image_data_uint8:
                        # Convert to numpy (full pipeline)
                        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                        img_rgb = img1d.reshape(response.height, response.width, 3)
                        frames += 1
                else:
                    errors += 1
                    
            except Exception as e:
                errors += 1
        
        elapsed = time.time() - start_time
        fps = frames / elapsed
        
        print(f"  Frames captured: {frames}")
        print(f"  Errors: {errors}")
        print(f"  Average FPS: {fps:.2f}")
        print(f"  Frame time: {1000/fps:.2f}ms")
        
        self.method_stats['uncompressed'] = fps
        return fps
    
    def test_parallel_capture(self, duration=10):
        """Test FPS with parallel capture threads"""
        print("\nTesting Method 3: Parallel Capture (2 threads)")
        print("-" * 50)
        
        frames = [0, 0]
        start_time = time.time()
        stop_flag = threading.Event()
        
        def capture_thread(thread_id):
            client = airsim.MultirotorClient()
            client.confirmConnection()
            
            while not stop_flag.is_set():
                try:
                    png_data = client.simGetImage("0", airsim.ImageType.Scene)
                    if png_data:
                        frames[thread_id] += 1
                except:
                    pass
        
        # Start threads
        threads = []
        for i in range(2):
            t = threading.Thread(target=capture_thread, args=(i,))
            t.start()
            threads.append(t)
        
        # Wait for duration
        time.sleep(duration)
        stop_flag.set()
        
        # Wait for threads
        for t in threads:
            t.join()
        
        elapsed = time.time() - start_time
        total_frames = sum(frames)
        fps = total_frames / elapsed
        
        print(f"  Thread 1 frames: {frames[0]}")
        print(f"  Thread 2 frames: {frames[1]}")
        print(f"  Total frames: {total_frames}")
        print(f"  Average FPS: {fps:.2f}")
        print(f"  Frame time: {1000/fps:.2f}ms")
        
        self.method_stats['parallel'] = fps
        return fps
    
    def test_lower_resolution(self, duration=10):
        """Test FPS with lower resolution request"""
        print("\nTesting Method 4: Lower Resolution (960x540)")
        print("-" * 50)
        
        frames = 0
        start_time = time.time()
        errors = 0
        
        while time.time() - start_time < duration:
            try:
                # Get compressed image
                png_data = self.client.simGetImage("0", airsim.ImageType.Scene)
                
                if png_data:
                    # Decode and resize
                    nparr = np.frombuffer(png_data, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if img is not None:
                        # Resize to lower resolution
                        img_small = cv2.resize(img, (960, 540), interpolation=cv2.INTER_LINEAR)
                        frames += 1
                else:
                    errors += 1
                    
            except Exception as e:
                errors += 1
        
        elapsed = time.time() - start_time
        fps = frames / elapsed
        
        print(f"  Frames captured: {frames}")
        print(f"  Errors: {errors}")
        print(f"  Average FPS: {fps:.2f}")
        print(f"  Frame time: {1000/fps:.2f}ms")
        
        self.method_stats['low_res'] = fps
        return fps
    
    def run_live_fps_test(self):
        """Run live FPS test with display"""
        print("\nLive FPS Test (Press 'q' to stop)")
        print("-" * 50)
        
        cv2.namedWindow("FPS Test", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("FPS Test", 1280, 720)
        
        frame_times = deque(maxlen=100)
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                frame_start = time.time()
                
                # Get image
                png_data = self.client.simGetImage("0", airsim.ImageType.Scene)
                
                if png_data:
                    # Decode
                    nparr = np.frombuffer(png_data, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if img is not None:
                        # Resize for display
                        img = cv2.resize(img, (1280, 720))
                        
                        # Calculate FPS
                        frame_time = time.time() - frame_start
                        frame_times.append(frame_time)
                        
                        if len(frame_times) > 10:
                            avg_frame_time = sum(frame_times) / len(frame_times)
                            current_fps = 1.0 / avg_frame_time
                        else:
                            current_fps = 0
                        
                        frame_count += 1
                        
                        # Add overlay
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(img, f"FPS: {current_fps:.1f}", (10, 30), 
                                   font, 1, (0, 255, 0), 2)
                        cv2.putText(img, f"Frame: {frame_count}", (10, 60), 
                                   font, 0.7, (0, 255, 0), 2)
                        cv2.putText(img, f"Frame Time: {frame_time*1000:.1f}ms", (10, 90), 
                                   font, 0.7, (0, 255, 0), 2)
                        
                        # Display histogram of frame times
                        hist_height = 100
                        hist_width = 300
                        hist_img = np.zeros((hist_height, hist_width, 3), dtype=np.uint8)
                        
                        if len(frame_times) > 1:
                            max_time = max(frame_times)
                            for i, ft in enumerate(frame_times):
                                x = int(i * hist_width / len(frame_times))
                                h = int(ft / max_time * hist_height)
                                cv2.line(hist_img, (x, hist_height), (x, hist_height - h), (0, 255, 0), 1)
                        
                        # Overlay histogram
                        img[img.shape[0]-hist_height-10:img.shape[0]-10, 10:10+hist_width] = hist_img
                        
                        # Show
                        cv2.imshow("FPS Test", img)
                        
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                
        except KeyboardInterrupt:
            pass
        
        cv2.destroyAllWindows()
        
        elapsed = time.time() - start_time
        avg_fps = frame_count / elapsed
        
        print(f"\nFinal Statistics:")
        print(f"  Total frames: {frame_count}")
        print(f"  Total time: {elapsed:.1f}s")
        print(f"  Average FPS: {avg_fps:.1f}")

def main():
    print("=" * 60)
    print("CAMERA FPS PERFORMANCE TEST")
    print("=" * 60)
    print()
    
    tester = FPSTest()
    
    # Run tests
    print("Running 10-second tests for each method...")
    print("=" * 60)
    
    tester.test_compressed_images(10)
    tester.test_uncompressed_images(10)
    tester.test_parallel_capture(10)
    tester.test_lower_resolution(10)
    
    # Summary
    print("\n" + "=" * 60)
    print("PERFORMANCE SUMMARY")
    print("=" * 60)
    
    best_method = max(tester.method_stats, key=tester.method_stats.get)
    
    for method, fps in sorted(tester.method_stats.items(), key=lambda x: x[1], reverse=True):
        marker = "👑" if method == best_method else "  "
        print(f"{marker} {method:15s}: {fps:6.2f} FPS")
    
    print("\n" + "=" * 60)
    print(f"Best method: {best_method} ({tester.method_stats[best_method]:.2f} FPS)")
    print("=" * 60)
    
    # Live test
    print("\nWould you like to run a live FPS test? (y/n): ", end="")
    if input().lower() == 'y':
        tester.run_live_fps_test()

if __name__ == "__main__":
    main()