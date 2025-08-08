#!/usr/bin/env python3
"""
Fix the Euler angles function name change from airsim to cosysairsim
"""

import os
import re

def fix_euler_in_file(filepath):
    """Fix to_eularian_angles to quaternion_to_euler_angles"""
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Check if file has the old function name
    if 'to_eularian_angles' in content:
        # Replace airsim.quaternion_to_euler_angles with airsim.quaternion_to_euler_angles
        new_content = content.replace('airsim.quaternion_to_euler_angles', 'airsim.quaternion_to_euler_angles')
        
        with open(filepath, 'w') as f:
            f.write(new_content)
        
        print(f"✅ Fixed: {os.path.basename(filepath)}")
        return True
    else:
        return False

def main():
    workspace_dir = '/home/mbs/SAR-Drones-MSc-Thesis/airsim_native_workspace'
    
    python_files = []
    for root, dirs, files in os.walk(workspace_dir):
        for file in files:
            if file.endswith('.py'):
                python_files.append(os.path.join(root, file))
    
    print("Fixing Euler angle function names...")
    print("=" * 60)
    
    fixed = 0
    for filepath in python_files:
        if fix_euler_in_file(filepath):
            fixed += 1
    
    print("=" * 60)
    print(f"✅ Fixed {fixed} files")

if __name__ == "__main__":
    main()