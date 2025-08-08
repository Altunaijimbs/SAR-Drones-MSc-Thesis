#!/usr/bin/env python3
"""
Revert all AirSim imports back to regular airsim
"""

import os
import re

def revert_import_in_file(filepath):
    """Revert cosysairsim imports back to regular airsim"""
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Pattern to match the try/except import block
    pattern = r'try:\s*\n\s*import cosysairsim as airsim\s*\nexcept ImportError:\s*\n\s*import airsim'
    replacement = 'import airsim'
    
    # Check if file has the try/except pattern
    if re.search(pattern, content):
        # Replace with simple import
        new_content = re.sub(pattern, replacement, content)
        
        with open(filepath, 'w') as f:
            f.write(new_content)
        
        print(f"✅ Reverted: {filepath}")
        return True
    else:
        print(f"⊘ No cosysairsim import found: {filepath}")
        return False

def main():
    # Find all Python files
    workspace_dir = '/home/mbs/SAR-Drones-MSc-Thesis/airsim_native_workspace'
    
    python_files = []
    for root, dirs, files in os.walk(workspace_dir):
        for file in files:
            if file.endswith('.py'):
                python_files.append(os.path.join(root, file))
    
    print(f"Found {len(python_files)} Python files")
    print("=" * 60)
    
    fixed_count = 0
    for filepath in python_files:
        if revert_import_in_file(filepath):
            fixed_count += 1
    
    print("=" * 60)
    print(f"Reverted {fixed_count} files")

if __name__ == "__main__":
    main()