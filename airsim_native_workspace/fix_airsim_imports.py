#!/usr/bin/env python3
"""
Fix all AirSim imports to use cosysairsim with fallback
"""

import os
import re

def fix_import_in_file(filepath):
    """Fix import airsim to use cosysairsim with fallback"""
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Pattern to match import airsim at the beginning of files
    pattern = r'^import airsim\s*$'
    replacement = '''import airsim'''
    
    # Check if file has simple import airsim
    if re.search(pattern, content, re.MULTILINE):
        # Replace the import
        new_content = re.sub(pattern, replacement, content, count=1, flags=re.MULTILINE)
        
        with open(filepath, 'w') as f:
            f.write(new_content)
        
        print(f"✅ Fixed: {filepath}")
        return True
    else:
        # Check if already fixed
        if 'cosysairsim' in content:
            print(f"✓ Already fixed: {filepath}")
        else:
            print(f"⊘ No simple import found: {filepath}")
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
        if fix_import_in_file(filepath):
            fixed_count += 1
    
    print("=" * 60)
    print(f"Fixed {fixed_count} files")

if __name__ == "__main__":
    main()