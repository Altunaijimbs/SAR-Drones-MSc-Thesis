#!/usr/bin/env python3
"""
Update all scripts to use cosysairsim (which works with camera!)
"""

import os
import re

def update_file(filepath):
    """Update imports to use cosysairsim"""
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Replace import airsim with import cosysairsim as airsim
    pattern = r'^import airsim\s*$'
    replacement = 'import cosysairsim as airsim'
    
    if re.search(pattern, content, re.MULTILINE):
        new_content = re.sub(pattern, replacement, content, flags=re.MULTILINE)
        
        with open(filepath, 'w') as f:
            f.write(new_content)
        
        print(f"✅ Updated: {filepath}")
        return True
    else:
        if 'cosysairsim' in content:
            print(f"✓ Already using cosysairsim: {filepath}")
        else:
            print(f"⊘ No airsim import: {filepath}")
        return False

def main():
    workspace_dir = '/home/mbs/SAR-Drones-MSc-Thesis/airsim_native_workspace'
    
    python_files = []
    for root, dirs, files in os.walk(workspace_dir):
        for file in files:
            if file.endswith('.py'):
                python_files.append(os.path.join(root, file))
    
    print(f"Updating {len(python_files)} Python files to use cosysairsim...")
    print("=" * 60)
    
    updated = 0
    for filepath in python_files:
        if update_file(filepath):
            updated += 1
    
    print("=" * 60)
    print(f"✅ Updated {updated} files to use cosysairsim")
    print("\nThe camera should now work in all scripts!")

if __name__ == "__main__":
    main()