#!/usr/bin/env python3
"""
Version bumping script that reads current version from source files.
This allows manual version edits to be preserved and incremented correctly.
"""

import re
import subprocess
import sys
from pathlib import Path

def get_current_version():
    """Read current version from __init__.py"""
    init_file = Path("basicmicro/__init__.py")
    content = init_file.read_text()
    match = re.search(r'__version__ = ["\']([^"\']+)["\']', content)
    if not match:
        raise ValueError("Could not find version in __init__.py")
    return match.group(1)

def get_setup_version():
    """Read current version from setup.py"""
    setup_file = Path("setup.py")
    content = setup_file.read_text()
    match = re.search(r'version="([^"]+)"', content)
    if not match:
        raise ValueError("Could not find version in setup.py")
    return match.group(1)

def sync_versions():
    """Ensure setup.py version matches __init__.py version"""
    init_version = get_current_version()
    try:
        setup_version = get_setup_version()
        
        if init_version != setup_version:
            print(f"Version mismatch detected:")
            print(f"  __init__.py: {init_version}")
            print(f"  setup.py:    {setup_version}")
            print(f"Syncing setup.py to match __init__.py...")
            
            # Update setup.py to match __init__.py
            setup_file = Path("setup.py")
            content = setup_file.read_text()
            content = re.sub(
                r'version="[^"]+"',
                f'version="{init_version}"',
                content
            )
            setup_file.write_text(content)
            print(f"✅ Updated setup.py version to {init_version}")
            return True
        else:
            print(f"✅ Versions already in sync: {init_version}")
            return False
    except ValueError as e:
        print(f"Warning: {e}")
        return False

def bump_version(current_version, bump_type):
    """Bump version based on type (major, minor, patch)"""
    parts = list(map(int, current_version.split('.')))
    
    if bump_type == "major":
        parts[0] += 1
        parts[1] = 0
        parts[2] = 0
    elif bump_type == "minor":
        parts[1] += 1
        parts[2] = 0
    elif bump_type == "patch":
        parts[2] += 1
    else:
        raise ValueError(f"Invalid bump type: {bump_type}")
    
    return f"{parts[0]}.{parts[1]}.{parts[2]}"

def update_version_files(new_version):
    """Update version in both __init__.py and setup.py"""
    # Update __init__.py
    init_file = Path("basicmicro/__init__.py")
    content = init_file.read_text()
    content = re.sub(
        r'__version__ = ["\'][^"\']+["\']',
        f'__version__ = "{new_version}"',
        content
    )
    init_file.write_text(content)
    
    # Update setup.py
    setup_file = Path("setup.py")
    content = setup_file.read_text()
    content = re.sub(
        r'version="[^"]+",',
        f'version="{new_version}",',
        content
    )
    setup_file.write_text(content)
    
    print(f"Updated version to {new_version} in:")
    print(f"  - {init_file}")
    print(f"  - {setup_file}")

def get_commits_since_last_tag():
    """Get commits since last tag to determine bump type"""
    try:
        # Get last tag
        result = subprocess.run(
            ["git", "describe", "--tags", "--abbrev=0"],
            capture_output=True, text=True, check=True
        )
        last_tag = result.stdout.strip()
        
        # Get commits since last tag
        result = subprocess.run(
            ["git", "log", f"{last_tag}..HEAD", "--oneline"],
            capture_output=True, text=True, check=True
        )
        commits = result.stdout.strip().split('\n') if result.stdout.strip() else []
        
        return commits
    except subprocess.CalledProcessError:
        # No tags exist yet
        result = subprocess.run(
            ["git", "log", "--oneline"],
            capture_output=True, text=True, check=True
        )
        return result.stdout.strip().split('\n') if result.stdout.strip() else []

def determine_bump_type(commits):
    """Determine bump type based on conventional commit messages"""
    has_breaking = any("!" in commit for commit in commits)
    has_feat = any("feat:" in commit or "feat(" in commit for commit in commits)
    has_fix_or_perf = any(
        any(keyword in commit for keyword in ["fix:", "fix(", "perf:", "perf("]) 
        for commit in commits
    )
    
    if has_breaking:
        return "major"
    elif has_feat:
        return "minor"
    elif has_fix_or_perf:
        return "patch"
    else:
        return None  # No version bump needed

def main():
    # First, always sync versions to ensure setup.py matches __init__.py
    was_synced = sync_versions()
    
    if len(sys.argv) > 1:
        bump_type = sys.argv[1]
        if bump_type not in ["major", "minor", "patch", "sync"]:
            print("Usage: python bump_version.py [major|minor|patch|sync]")
            sys.exit(1)
        
        # If only syncing, exit after sync
        if bump_type == "sync":
            if was_synced:
                print("Version sync completed successfully")
            else:
                print("Versions were already in sync")
            return
    else:
        # Auto-determine bump type from commits
        commits = get_commits_since_last_tag()
        bump_type = determine_bump_type(commits)
        if not bump_type:
            if was_synced:
                print("Version sync completed, but no version bump needed (no feat/fix/perf commits found)")
            else:
                print("No version bump needed (no feat/fix/perf commits found)")
            return
    
    current_version = get_current_version()
    new_version = bump_version(current_version, bump_type)
    
    print(f"Bumping version from {current_version} to {new_version} ({bump_type})")
    update_version_files(new_version)
    
    # Create git tag
    tag_name = f"v{new_version}"
    subprocess.run(["git", "add", "basicmicro/__init__.py", "setup.py"], check=True)
    subprocess.run(["git", "commit", "-m", f"chore(release): {new_version}"], check=True)
    subprocess.run(["git", "tag", tag_name], check=True)
    
    print(f"Created tag: {tag_name}")
    print(f"Version successfully bumped to {new_version}")

if __name__ == "__main__":
    main()