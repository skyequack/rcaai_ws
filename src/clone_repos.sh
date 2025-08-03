#!/bin/bash

# Script to clone all GraspNet simulation stack repositories
# This script clones all packages for the UR5 GraspNet simulation stack

# List of repository names to clone
REPOS=(
    "graspnet_hardware"
    "graspnet_description" 
    "graspnet_gazebo"
    "graspnet_moveit"
    "graspnet_control"
    "graspnet_perception"
    "graspnet_bringup"
    # Add more repo names here if needed
)

# Base GitHub SSH URL - update this to match your GitHub username
BASE_URL="git@github.com:skyequack"

# Alternative HTTPS URL (uncomment if you prefer HTTPS over SSH)
# BASE_URL="https://github.com/skyequack"

# Directory to clone repos into (current directory)
CLONE_DIR="$(pwd)"

echo "Starting to clone GraspNet simulation stack repositories..."
echo "Clone directory: $CLONE_DIR"
echo "Base URL: $BASE_URL"
echo ""

# Function to check if git is available
check_git() {
    if ! command -v git &> /dev/null; then
        echo "Error: git is not installed or not in PATH"
        exit 1
    fi
}

# Check git availability
check_git

# Clone repositories
successful_clones=0
skipped_repos=0
failed_clones=0

for repo in "${REPOS[@]}"; do
    echo "Processing repository: $repo"
    
    if [ -d "$CLONE_DIR/$repo" ]; then
        echo "  ✓ Repository '$repo' already exists. Skipping."
        ((skipped_repos++))
    else
        echo "  → Cloning $repo..."
        if git clone "$BASE_URL/$repo.git" "$CLONE_DIR/$repo"; then
            echo "  ✓ Successfully cloned $repo"
            ((successful_clones++))
        else
            echo "  ✗ Failed to clone $repo"
            ((failed_clones++))
        fi
    fi
    echo ""
done

# Summary
echo "=== Clone Summary ==="
echo "Successfully cloned: $successful_clones repositories"
echo "Skipped (already exist): $skipped_repos repositories"
echo "Failed to clone: $failed_clones repositories"
echo "Total repositories processed: ${#REPOS[@]}"

if [ $failed_clones -gt 0 ]; then
    echo ""
    echo "⚠️  Some repositories failed to clone. Please check:"
    echo "   - Your internet connection"
    echo "   - Repository names and URLs are correct"
    echo "   - You have proper access permissions (for SSH)"
    echo "   - Consider using HTTPS instead of SSH if SSH keys aren't set up"
fi

echo ""
echo "Done! You can now build the workspace with:"
echo "cd /home/omer/rcaai_ws && colcon build"