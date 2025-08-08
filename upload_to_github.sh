#!/bin/bash
# Script to upload project to GitHub

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     GITHUB UPLOAD SCRIPT                                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colors
G='\033[0;32m'
Y='\033[1;33m'
R='\033[0;31m'
NC='\033[0m'

# Check git status
echo -e "${Y}Current git status:${NC}"
git status --short
echo ""

# Count changes
CHANGES=$(git status --porcelain | wc -l)
echo -e "${Y}Total changes to commit: $CHANGES files${NC}"
echo ""

# Show large files that might cause issues
echo -e "${Y}Checking for large files (>50MB):${NC}"
find . -type f -size +50M -not -path "./.git/*" 2>/dev/null | while read file; do
    size=$(du -h "$file" | cut -f1)
    echo -e "${R}  Warning: $file ($size)${NC}"
done
echo ""

# Prompt for action
echo "Options:"
echo "  1) Add ALL files and commit"
echo "  2) Add only code files (*.py, *.sh, *.md, *.txt)"
echo "  3) Interactive add (choose files)"
echo "  4) View detailed status first"
echo "  5) Exit"
echo ""
echo -n "Choose option [1-5]: "
read choice

case $choice in
    1)
        echo -e "${G}Adding all files...${NC}"
        git add .
        ;;
    2)
        echo -e "${G}Adding only code files...${NC}"
        git add "*.py" "*.sh" "*.md" "*.txt" "*.launch.py" "*.xml" "*.yaml" 2>/dev/null
        git add ros2_ws/src/ 2>/dev/null
        git add airsim_native_workspace/ 2>/dev/null
        ;;
    3)
        echo -e "${G}Opening interactive add...${NC}"
        git add -i
        ;;
    4)
        git status
        echo ""
        echo "Run this script again after reviewing"
        exit 0
        ;;
    5)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo -e "${R}Invalid choice${NC}"
        exit 1
        ;;
esac

# Show what will be committed
echo ""
echo -e "${Y}Files staged for commit:${NC}"
git diff --cached --name-status
echo ""

# Get commit message
echo -e "${Y}Enter commit message:${NC}"
echo "(Describe what changes you made)"
read -p "> " commit_msg

if [ -z "$commit_msg" ]; then
    # Generate automatic commit message
    commit_msg="Update SAR drone system - $(date +%Y-%m-%d)"
    echo -e "${Y}Using auto-generated message: $commit_msg${NC}"
fi

# Commit
echo ""
echo -e "${G}Committing changes...${NC}"
git commit -m "$commit_msg"

# Ask about push
echo ""
echo -e "${Y}Ready to push to GitHub${NC}"
echo "Repository: https://github.com/Altunaijimbs/SAR-Drones-MSc-Thesis.git"
echo ""
echo -n "Push to GitHub? (y/n): "
read push_choice

if [[ "$push_choice" == "y" || "$push_choice" == "Y" ]]; then
    echo -e "${G}Pushing to GitHub...${NC}"
    git push origin master
    
    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${G}✅ Successfully pushed to GitHub!${NC}"
        echo ""
        echo "View your repository at:"
        echo "https://github.com/Altunaijimbs/SAR-Drones-MSc-Thesis"
    else
        echo ""
        echo -e "${R}Push failed. You may need to:${NC}"
        echo "  1. Set up GitHub credentials:"
        echo "     git config --global user.name 'Your Name'"
        echo "     git config --global user.email 'your.email@example.com'"
        echo ""
        echo "  2. Or pull first if remote has changes:"
        echo "     git pull origin master --rebase"
        echo ""
        echo "  3. Or force push (careful!):"
        echo "     git push origin master --force"
    fi
else
    echo ""
    echo "Changes committed locally but not pushed."
    echo "To push later, run:"
    echo "  git push origin master"
fi

echo ""
echo "Done!"