#!/bin/bash

# Usage: ./setup_venv.sh

# Define color codes
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Change to that directory
cd "$SCRIPT_DIR" || exit

echo "Changed directory to: $SCRIPT_DIR"

# Create a virtual environment named 'venv'
if [ ! -d "venv" ]; then
    echo "Creating virtual environment ..."
    python3 -m venv venv
    source venv/bin/activate

    echo "Installing dependencies from requirements.txt ..."
    pip install -r requirements.txt

    echo "Installing local custom package from setup.py ..."
    pip install -e .

    deactivate
    echo "Virtual environment created and all packages installed"
else
    echo -e "${YELLOW}Virtual environment already exists. No changes made${NC}"
fi
