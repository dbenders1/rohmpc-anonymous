#!/bin/bash

# Usage: ./setup_venv.sh [/path/to/acados_template]

# Define color codes
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Optional argument: path to acados_template
ACADOS_TEMPLATE_PATH="$1"

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

    # Handle acados_template installation
    if [ -n "$ACADOS_TEMPLATE_PATH" ]; then
        if [ -d "$ACADOS_TEMPLATE_PATH" ]; then
            echo "Installing acados_template from: $ACADOS_TEMPLATE_PATH"
            pip install -e "$ACADOS_TEMPLATE_PATH"
        else
            echo -e "${YELLOW}Provided path does not exist or is not a directory: $ACADOS_TEMPLATE_PATH${NC}"
        fi
    else
        echo -e "${YELLOW}No path to acados_template provided. Some functionality may not work correctly${NC}"
    fi

    deactivate
    echo "Virtual environment created and all packages installed"
else
    echo -e "${YELLOW}Virtual environment already exists. No changes made${NC}"
fi
