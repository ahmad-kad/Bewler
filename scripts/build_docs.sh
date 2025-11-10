#!/bin/bash
# Build script for URC Machiato 2026 documentation

set -e  # Exit on any error

echo "🚀 Building URC Machiato 2026 Documentation"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the project root
if [ ! -d "docs/docs" ]; then
    print_error "docs/docs directory not found. Please run from project root."
    exit 1
fi

cd docs/docs

print_status "Converting markdown documentation to RST..."
if python3 ../../../scripts/convert_md_to_rst.py; then
    print_success "Markdown conversion completed"
else
    print_error "Markdown conversion failed"
    exit 1
fi

print_status "Building HTML documentation..."

# Build with Sphinx
if make html; then
    print_success "HTML documentation built successfully"
else
    print_error "HTML documentation build failed"
    exit 1
fi

print_status "Building PDF documentation..."
if make latexpdf; then
    print_success "PDF documentation built successfully"
else
    print_warning "PDF documentation build failed (LaTeX may not be installed)"
fi

print_status "Running documentation tests..."
if make linkcheck; then
    print_success "Link check passed"
else
    print_warning "Some links may be broken"
fi

print_success "Documentation build complete!"
echo ""
echo "📖 Documentation available at:"
echo "   HTML: docs/docs/_build/html/index.html"
echo "   PDF:  docs/docs/_build/latex/urcmachiato2026.pdf (if built)"
echo ""
echo "🌐 To serve locally: cd docs/docs/_build/html && python3 -m http.server 8000"
