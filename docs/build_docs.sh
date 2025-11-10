#!/bin/bash
# URC Machiato 2026 Documentation Build Script
# =============================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DOCS_DIR="$SCRIPT_DIR"
BUILD_DIR="$DOCS_DIR/_build"
DOXYGEN_XML_DIR="$PROJECT_ROOT/Autonomy/docs/doxygen/xml"
JSDOC_DIR="$DOCS_DIR/_build/jsdoc"

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check system requirements
check_requirements() {
    log_info "Checking system requirements..."

    # Check Python
    if ! command_exists python3; then
        log_error "Python 3 is required but not found."
        exit 1
    fi

    # Check pip
    if ! command_exists pip3; then
        log_error "pip3 is required but not found."
        exit 1
    fi

    # Check sphinx-build
    if ! command_exists sphinx-build; then
        log_warning "sphinx-build not found. Installing Sphinx..."
        pip3 install sphinx
    fi

    # Check optional tools
    if ! command_exists doxygen; then
        log_warning "Doxygen not found. C++ documentation will be skipped."
        log_warning "Install doxygen to enable C++ API documentation."
    else
        log_info "Doxygen found - C++ API documentation will be generated."
    fi

    # Check for graphviz (required for diagrams)
    if ! command_exists dot; then
        log_warning "Graphviz (dot) not found. Diagrams will not be generated."
        log_warning "Install graphviz to enable diagram generation."
    else
        log_info "Graphviz found - diagrams will be generated."
    fi

    if ! command_exists jsdoc; then
        log_warning "JSDoc not found. JavaScript documentation will be skipped."
        log_warning "Install jsdoc globally: npm install -g jsdoc"
    fi

    log_success "Requirements check complete."
}

# Install Python dependencies
install_dependencies() {
    log_info "Installing documentation dependencies..."

    if [ -f "$PROJECT_ROOT/pyproject.toml" ]; then
        cd "$PROJECT_ROOT"
        pip3 install -e .[docs]
        cd "$DOCS_DIR"
    else
        log_warning "pyproject.toml not found. Installing basic Sphinx packages..."
        pip3 install sphinx sphinx-rtd-theme breathe sphinx-js myst-parser sphinx-copybutton
    fi

    log_success "Dependencies installed."
}

# Generate Doxygen documentation
generate_doxygen() {
    if command_exists doxygen; then
        log_info "Generating Doxygen documentation for C++ code..."

        cd "$PROJECT_ROOT/Autonomy/docs"
        if [ -f "Doxyfile" ]; then
            doxygen Doxyfile
            log_success "Doxygen XML generated in $DOXYGEN_XML_DIR"
        else
            log_warning "Doxyfile not found. Skipping Doxygen generation."
        fi
        cd "$DOCS_DIR"
    else
        log_warning "Doxygen not available. Skipping C++ documentation generation."
    fi
}

# Generate JSDoc documentation
generate_jsdoc() {
    if command_exists jsdoc; then
        log_info "Generating JSDoc documentation for JavaScript/TypeScript..."

        cd "$PROJECT_ROOT/frontend"
        if [ -f "jsdoc.json" ]; then
            jsdoc -c jsdoc.json
            log_success "JSDoc generated in $JSDOC_DIR"
        else
            log_warning "jsdoc.json not found. Skipping JSDoc generation."
        fi
        cd "$DOCS_DIR"
    else
        log_warning "JSDoc not available. Skipping JavaScript documentation generation."
    fi
}

# Build Sphinx documentation
build_sphinx() {
    local format="$1"
    log_info "Building $format documentation with Sphinx..."

    if [ "$format" = "html" ]; then
        sphinx-build -b html "$DOCS_DIR" "$BUILD_DIR/html"
        log_success "HTML documentation built in $BUILD_DIR/html"
    elif [ "$format" = "latex" ]; then
        sphinx-build -b latex "$DOCS_DIR" "$BUILD_DIR/latex"
        log_success "LaTeX documentation built in $BUILD_DIR/latex"
    elif [ "$format" = "pdf" ]; then
        sphinx-build -b latex "$DOCS_DIR" "$BUILD_DIR/latex"
        cd "$BUILD_DIR/latex"
        make
        log_success "PDF documentation built in $BUILD_DIR/latex"
        cd "$DOCS_DIR"
    else
        log_error "Unknown format: $format"
        exit 1
    fi
}

# Clean build artifacts
clean_build() {
    log_info "Cleaning build artifacts..."
    rm -rf "$BUILD_DIR"/*
    log_success "Build artifacts cleaned."
}

# Serve documentation locally
serve_docs() {
    if [ -d "$BUILD_DIR/html" ]; then
        log_info "Serving documentation on http://localhost:8000"
        log_info "Press Ctrl+C to stop"
        cd "$BUILD_DIR/html"
        python3 -m http.server 8000
    else
        log_error "HTML documentation not found. Run '$0 html' first."
        exit 1
    fi
}

# Show usage information
show_usage() {
    cat << EOF
URC Machiato 2026 Documentation Build Script

USAGE:
    $0 [COMMAND] [OPTIONS]

COMMANDS:
    check       Check system requirements
    install     Install documentation dependencies
    doxygen     Generate Doxygen XML for C++ code
    jsdoc       Generate JSDoc for JavaScript/TypeScript
    html        Build HTML documentation
    latex       Build LaTeX documentation
    pdf         Build PDF documentation
    all         Build all documentation formats
    clean       Clean build artifacts
    serve       Serve HTML documentation locally
    help        Show this help message

OPTIONS:
    --deps      Install dependencies before building (for html/latex/pdf/all commands)

EXAMPLES:
    $0 check                    # Check requirements
    $0 install                  # Install dependencies
    $0 html                     # Build HTML docs
    $0 html --deps              # Install deps and build HTML
    $0 all                      # Build all formats
    $0 serve                    # Serve docs locally
    $0 clean                    # Clean build artifacts

ENVIRONMENT VARIABLES:
    SPHINXOPTS    Additional options for sphinx-build
    SPHINXBUILD   Path to sphinx-build executable (default: sphinx-build)

EOF
}

# Main script logic
main() {
    local command="$1"
    local install_deps=false

    # Parse options
    case "$2" in
        --deps)
            install_deps=true
            ;;
    esac

    case "$command" in
        check)
            check_requirements
            ;;
        install)
            install_dependencies
            ;;
        doxygen)
            generate_doxygen
            ;;
        jsdoc)
            generate_jsdoc
            ;;
        html)
            if [ "$install_deps" = true ]; then
                install_dependencies
            fi
            generate_doxygen
            generate_jsdoc
            build_sphinx html
            ;;
        latex)
            if [ "$install_deps" = true ]; then
                install_dependencies
            fi
            generate_doxygen
            generate_jsdoc
            build_sphinx latex
            ;;
        pdf)
            if [ "$install_deps" = true ]; then
                install_dependencies
            fi
            generate_doxygen
            generate_jsdoc
            build_sphinx pdf
            ;;
        all)
            if [ "$install_deps" = true ]; then
                install_dependencies
            fi
            generate_doxygen
            generate_jsdoc
            build_sphinx html
            build_sphinx pdf
            ;;
        clean)
            clean_build
            ;;
        serve)
            serve_docs
            ;;
        help|--help|-h)
            show_usage
            ;;
        *)
            log_error "Unknown command: $command"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"
