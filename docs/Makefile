# Makefile for Sphinx documentation

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build
DOXYGEN       = doxygen
JSDOC         = jsdoc

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help clean html dirhtml singlehtml pickle json htmlhelp qthelp devhelp epub latex latexpdf text man changes linkcheck doctest coverage gettext

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Custom targets for multi-language documentation

# Build all documentation (Sphinx + Doxygen + JSDoc)
all: doxygen jsdoc html
	@echo "All documentation built successfully!"

# Build Doxygen documentation for C++ code
doxygen:
	@echo "Building Doxygen documentation..."
	@mkdir -p $(BUILDDIR)/doxygen
	@$(DOXYGEN) Doxyfile
	@echo "Doxygen documentation built in $(BUILDDIR)/doxygen/"

# Build JSDoc documentation for JavaScript/TypeScript
jsdoc:
	@echo "Building JSDoc documentation..."
	@mkdir -p $(BUILDDIR)/jsdoc
	@if command -v jsdoc >/dev/null 2>&1; then \
		$(JSDOC) -c jsdoc.json; \
		echo "JSDoc documentation built in $(BUILDDIR)/jsdoc/"; \
	else \
		echo "JSDoc not found. Install with: npm install -g jsdoc"; \
		echo "Skipping JSDoc documentation build."; \
	fi

# Clean all documentation builds
clean-all: clean clean-doxygen clean-jsdoc
	@echo "All documentation cleaned."

# Clean Doxygen documentation
clean-doxygen:
	@echo "Cleaning Doxygen documentation..."
	@rm -rf $(BUILDDIR)/doxygen
	@echo "Doxygen documentation cleaned."

# Clean JSDoc documentation
clean-jsdoc:
	@echo "Cleaning JSDoc documentation..."
	@rm -rf $(BUILDDIR)/jsdoc
	@echo "JSDoc documentation cleaned."

# Install documentation dependencies
install-deps:
	@echo "Installing documentation dependencies..."
	@pip install -r requirements.txt
	@if command -v npm >/dev/null 2>&1; then \
		npm install -g jsdoc; \
	else \
		echo "npm not found. Please install Node.js to build JSDoc documentation."; \
	fi
	@echo "Documentation dependencies installed."

# Serve documentation locally for development
serve: html
	@echo "Starting local documentation server..."
	@cd $(BUILDDIR)/html && python3 -m http.server 8000

# Check for broken links
linkcheck-all: linkcheck
	@echo "Link check completed."

# Generate API documentation only
api-docs: doxygen jsdoc
	@echo "API documentation generated."

# Validate documentation
validate: doctest linkcheck
	@echo "Documentation validation completed."

# Development build with auto-reload (requires sphinx-autobuild)
dev:
	@echo "Starting development server with auto-reload..."
	@if command -v sphinx-autobuild >/dev/null 2>&1; then \
		sphinx-autobuild "$(SOURCEDIR)" "$(BUILDDIR)/html" $(SPHINXOPTS) --open-browser --port 8001; \
	else \
		echo "sphinx-autobuild not found. Install with: pip install sphinx-autobuild"; \
		make serve; \
	fi

# Build and deploy to GitHub Pages
github-pages: clean-all all
	@echo "Building for GitHub Pages..."
	@touch $(BUILDDIR)/html/.nojekyll
	@echo "GitHub Pages build completed."

# Generate PDF documentation (requires LaTeX)
pdf: latexpdf
	@echo "PDF documentation generated: $(BUILDDIR)/latex/URC2026.pdf"

# Show documentation statistics
stats:
	@echo "Documentation Statistics:"
	@echo "========================"
	@echo "Source files: $$(find . -name "*.rst" -o -name "*.md" | wc -l)"
	@echo "Python modules documented: $$(find ../Autonomy -name "*.py" | wc -l)"
	@echo "C++ files documented: $$(find ../Autonomy/code -name "*.cpp" -o -name "*.hpp" -o -name "*.h" | wc -l)"
	@echo "JS/TS files documented: $$(find ../frontend/src -name "*.js" -o -name "*.jsx" -o -name "*.ts" -o -name "*.tsx" | wc -l)"
	@echo "Total documentation size: $$(du -sh $(BUILDDIR) 2>/dev/null | cut -f1 || echo 'Not built yet')"

# Check documentation coverage
coverage-check: coverage
	@echo "Documentation coverage report generated."

# CI/CD build target
ci-build: install-deps clean-all all validate
	@echo "CI/CD build completed successfully!"

.PHONY: all doxygen jsdoc clean-all clean-doxygen clean-jsdoc install-deps serve linkcheck-all api-docs validate dev github-pages pdf stats coverage-check ci-build
