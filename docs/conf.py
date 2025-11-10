# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
from datetime import datetime

# Add the project root to the Python path so Sphinx can find our modules
sys.path.insert(0, os.path.abspath('..'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'URC 2026 - Mars Rover Autonomy System'
copyright = f'{datetime.now().year}, University Rover Challenge Team'
author = 'URC 2026 Team'
release = '1.0.0'
version = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    # Python documentation
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.napoleon',

    # General documentation features
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',

    # Markdown support
    'myst_parser',

    # Diagram and visualization extensions
    'sphinx.ext.graphviz',
    'sphinx.ext.inheritance_diagram',

    # C++ documentation with Doxygen
    'breathe',

    # Note: Additional extensions require separate installation
    # 'sphinx_js',        # For JavaScript/TypeScript docs
    # 'autoapi.extension', # For auto-generated API docs
    # 'sphinx.ext.coverage',
]

# Note: AutoAPI, Breathe, and sphinx-js require additional packages
# AutoAPI configuration for multi-language support (requires autoapi package)
# autoapi_dirs = [
#     '../Autonomy',  # Python code
#     '../frontend/src',  # JavaScript/TypeScript
# ]
# autoapi_type = 'python'
# autoapi_template_dir = '_templates/autoapi'
# autoapi_generate_api_docs = True
# autoapi_add_toctree_entry = True
# autoapi_options = [
#     'members',
#     'undoc-members',
#     'show-inheritance',
#     'show-module-summary',
#     'special-members',
#     'imported-members',
# ]

# Breathe configuration for C++ documentation (requires breathe package)
breathe_projects = {
    "autonomy": "../Autonomy/docs/doxygen/xml",
}
breathe_default_project = "autonomy"
breathe_domain_by_extension = {
    "h": "cpp",
    "hpp": "cpp",
}
breathe_default_members = ('members', 'undoc-members', 'protected-members', 'private-members')

# sphinx-js configuration for JavaScript/TypeScript (requires sphinx-js package)
# js_source_path = '../frontend/src'
# jsdoc_config_path = 'jsdoc.json'

# MyST Parser configuration for Markdown
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "html_admonition",
    "html_image",
    "replacements",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist",
]

# Napoleon settings for Google/NumPy style docstrings
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = False
napoleon_use_admonition_for_notes = False
napoleon_use_admonition_for_references = False
napoleon_use_ivar = False
napoleon_use_param = True
napoleon_use_rtype = True
napoleon_type_aliases = None

# Autodoc configuration
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}
autodoc_typehints = 'description'
autodoc_typehints_description_target = 'all'

# Intersphinx mapping for external documentation
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = [
    'css/custom.css',
]

# Theme options
html_theme_options = {
    'canonical_url': '',
    'analytics_id': '',
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    'style_nav_header_background': '#2980B9',
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# The master toctree document.
master_doc = 'index'
root_doc = 'index'

# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    'papersize': 'letterpaper',
    'pointsize': '10pt',
    'preamble': '',
    'figure_align': 'htbp',
}

latex_documents = [
    (master_doc, 'URC2026.tex', 'URC 2026 Documentation',
     'URC 2026 Team', 'manual'),
]

# -- Options for manual page output ------------------------------------------

man_pages = [
    (master_doc, 'urc2026', 'URC 2026 Documentation',
     [author], 1)
]

# -- Options for Texinfo output ----------------------------------------------

texinfo_documents = [
    (master_doc, 'URC2026', 'URC 2026 Documentation',
     author, 'URC2026', 'One line description of project.',
     'Miscellaneous'),
]

# -- Extension configuration --------------------------------------------------

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
source_suffix = {
    '.rst': None,
    '.md': None,
}

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = [
    '_build',
    'Thumbs.db',
    '.DS_Store',
    '_static',
    '_templates',
]

# -- Custom setup -------------------------------------------------------------

def setup(app):
    """Custom setup function for additional configurations."""
    # Add custom CSS
    app.add_css_file('css/custom.css')

    # Add custom JavaScript if needed
    # app.add_js_file('js/custom.js')

    return {
        'version': '1.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
