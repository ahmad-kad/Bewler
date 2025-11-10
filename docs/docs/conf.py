<<<<<<< Current (Your changes)
=======
# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

# Add project root to path for autodoc
sys.path.insert(0, os.path.abspath('../..'))
sys.path.insert(0, os.path.abspath('../../Autonomy/code/led_status'))
sys.path.insert(0, os.path.abspath('../../Autonomy/code/navigation'))
sys.path.insert(0, os.path.abspath('../../Autonomy/code/state_management'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'URC Machiato 2026'
copyright = '2025, Autonomy Team'
author = 'Autonomy Team'
release = '1.0'
version = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    # Core Sphinx extensions
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.imgmath',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinx.ext.napoleon',  # Google/NumPy style docstrings

    # Multi-language support
    'breathe',              # C++ documentation via Doxygen
    'exhale',               # C++ API documentation
    'sphinx_js',            # JavaScript/TypeScript documentation
    'myst_parser',          # Markdown support
    'sphinx_tabs.tabs',     # Tabbed content

    # Additional utilities
    'sphinx.ext.graphviz',  # Graphviz diagrams
]

# MyST Parser settings for Markdown integration
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "dollarmath",
    "fieldlist",
    "html_admonition",
    "html_image",
    "linkify",
    "replacements",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist",
]

myst_heading_anchors = 3

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
    'member-order': 'bysource',
}

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
napoleon_preprocess_types = False
napoleon_type_aliases = None
napoleon_attr_annotations = True

# Breathe settings for C++ documentation
breathe_projects = {
    "URC_Machiato": "../../doxygen/xml"
}
breathe_default_project = "URC_Machiato"

# Exhale settings for C++ API docs
exhale_args = {
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "C++ API Reference",
    "doxygenStripFromPath": "../..",
    "createTreeView": True,
    "exhaleExecutesDoxygen": True,
    "exhaleDoxygenStdin": """
    INPUT = ../../Autonomy/code/autonomy_interfaces \
            ../../Autonomy/code/autonomous_typing \
            ../../Autonomy/code/computer_vision \
            ../../Autonomy/code/led_status \
            ../../Autonomy/code/navigation \
            ../../Autonomy/code/simulation \
            ../../Autonomy/code/slam \
            ../../Autonomy/code/state_management
    RECURSIVE = YES
    EXCLUDE_PATTERNS = */test_* */__pycache__/* */_build/*
    GENERATE_HTML = NO
    GENERATE_LATEX = NO
    GENERATE_XML = YES
    XML_OUTPUT = xml
    """
}

# Sphinx-js settings for JavaScript/TypeScript
js_source_path = '../../frontend/src'
jsdoc_config_path = '../../frontend/jsdoc.json'

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# Source file suffixes
source_suffix = {
    '.rst': None,
    '.md': 'myst_parser.sphinx_',
}

language = 'en'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Theme options
html_theme_options = {
    'canonical_url': '',
    'analytics_id': '',
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    'style_nav_header_background': '#2980B9',
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# Custom CSS
html_css_files = [
    'custom.css',
]

# -- Options for intersphinx extension ---------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/intersphinx.html#configuration

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'pandas': ('https://pandas.pydata.org/docs/', None),
    'matplotlib': ('https://matplotlib.org/stable/', None),
    'ros2': ('https://docs.ros2.org/latest/', None),
    'opencv': ('https://docs.opencv.org/4.8.0/', None),
}

# -- Options for todo extension ----------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/todo.html#configuration

todo_include_todos = True

# -- Options for autodoc -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html

autoclass_content = 'both'
autodoc_member_order = 'bysource'
autodoc_default_flags = ['members', 'undoc-members', 'show-inheritance']

# -- Graphviz settings -------------------------------------------------------
graphviz_output_format = 'svg'
>>>>>>> Incoming (Background Agent changes)
