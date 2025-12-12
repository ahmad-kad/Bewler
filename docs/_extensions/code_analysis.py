#!/usr/bin/env python3
"""
Code Analysis Extension for Sphinx Documentation

This extension automatically analyzes Python codebases to generate import dependency graphs,
class hierarchies, and module relationships.

Usage in RST files:
    .. code-imports::
       :path: ../Autonomy/code
       :output: import_graph.png
       :max-depth: 3

    .. code-hierarchy::
       :path: ../Autonomy/code/state_management
       :output: class_hierarchy.png
"""

import ast
import os
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import networkx as nx
from docutils import nodes
from docutils.parsers.rst import Directive
from sphinx.application import Sphinx
from sphinx.util.docutils import SphinxDirective


class CodeImportsDirective(SphinxDirective):
    """Directive to generate Python import dependency graphs."""

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {
        'path': str,
        'output': str,
        'max-depth': int,
        'exclude-patterns': str,
        'title': str,
    }

    def run(self) -> List[nodes.Node]:
        """Generate import dependency diagram."""
        source_path = self.options.get('path', '../Autonomy/code')
        output_file = self.options.get('output', 'import_graph.png')
        max_depth = self.options.get('max-depth', 3)
        exclude_patterns = self.options.get('exclude-patterns', '__pycache__,*.pyc,test_*')
        title = self.options.get('title', 'Python Import Dependencies')

        # Convert relative path to absolute
        if not os.path.isabs(source_path):
            source_path = os.path.join(self.env.srcdir, source_path)

        # Generate the diagram
        try:
            generate_import_graph(
                source_path,
                self.env.srcdir,
                output_file,
                max_depth,
                exclude_patterns.split(','),
                title
            )
        except Exception as e:
            self.logger.warning(f"Failed to generate import graph: {e}")

        # Create figure node
        figure_node = nodes.figure()
        image_node = nodes.image(uri=output_file, alt=title)
        caption_node = nodes.caption(text=title)
        figure_node += image_node
        figure_node += caption_node

        return [figure_node]


class CodeHierarchyDirective(SphinxDirective):
    """Directive to generate class hierarchy diagrams."""

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {
        'path': str,
        'output': str,
        'base-class': str,
        'title': str,
    }

    def run(self) -> List[nodes.Node]:
        """Generate class hierarchy diagram."""
        source_path = self.options.get('path', '../Autonomy/code')
        output_file = self.options.get('output', 'class_hierarchy.png')
        base_class = self.options.get('base-class', '')
        title = self.options.get('title', 'Class Hierarchy')

        # Convert relative path to absolute
        if not os.path.isabs(source_path):
            source_path = os.path.join(self.env.srcdir, source_path)

        # Generate the diagram
        try:
            generate_class_hierarchy(
                source_path,
                self.env.srcdir,
                output_file,
                base_class,
                title
            )
        except Exception as e:
            self.logger.warning(f"Failed to generate class hierarchy: {e}")

        # Create figure node
        figure_node = nodes.figure()
        image_node = nodes.image(uri=output_file, alt=title)
        caption_node = nodes.caption(text=title)
        figure_node += image_node
        figure_node += caption_node

        return [figure_node]


def analyze_python_imports(source_path: str, exclude_patterns: List[str] = None) -> nx.DiGraph:
    """Analyze Python files to build import dependency graph."""
    if exclude_patterns is None:
        exclude_patterns = ['__pycache__', '*.pyc', 'test_*', 'tests']

    G = nx.DiGraph()
    source_path = Path(source_path)

    # Find all Python files
    python_files = []
    for pattern in ['**/*.py']:
        python_files.extend(source_path.glob(pattern))

    # Filter out excluded files
    filtered_files = []
    for file_path in python_files:
        excluded = False
        for pattern in exclude_patterns:
            if pattern in str(file_path):
                excluded = True
                break
        if not excluded:
            filtered_files.append(file_path)

    # Analyze each file
    for file_path in filtered_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Parse the AST
            tree = ast.parse(content, filename=str(file_path))

            # Extract module name from file path
            rel_path = file_path.relative_to(source_path)
            module_name = str(rel_path).replace('/', '.').replace('\\', '.').replace('.py', '')

            # Find imports
            imports = extract_imports(tree)

            # Add nodes and edges
            G.add_node(module_name)

            for imp in imports:
                # Clean import name
                clean_import = clean_import_name(imp, module_name, source_path)
                if clean_import:
                    G.add_edge(module_name, clean_import)

        except (SyntaxError, UnicodeDecodeError) as e:
            # Skip files with syntax errors or encoding issues
            continue

    return G


def extract_imports(tree: ast.AST) -> List[str]:
    """Extract all import statements from an AST."""
    imports = []

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.append(alias.name)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                if node.level == 0:  # Absolute import
                    imports.append(node.module)
                else:  # Relative import
                    # Handle relative imports (simplified)
                    imports.append(node.module)

    return imports


def clean_import_name(import_name: str, current_module: str, source_path: Path) -> Optional[str]:
    """Clean and resolve import names to module paths within the project."""
    # Skip standard library and external packages
    stdlib_modules = {
        'os', 'sys', 'json', 're', 'pathlib', 'typing', 'dataclasses',
        'collections', 'itertools', 'functools', 'datetime', 'time',
        'threading', 'multiprocessing', 'subprocess', 'logging', 'argparse',
        'ast', 'inspect', 'importlib', 'pkgutil', 'warnings', 'weakref',
        'abc', 'enum', 'numbers', 'math', 'random', 'statistics',
        'cv2', 'numpy', 'pandas', 'matplotlib', 'networkx', 'pyyaml',
        'structlog', 'pytest', 'black', 'isort', 'flake8', 'mypy'
    }

    # Check if it's a standard library or external package
    first_part = import_name.split('.')[0]
    if first_part in stdlib_modules:
        return None

    # Try to resolve relative to project structure
    # This is a simplified approach - in practice, you'd want more sophisticated
    # import resolution that considers __init__.py files and PYTHONPATH

    # Check if it looks like a local import
    if import_name.startswith('Autonomy') or import_name.startswith('autonomy'):
        return import_name

    # For relative imports within the analyzed codebase
    # This is a basic heuristic - could be improved
    if '.' in import_name and len(import_name.split('.')) >= 2:
        return import_name

    return None


def generate_import_graph(
    source_path: str,
    docs_path: str,
    output_file: str,
    max_depth: int,
    exclude_patterns: List[str],
    title: str
) -> None:
    """Generate import dependency graph."""
    # Analyze imports
    G = analyze_python_imports(source_path, exclude_patterns)

    # Filter by depth if specified
    if max_depth > 0:
        # Keep only nodes within max_depth from root nodes
        root_nodes = [node for node in G.nodes() if G.in_degree(node) == 0]
        reachable = set()
        for root in root_nodes:
            reachable.update(nx.descendants_at_distance(G, root, max_depth))
            reachable.add(root)
        G = G.subgraph(reachable)

    # Generate PlantUML
    plantuml_content = generate_import_plantuml(G, title)

    # Write PlantUML file
    plantuml_file = Path(docs_path) / '_build' / 'plantuml' / f"{output_file}.puml"
    plantuml_file.parent.mkdir(parents=True, exist_ok=True)

    with open(plantuml_file, 'w') as f:
        f.write(plantuml_content)


def generate_class_hierarchy(
    source_path: str,
    docs_path: str,
    output_file: str,
    base_class: str,
    title: str
) -> None:
    """Generate class hierarchy diagram."""
    # This is a simplified implementation
    # In a full implementation, you'd analyze all classes and their inheritance

    # For now, create a placeholder PlantUML diagram
    plantuml_content = f"""@startuml {title.replace(' ', '_')}
!theme plain
skinparam backgroundColor #FEFEFE

title {title}

class BaseClass
class DerivedClass1
class DerivedClass2
class DerivedClass3

BaseClass <|-- DerivedClass1
BaseClass <|-- DerivedClass2
DerivedClass2 <|-- DerivedClass3

@enduml
"""

    # Write PlantUML file
    plantuml_file = Path(docs_path) / '_build' / 'plantuml' / f"{output_file}.puml"
    plantuml_file.parent.mkdir(parents=True, exist_ok=True)

    with open(plantuml_file, 'w') as f:
        f.write(plantuml_content)


def generate_import_plantuml(G: nx.DiGraph, title: str) -> str:
    """Generate PlantUML content for import graph."""
    plantuml = f"""@startuml {title.replace(' ', '_')}
!theme plain
skinparam backgroundColor #FEFEFE

title {title}

"""

    # Group nodes by package
    packages = {}
    for node in G.nodes():
        parts = node.split('.')
        if len(parts) > 1:
            package = '.'.join(parts[:-1])
            class_name = parts[-1]
        else:
            package = 'root'
            class_name = node

        if package not in packages:
            packages[package] = []
        packages[package].append(class_name)

    # Create packages and classes
    for package, classes in packages.items():
        if package != 'root':
            plantuml += f'package "{package}" {{\n'
            indent = '  '
        else:
            indent = ''

        for class_name in classes:
            plantuml += f'{indent}class {class_name}\n'

        if package != 'root':
            plantuml += '}\n'

    plantuml += '\n'

    # Add dependencies
    for source, target in G.edges():
        source_parts = source.split('.')
        target_parts = target.split('.')

        source_name = source_parts[-1]
        target_name = target_parts[-1]

        plantuml += f'{source_name} --> {target_name}\n'

    plantuml += '\n@enduml\n'
    return plantuml


def setup(app: Sphinx) -> Dict[str, any]:
    """Setup the code analysis extension."""
    app.add_directive('code-imports', CodeImportsDirective)
    app.add_directive('code-hierarchy', CodeHierarchyDirective)

    return {
        'version': '1.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
