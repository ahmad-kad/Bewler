#!/usr/bin/env python3
"""
ROS2 Graph Generation Extension for Sphinx Documentation

This extension automatically generates ROS2 system graphs and node/topic visualizations
from running ROS2 systems or saved introspection data.

Usage in RST files:
    .. ros2-nodes::
       :output: nodes_diagram.png

    .. ros2-topics::
       :output: topics_diagram.png

    .. ros2-services::
       :output: services_diagram.png
"""

import os
import re
import subprocess
from pathlib import Path
from typing import Dict, List, Set, Tuple

from docutils import nodes
from docutils.parsers.rst import Directive
from sphinx.application import Sphinx
from sphinx.util.docutils import SphinxDirective


class ROS2NodesDirective(SphinxDirective):
    """Directive to generate ROS2 node graph."""

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {
        'output': str,
        'format': str,
        'title': str,
    }

    def run(self) -> List[nodes.Node]:
        """Generate ROS2 nodes diagram."""
        output_file = self.options.get('output', 'ros2_nodes.png')
        output_format = self.options.get('output_format', 'png')
        title = self.options.get('title', 'ROS2 Node Graph')

        # Generate the diagram
        try:
            generate_ros2_nodes_graph(
                self.env.srcdir,
                output_file,
                output_format,
                title
            )
        except Exception as e:
            print(f"WARNING: Failed to generate ROS2 nodes graph: {e}")

        # Create figure node
        figure_node = nodes.figure()
        image_node = nodes.image(uri=output_file, alt=title)
        caption_node = nodes.caption(text=title)
        figure_node += image_node
        figure_node += caption_node

        return [figure_node]


class ROS2TopicsDirective(SphinxDirective):
    """Directive to generate ROS2 topic graph."""

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {
        'output': str,
        'format': str,
        'title': str,
    }

    def run(self) -> List[nodes.Node]:
        """Generate ROS2 topics diagram."""
        output_file = self.options.get('output', 'ros2_topics.png')
        output_format = self.options.get('output_format', 'png')
        title = self.options.get('title', 'ROS2 Topic Graph')

        # Generate the diagram
        try:
            generate_ros2_topics_graph(
                self.env.srcdir,
                output_file,
                output_format,
                title
            )
        except Exception as e:
            print(f"WARNING: Failed to generate ROS2 topics graph: {e}")

        # Create figure node
        figure_node = nodes.figure()
        image_node = nodes.image(uri=output_file, alt=title)
        caption_node = nodes.caption(text=title)
        figure_node += image_node
        figure_node += caption_node

        return [figure_node]


class ROS2ServicesDirective(SphinxDirective):
    """Directive to generate ROS2 services graph."""

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {
        'output': str,
        'format': str,
        'title': str,
    }

    def run(self) -> List[nodes.Node]:
        """Generate ROS2 services diagram."""
        output_file = self.options.get('output', 'ros2_services.png')
        output_format = self.options.get('output_format', 'png')
        title = self.options.get('title', 'ROS2 Services Graph')

        # Generate the diagram
        try:
            generate_ros2_services_graph(
                self.env.srcdir,
                output_file,
                output_format,
                title
            )
        except Exception as e:
            print(f"WARNING: Failed to generate ROS2 services graph: {e}")

        # Create figure node
        figure_node = nodes.figure()
        image_node = nodes.image(uri=output_file, alt=title)
        caption_node = nodes.caption(text=title)
        figure_node += image_node
        figure_node += caption_node

        return [figure_node]


def generate_ros2_nodes_graph(srcdir: str, output_file: str, output_format: str, title: str) -> None:
    """Generate ROS2 nodes graph using ros2 node list and graph generation tools."""
    try:
        # Get list of nodes
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode != 0:
            raise RuntimeError(f"ros2 node list failed: {result.stderr}")

        nodes_list = result.stdout.strip().split('\n')
        nodes_list = [node.strip() for node in nodes_list if node.strip()]

        # Generate PlantUML diagram
        plantuml_content = generate_nodes_plantuml(nodes_list, title)

        # Write PlantUML file
        plantuml_file = Path(srcdir) / '_build' / 'plantuml' / f"{output_file}.puml"
        plantuml_file.parent.mkdir(parents=True, exist_ok=True)

        with open(plantuml_file, 'w') as f:
            f.write(plantuml_content)

        # Convert to image (assuming PlantUML is available)
        # This would be handled by sphinxcontrib-plantuml

    except subprocess.TimeoutExpired:
        raise RuntimeError("ROS2 command timed out - is ROS2 running?")
    except FileNotFoundError:
        raise RuntimeError("ROS2 tools not found - is ROS2 installed and sourced?")


def generate_ros2_topics_graph(srcdir: str, output_file: str, output_format: str, title: str) -> None:
    """Generate ROS2 topics graph."""
    try:
        # Get list of topics
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode != 0:
            raise RuntimeError(f"ros2 topic list failed: {result.stderr}")

        topics_list = result.stdout.strip().split('\n')
        topics_list = [topic.strip() for topic in topics_list if topic.strip()]

        # Generate PlantUML diagram
        plantuml_content = generate_topics_plantuml(topics_list, title)

        # Write PlantUML file
        plantuml_file = Path(srcdir) / '_build' / 'plantuml' / f"{output_file}.puml"
        plantuml_file.parent.mkdir(parents=True, exist_ok=True)

        with open(plantuml_file, 'w') as f:
            f.write(plantuml_content)

    except subprocess.TimeoutExpired:
        raise RuntimeError("ROS2 command timed out - is ROS2 running?")
    except FileNotFoundError:
        raise RuntimeError("ROS2 tools not found - is ROS2 installed and sourced?")


def generate_ros2_services_graph(srcdir: str, output_file: str, output_format: str, title: str) -> None:
    """Generate ROS2 services graph."""
    try:
        # Get list of services
        result = subprocess.run(
            ['ros2', 'service', 'list'],
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode != 0:
            raise RuntimeError(f"ros2 service list failed: {result.stderr}")

        services_list = result.stdout.strip().split('\n')
        services_list = [service.strip() for service in services_list if service.strip()]

        # Generate PlantUML diagram
        plantuml_content = generate_services_plantuml(services_list, title)

        # Write PlantUML file
        plantuml_file = Path(srcdir) / '_build' / 'plantuml' / f"{output_file}.puml"
        plantuml_file.parent.mkdir(parents=True, exist_ok=True)

        with open(plantuml_file, 'w') as f:
            f.write(plantuml_content)

    except subprocess.TimeoutExpired:
        raise RuntimeError("ROS2 command timed out - is ROS2 running?")
    except FileNotFoundError:
        raise RuntimeError("ROS2 tools not found - is ROS2 installed and sourced?")


def generate_nodes_plantuml(nodes_list: List[str], title: str) -> str:
    """Generate PlantUML content for ROS2 nodes."""
    plantuml = f"""@startuml {title.replace(' ', '_')}
!theme plain
skinparam backgroundColor #FEFEFE

title {title}

"""

    # Group nodes by namespace/package
    node_groups = {}
    for node in nodes_list:
        if '/' in node:
            namespace, name = node.rsplit('/', 1)
            if namespace not in node_groups:
                node_groups[namespace] = []
            node_groups[namespace].append(name)
        else:
            if 'root' not in node_groups:
                node_groups['root'] = []
            node_groups['root'].append(node)

    # Create diagram
    for namespace, nodes_in_group in node_groups.items():
        if namespace != 'root':
            plantuml += f'package "{namespace}" {{\n'
            indent = '  '
        else:
            indent = ''

        for node in nodes_in_group:
            plantuml += f'{indent}node "{node}" as {node.replace("/", "_")}\n'

        if namespace != 'root':
            plantuml += '}\n'

    plantuml += '\n@enduml\n'
    return plantuml


def generate_topics_plantuml(topics_list: List[str], title: str) -> str:
    """Generate PlantUML content for ROS2 topics."""
    plantuml = f"""@startuml {title.replace(' ', '_')}
!theme plain
skinparam backgroundColor #FEFEFE

title {title}

"""

    # Create topic rectangles
    for topic in topics_list:
        # Clean topic name for PlantUML
        clean_name = topic.replace('/', '_').replace('-', '_')
        plantuml += f'rectangle "{topic}" as {clean_name}\n'

    plantuml += '\n@enduml\n'
    return plantuml


def generate_services_plantuml(services_list: List[str], title: str) -> str:
    """Generate PlantUML content for ROS2 services."""
    plantuml = f"""@startuml {title.replace(' ', '_')}
!theme plain
skinparam backgroundColor #FEFEFE

title {title}

"""

    # Create service circles
    for service in services_list:
        # Clean service name for PlantUML
        clean_name = service.replace('/', '_').replace('-', '_')
        plantuml += f'circle "{service}" as {clean_name}\n'

    plantuml += '\n@enduml\n'
    return plantuml


def setup(app: Sphinx) -> Dict[str, any]:
    """Setup the ROS2 graphs extension."""
    app.add_directive('ros2-nodes', ROS2NodesDirective)
    app.add_directive('ros2-topics', ROS2TopicsDirective)
    app.add_directive('ros2-services', ROS2ServicesDirective)

    return {
        'version': '1.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
