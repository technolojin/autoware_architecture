import fcntl
import os
from pathlib import Path
import logging

logger = logging.getLogger(__name__)

def get_install_root(path: Path) -> Path:
    """
    Find the install root directory from a given path.
    Assumes the path is somewhere inside an 'install' directory.
    """
    path = path.resolve()
    parts = path.parts

    # Look for 'install' in the path from right to left
    # This handles cases where the workspace itself might be in a directory named 'install'
    # But typically we want the one that structures the package layout.
    # Standard layout: .../install/<pkg>/share/<pkg>/...
    # or .../install/share/<pkg>/...

    # We'll assume the 'install' directory we care about is the one closest to the workspace root
    # but strictly speaking, we just need *a* common root to place the index.

    # Simple heuristic: search for 'install'
    if 'install' in parts:
        # Find the index of 'install'
        # If there are multiple, we probably want the one that is part of the current build workspace
        # usually the last one?
        # /home/user/workspace/install/pkg/... -> index is len-3
        # If /home/user/install/workspace/install/pkg -> we want the last one.

        try:
            # finding the last occurrence of 'install'
            idx = len(parts) - 1 - parts[::-1].index('install')
            return Path(*parts[:idx+1])
        except ValueError:
            pass

    return None

def update_index(output_root_dir: str):
    """
    Update the architecture_visualization.html index file in the install root.
    Uses file locking to safely handle concurrent builds.
    """
    output_path = Path(output_root_dir).resolve()
    install_root = get_install_root(output_path)

    if not install_root or not install_root.exists():
        logger.warning(f"Could not determine install root from {output_root_dir}. Skipping index update.")
        return

    index_file = install_root / "architecture_visualization.html"
    lock_file = install_root / ".architecture_visualization_index.lock"

    # Ensure we can write to lock file
    try:
        with open(lock_file, 'w') as lock:
            try:
                # Acquire exclusive lock
                fcntl.flock(lock, fcntl.LOCK_EX)

                # Now we have the lock, regenerate the index
                _generate_index_file(install_root, index_file)

            finally:
                # Release lock
                fcntl.flock(lock, fcntl.LOCK_UN)
    except Exception as e:
        logger.error(f"Failed to update visualization index: {e}")

def _generate_deployment_overview_pages(install_root: Path, deployments):
    """Generate deployment overview pages that combine different diagram types."""
    from ..template_utils import TemplateRenderer

    renderer = TemplateRenderer()

    for dep in deployments:
        # Find available modes for this deployment
        visualization_root = install_root / dep['path'].parent.parent  # Go up from web/node_diagram.html
        available_modes = set()

        # Look for data files to determine available modes
        data_dir = visualization_root / "web" / "data"
        if data_dir.exists():
            for js_file in data_dir.glob("*_node_diagram.js"):
                mode = js_file.stem.replace("_node_diagram", "")
                available_modes.add(mode)
            for js_file in data_dir.glob("*_sequence_diagram.js"):
                mode = js_file.stem.replace("_sequence_diagram", "")
                available_modes.add(mode)

        available_modes = sorted(list(available_modes))
        default_mode = "default" if "default" in available_modes else (available_modes[0] if available_modes else "default")
        default_diagram_type = dep['diagram_types'][0] if dep['diagram_types'] else "node_diagram"

        # Prepare template data
        template_data = {
            "deployment_name": dep['name'],
            "package_name": dep['package'],
            "available_diagram_types": dep['diagram_types'],
            "available_modes": available_modes,
            "default_diagram_type": default_diagram_type,
            "default_mode": default_mode
        }

        # Generate the overview page
        overview_path = install_root / dep['path'].parent / f"{dep['name']}_overview.html"
        try:
            renderer.render_template_to_file("visualization/deployment_overview.html.jinja2", str(overview_path), **template_data)
            logger.info(f"Generated deployment overview page: {overview_path}")
        except Exception as e:
            logger.error(f"Failed to generate deployment overview page for {dep['name']}: {e}")

def _generate_index_file(install_root: Path, output_file: Path):
    deployments = []

    # Walk through the install directory to find deployments
    # We scan specifically for our known structure to avoid false positives
    for path in install_root.rglob("node_diagram.html"):
        try:
            # Expected path structure:
            # .../exports/<system_name>/visualization/web/node_diagram.html

            if len(path.parts) < 6:
                continue

            if path.parts[-2] == 'web' and path.parts[-3] == 'visualization':
                if path.parts[-5] == 'exports':
                    deployment_dir_name = path.parts[-4]
                    package_name = path.parts[-6] # .../share/<pkg>/exports/...

                    # Calculate relative path for the link
                    try:
                        rel_path = path.relative_to(install_root)
                    except ValueError:
                        continue

                    # Find all available diagram types
                    diagram_types = set()
                    # Looking in .../visualization/ (parent of web)
                    visualization_root = path.parents[1]
                    if visualization_root.exists():
                        # Look for node_diagram.html (already found via the rglob above)
                        diagram_types.add('node_diagram')

                        # Look for sequence_diagram.html file
                        sequence_diagram_path = visualization_root / "web" / "sequence_diagram.html"
                        if sequence_diagram_path.exists():
                            diagram_types.add('sequence_diagram')

                        # Could add more diagram types here in the future
                        # e.g., look for other diagram files

                    deployments.append({
                        'name': deployment_dir_name,
                        'package': package_name,
                        'path': rel_path,
                        'diagram_types': sorted(list(diagram_types))
                    })
        except IndexError:
            continue

    # Sort by package then system name
    deployments.sort(key=lambda x: (x['package'], x['name']))

    # Generate deployment overview pages
    _generate_deployment_overview_pages(install_root, deployments)

    # Generate HTML
    html_content = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Autoware Architecture Deployments</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f4f4f4;
            color: #333;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            padding: 30px;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #2c3e50;
            border-bottom: 2px solid #eee;
            padding-bottom: 10px;
        }
        .description {
            color: #666;
            margin-bottom: 30px;
        }
        .deployment-list {
            list-style-type: none;
            padding: 0;
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            gap: 20px;
        }
        .deployment-item {
            background: #fff;
            border: 1px solid #e0e0e0;
            border-radius: 6px;
            transition: transform 0.2s, box-shadow 0.2s;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }
        .deployment-item:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.05);
            border-color: #b0b0b0;
        }
        .deployment-header {
            padding: 20px 20px 15px 20px;
        }
        .diagram-buttons {
            display: flex;
            gap: 10px;
            padding: 0 20px 20px 20px;
            border-top: 1px solid #f0f0f0;
            background: #fafafa;
        }
        .diagram-button {
            text-decoration: none;
            color: #0066cc;
            background: #f8f9fa;
            border: 1px solid #dee2e6;
            border-radius: 4px;
            padding: 8px 12px;
            font-size: 0.9em;
            font-weight: 500;
            transition: all 0.2s;
            display: flex;
            align-items: center;
            gap: 6px;
        }
        .diagram-button:hover {
            background: #e9ecef;
            border-color: #adb5bd;
            color: #0056b3;
        }
        .diagram-icon {
            display: flex;
            align-items: center;
        }
        .deployment-name {
            font-weight: bold;
            font-size: 1.1em;
            color: #0066cc;
            margin-bottom: 10px;
            word-break: break-word;
        }
        .deployment-meta {
            color: #666;
            font-size: 0.9em;
            margin-bottom: 10px;
        }
        .deployment-package {
            display: inline-block;
            background: #f0f0f0;
            padding: 2px 6px;
            border-radius: 4px;
            font-family: monospace;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Autoware Architecture Deployments</h1>
        <p class="description">Index of available system architecture visualizations found in the <code>install/</code> directory.</p>
        <ul class="deployment-list">
"""

    for dep in deployments:
        html_content += f"""
            <li class="deployment-item">
                <div class="deployment-header">
                    <div class="deployment-name">{dep['name']}</div>
                    <div class="deployment-meta">Package: <span class="deployment-package">{dep['package']}</span></div>
                </div>
                <div class="diagram-buttons">"""

        for diagram_type in dep['diagram_types']:
            diagram_label = diagram_type.replace('_', ' ').title()
            # Create a link to a unified deployment page with diagram type parameter
            deployment_overview_path = dep['path'].parent / f"{dep['name']}_overview.html"
            html_content += f"""
                    <a class="diagram-button" href="{deployment_overview_path}?diagram={diagram_type}" data-type="{diagram_type}">
                        <span class="diagram-icon">{diagram_label}</span>
                    </a>"""

        html_content += """
                </div>
            </li>"""

    html_content += """
        </ul>
    </div>
</body>
</html>
"""

    with open(output_file, 'w') as f:
        f.write(html_content)
