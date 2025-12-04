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

                    # Find associated sequence diagrams
                    sequence_diagrams = []
                    # Looking in .../visualization/ (parent of web)
                    visualization_root = path.parents[1]
                    if visualization_root.exists():
                        # Look recursively for _sequence_graph.html files
                        for seq_path in visualization_root.rglob("*_sequence_graph.html"):
                             try:
                                 seq_rel_path = seq_path.relative_to(install_root)
                                 # Extract mode from filename if possible, or directory
                                 # Structure: visualization/<mode>/<name>_sequence_graph.html
                                 # Filename: <name>_<mode>_sequence_graph.html or <name>_sequence_graph.html
                                 mode_name = seq_path.parent.name
                                 sequence_diagrams.append({
                                     'name': mode_name,
                                     'path': seq_rel_path
                                 })
                             except ValueError:
                                 continue

                    sequence_diagrams.sort(key=lambda x: x['name'])

                    deployments.append({
                        'name': deployment_dir_name,
                        'package': package_name,
                        'path': rel_path,
                        'sequence_diagrams': sequence_diagrams
                    })
        except IndexError:
            continue

    # Sort by package then system name
    deployments.sort(key=lambda x: (x['package'], x['name']))

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
        }
        .deployment-item:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.05);
            border-color: #b0b0b0;
        }
        .deployment-link {
            text-decoration: none;
            color: inherit;
            display: block;
            padding: 20px;
            flex-grow: 1;
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
        .sequence-links {
            padding: 0 20px 20px 20px;
            border-top: 1px solid #eee;
            font-size: 0.9em;
        }
        .sequence-title {
            margin: 10px 0 5px 0;
            font-weight: 600;
            color: #555;
        }
        .sequence-link-item {
            margin-bottom: 4px;
        }
        .sequence-link-item a {
            color: #0066cc;
            text-decoration: none;
        }
        .sequence-link-item a:hover {
            text-decoration: underline;
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
                <a class="deployment-link" href="{dep['path']}">
                    <div class="deployment-name">{dep['name']}</div>
                    <div class="deployment-meta">Package: <span class="deployment-package">{dep['package']}</span></div>
                    <div class="deployment-meta" style="font-size: 0.8em; color: #888;">Node Diagram (Main)</div>
                </a>"""

        if dep['sequence_diagrams']:
            html_content += """
                <div class="sequence-links">
                    <div class="sequence-title">Sequence Diagrams:</div>"""
            for seq in dep['sequence_diagrams']:
                html_content += f"""
                    <div class="sequence-link-item">
                        <a href="{seq['path']}">{seq['name']}</a>
                    </div>"""
            html_content += """
                </div>"""

        html_content += """
            </li>"""

    html_content += """
        </ul>
    </div>
</body>
</html>
"""

    with open(output_file, 'w') as f:
        f.write(html_content)
