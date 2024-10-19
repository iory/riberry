#!/usr/bin/env python3

import argparse
import sys


def generate_html(template_path, output_path, namespace):
    with open(template_path) as template_file:
        template = template_file.read()

    if namespace:
        namespace = "/" + namespace
    html_content = template.replace("{{namespace}}", namespace)
    with open(output_path, "w") as output_file:
        output_file.write(html_content)


if __name__ == "__main__":
    # Suppress ROS-specific args
    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith("__")]

    parser = argparse.ArgumentParser(
        description="Generate HTML from template with optional namespace."
    )
    parser.add_argument("template_path", type=str, help="Path to the HTML template file.")
    parser.add_argument("output_path", type=str, help="Path to the output HTML file.")
    parser.add_argument(
        "--namespace",
        type=str,
        default="",
        help="Optional namespace to replace in the template.",
    )

    args = parser.parse_args(filtered_args)

    generate_html(args.template_path, args.output_path, args.namespace)
